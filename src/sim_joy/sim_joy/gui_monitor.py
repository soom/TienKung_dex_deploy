#!/usr/bin/env python3
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Optional
import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

try:
    from inputs import UnpluggedError, get_gamepad
    HAVE_INPUTS = True
except ImportError:
    HAVE_INPUTS = False
    get_gamepad = None

try:
    from pynput import keyboard as pynput_keyboard
    HAVE_PYNPUT = True
except ImportError:
    HAVE_PYNPUT = False
    pynput_keyboard = None

try:
    from bodyctrl_msgs.msg import SbusData
    HAVE_SBUS = True
except ImportError:
    HAVE_SBUS = False
    SbusData = None

HAVE_CONSOLE = sys.stdin.isatty()


# ---------------------------------------------------------------------------
# TeleopJoyNode  (formerly teleop_joy.py)
# ---------------------------------------------------------------------------
class TeleopJoyNode(Node):
    """Publishes /sbus_data Joy messages with Tiangong-specific axis mapping."""

    AXES_LEN = 12
    AXIS_KEYS = frozenset({'1', '2', '3', '4', '6', '7', '8', '9'})
    AXIS_OVERRIDE_INDEX = {
        'y2': 0,
        'turn': 1,
        'y1': 2,
        'x1': 3,
    }
    SWITCH_SEQUENCES = {
        'e': ('button_e', [('KEY_E_UP', -1), ('KEY_E_MID', 0), ('KEY_E_DOWN', 1)]),
        'f': ('button_f', [('KEY_F_UP', -1), ('KEY_F_MID', 0), ('KEY_F_DOWN', 1)]),
        'g': ('button_g', [('KEY_G_LEFT', 1), ('KEY_G_MID', 0), ('KEY_G_RIGHT', -1)]),
        'h': ('button_h', [('KEY_H_LEFT', 1), ('KEY_H_MID', 0), ('KEY_H_RIGHT', -1)]),
    }

    def __init__(self, *, enable_gamepad=None, enable_keyboard=None, enable_console=None):
        super().__init__('sim_joy')
        self.publisher_ = self.create_publisher(Joy, '/sbus_data', 10)
        self._sbus_available = HAVE_SBUS
        self.sbus_publisher = None
        if HAVE_SBUS:
            try:
                self.sbus_publisher = self.create_publisher(SbusData, '/sbus_data/event', 10)
            except Exception as exc:
                self._sbus_available = False
                self.get_logger().warn(f'SbusData unavailable: {exc}')

        self.declare_parameters('', [
            ('publish_rate', 30.0),
            ('deadzone', 0.15),
            ('gamepad_timeout', 1.0),
            ('key_axis_gain', 0.5),
        ])
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.gamepad_timeout = float(self.get_parameter('gamepad_timeout').value)
        self.key_axis_gain = float(self.get_parameter('key_axis_gain').value)

        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._gamepad_state = {
            'forward': 0.0, 'lateral': 0.0, 'turn': 0.0, 'y2': 0.0,
            'buttons': {k: 0.0 for k in ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start')},
            'connected': False,
        }
        self._axis_keys_pressed: set = set()
        self._button_states: Dict[str, bool] = {k: False for k in ('a', 'b', 'c', 'd')}
        self._switch_states = {k: 0 for k in self.SWITCH_SEQUENCES}
        self._switch_indices = {k: 1 for k in self.SWITCH_SEQUENCES}
        self._axis_overrides: Dict[str, Optional[float]] = {n: None for n in self.AXIS_OVERRIDE_INDEX}
        self._key_event_new = self._get_key_constant('KEY_NONE')
        self._key_event_old = self._get_key_constant('KEY_NONE')
        self._last_gamepad_event = 0.0
        self._keyboard_listener = None
        self._console_thread = None

        if enable_gamepad is None:
            enable_gamepad = True
        self._gamepad_enabled = enable_gamepad and HAVE_INPUTS
        if self._gamepad_enabled:
            threading.Thread(target=self._gamepad_loop, daemon=True).start()

        if enable_keyboard is None:
            enable_keyboard = True
        self._keyboard_enabled = enable_keyboard and HAVE_PYNPUT
        if self._keyboard_enabled:
            self._keyboard_listener = pynput_keyboard.Listener(
                on_press=lambda key: self._handle_key_event(key, True),
                on_release=lambda key: self._handle_key_event(key, False),
            )
            self._keyboard_listener.daemon = True
            self._keyboard_listener.start()

        if enable_console is None:
            enable_console = not self._keyboard_enabled
        self._console_enabled = enable_console and HAVE_CONSOLE
        if self._console_enabled:
            self._console_thread = threading.Thread(target=self._console_loop, daemon=True)
            self._console_thread.start()

        self.create_timer(self.publish_period, self._publish)

    def destroy_node(self):
        self._stop_event.set()
        if self._keyboard_listener:
            self._keyboard_listener.stop()
        return super().destroy_node()

    def _get_key_constant(self, name):
        if not self._sbus_available or SbusData is None:
            return 0
        return int(getattr(SbusData, name, 0))

    def _emit_key_event(self, const_name):
        if not self._sbus_available:
            return
        v = self._get_key_constant(const_name)
        if v:
            self._key_event_old = self._key_event_new
            self._key_event_new = v

    def _canonical_key(self, ident):
        if ident is None:
            return None
        ident = ident.lower()
        if ident.startswith('num_'):
            ident = ident[4:]
        alias = {'up': '8', 'down': '2', 'left': '4', 'right': '6',
                 'home': '7', 'page_up': '9', 'end': '1', 'page_down': '3'}
        return alias.get(ident, ident)

    def _cycle_switch(self, key):
        field, seq = self.SWITCH_SEQUENCES[key]
        idx = (self._switch_indices[key] + 1) % len(seq)
        self._switch_indices[key] = idx
        const_name, value = seq[idx]
        self._switch_states[key] = value
        self._emit_key_event(const_name)

    def _axis_value_from_keys(self, positives, negatives):
        pos = any(k in self._axis_keys_pressed for k in positives)
        neg = any(k in self._axis_keys_pressed for k in negatives)
        return self.key_axis_gain * (float(pos) - float(neg))

    def _gamepad_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                events = get_gamepad()
            except UnpluggedError:
                with self._state_lock:
                    self._gamepad_state['connected'] = False
                time.sleep(0.5)
                continue
            except Exception:
                time.sleep(0.1)
                continue
            with self._state_lock:
                for e in events:
                    self._process_gamepad_event(e.code, e.state)
                self._gamepad_state['connected'] = True
                self._last_gamepad_event = time.monotonic()

    def _process_gamepad_event(self, code, value):
        s = 32768.0
        m = {'ABS_X': ('lateral', 1), 'ABS_Y': ('forward', 1),
             'ABS_RX': ('turn', 1), 'ABS_RY': ('y2', 1)}
        if code in m:
            k, _ = m[code]
            self._gamepad_state[k] = self._clamp(value / s)
        btn = {'BTN_SOUTH': 'a', 'BTN_EAST': 'b', 'BTN_WEST': 'x', 'BTN_NORTH': 'y',
               'BTN_TL': 'lb', 'BTN_TR': 'rb', 'BTN_SELECT': 'back', 'BTN_START': 'start'}
        if code in btn:
            self._gamepad_state['buttons'][btn[code]] = float(value)

    def _handle_key_event(self, key, pressed):
        ident = self._canonical_key(self._key_identifier(key))
        self._update_key_state(ident, pressed)

    def _update_key_state(self, ident, pressed):
        if ident is None:
            return False
        with self._state_lock:
            if ident in self.AXIS_KEYS:
                (self._axis_keys_pressed.add if pressed else self._axis_keys_pressed.discard)(ident)
                return True
            if ident in self._button_states:
                self._button_states[ident] = pressed
                self._emit_key_event(f"KEY_{ident.upper()}_{'DOWN' if pressed else 'UP'}")
                return True
            if ident in self.SWITCH_SEQUENCES and pressed:
                self._cycle_switch(ident)
                return True
        return False

    def set_axis_override(self, axis_name, value):
        axis_name = axis_name.lower()
        if axis_name not in self.AXIS_OVERRIDE_INDEX:
            raise ValueError(f'Unknown axis "{axis_name}"')
        with self._state_lock:
            self._axis_overrides[axis_name] = None if value is None else float(self._clamp(value))

    def set_button_state(self, button, pressed):
        self._update_key_state(button.lower(), pressed)

    def set_switch_state(self, key, position):
        key = key.lower()
        if isinstance(position, str):
            if not self._set_switch_position(key, position):
                raise ValueError('Switch position must be up/mid/down or left/mid/right')
            return
        mapping = {-1: 'up', 0: 'mid', 1: 'down'} if key in {'e', 'f'} else {-1: 'left', 0: 'mid', 1: 'right'}
        label = mapping.get(int(position))
        if not label or not self._set_switch_position(key, label):
            raise ValueError('Switch value out of range')

    def _set_switch_position(self, key, target):
        target = target.lower()
        options = {
            'e': {'up': 0, 'mid': 1, 'down': 2}, 'f': {'up': 0, 'mid': 1, 'down': 2},
            'g': {'left': 0, 'mid': 1, 'right': 2}, 'h': {'left': 0, 'mid': 1, 'right': 2},
        }
        idx_map = options.get(key)
        if not idx_map or target not in idx_map:
            return False
        idx = idx_map[target]
        const_name, value = self.SWITCH_SEQUENCES[key][1][idx]
        with self._state_lock:
            self._switch_indices[key] = idx
            self._switch_states[key] = value
        self._emit_key_event(const_name)
        return True

    def _console_loop(self):
        while not self._stop_event.is_set():
            try:
                line = input('teleop> ')
            except (EOFError, KeyboardInterrupt):
                break
            cmd = line.strip()
            if cmd:
                self._handle_console_command(cmd)

    def _handle_console_command(self, text):
        parts = text.lower().split()
        if not parts:
            return
        cmd = parts[0]
        if cmd in ('press', 'release') and len(parts) == 2:
            self._update_key_state(parts[1], cmd == 'press')
        elif cmd == 'switch' and len(parts) >= 2:
            key = parts[1]
            if key in self.SWITCH_SEQUENCES:
                if len(parts) == 2:
                    self._cycle_switch(key)
                else:
                    self._set_switch_position(key, parts[2])

    @staticmethod
    def _key_identifier(key):
        try:
            if key.char:
                return key.char.lower()
        except AttributeError:
            pass
        try:
            return key.name
        except AttributeError:
            return None

    def _publish(self):
        now = time.monotonic()
        with self._state_lock:
            gamepad_axes = self._axes_from_gamepad(now)
            keyboard_axes = self._axes_from_keyboard()
        merged = self._merge_axes(gamepad_axes, keyboard_axes)
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joystick'
        msg.axes = merged
        msg.buttons = []
        self.publisher_.publish(msg)
        self._publish_sbus(merged)

    def _axes_from_gamepad(self, now):
        axes = [0.0] * self.AXES_LEN
        gs = self._gamepad_state
        if not gs['connected'] or now - self._last_gamepad_event > self.gamepad_timeout:
            gs['connected'] = False
            return axes
        axes[0] = self._apply_deadzone(-gs['y2'])
        axes[1] = self._apply_deadzone(-gs['turn'])
        axes[2] = self._apply_deadzone(-gs['forward'])
        axes[3] = self._apply_deadzone(-gs['lateral'])
        b = gs['buttons']
        axes[4] = b['lb']; axes[5] = b['back']; axes[6] = b['start']; axes[7] = b['rb']
        axes[8] = b['a'];  axes[9] = b['b'];    axes[10] = b['x'];    axes[11] = b['y']
        return axes

    def _axes_from_keyboard(self):
        axes = [0.0] * self.AXES_LEN
        axes[0] = self._axis_value_from_keys(('3',), ('1',))
        axes[1] = self._axis_value_from_keys(('9',), ('7',))
        axes[2] = self._axis_value_from_keys(('8',), ('2',))
        axes[3] = self._axis_value_from_keys(('6',), ('4',))
        axes[4] = self._switch_states['e']
        axes[5] = self._switch_states['g']
        axes[6] = self._switch_states['h']
        axes[7] = self._switch_states['f']
        axes[8]  = 1.0 if self._button_states['a'] else 0.0
        axes[9]  = 1.0 if self._button_states['b'] else 0.0
        axes[10] = 1.0 if self._button_states['c'] else 0.0
        axes[11] = 1.0 if self._button_states['d'] else 0.0
        for name, idx in self.AXIS_OVERRIDE_INDEX.items():
            ov = self._axis_overrides.get(name)
            if ov is not None:
                axes[idx] = ov
        return axes

    def _merge_axes(self, primary, fallback):
        return [float(self._clamp(p if abs(p) > 1e-6 else f)) for p, f in zip(primary, fallback)]

    def _publish_sbus(self, axes):
        if not self._sbus_available or not self.sbus_publisher:
            return
        msg = SbusData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.key_event_old = self._key_event_old
        msg.key_event_new = self._key_event_new
        msg.button_a = 1 if self._button_states['a'] else -1
        msg.button_b = 1 if self._button_states['b'] else -1
        msg.button_c = 1 if self._button_states['c'] else -1
        msg.button_d = 1 if self._button_states['d'] else -1
        msg.button_e = int(self._switch_states['e'])
        msg.button_f = int(self._switch_states['f'])
        msg.button_g = int(self._switch_states['g'])
        msg.button_h = int(self._switch_states['h'])
        msg.x1 = float(self._clamp(axes[3]))
        msg.y1 = float(self._clamp(axes[2]))
        msg.x2 = float(self._clamp(axes[1]))
        msg.y2 = float(self._clamp(axes[0]))
        self.sbus_publisher.publish(msg)

    def _apply_deadzone(self, v):
        return 0.0 if abs(v) < self.deadzone else self._clamp(v)

    @staticmethod
    def _clamp(v, limit=1.0):
        return max(-limit, min(limit, v))


# ---------------------------------------------------------------------------
# TeleopGuiMonitorNode  — subscribes to telemetry topics for GUI display
# ---------------------------------------------------------------------------
class TeleopGuiMonitorNode(Node):

    def __init__(self) -> None:
        super().__init__('teleop_gui_monitor')
        self._lock = threading.Lock()
        self._axes: List[float] = [0.0] * 12
        self._joy_stamp = None
        self._sbus_state = None
        self._elastic_state: str = 'suspended'
        self._elastic_length: float = 0.0
        self._sbus_available = HAVE_SBUS
        self._sbus_sub = None

        self.create_subscription(Joy, '/sbus_data', self._joy_cb, 10)
        self._elastic_cmd_pub = self.create_publisher(String, '/elastic_band_cmd', 10)
        self.create_subscription(String, '/elastic_band_status', self._elastic_cb, 10)

        if HAVE_SBUS:
            try:
                self._sbus_sub = self.create_subscription(
                    SbusData, '/sbus_data/event', self._sbus_cb, 10)
            except Exception as exc:
                self._sbus_available = False
                self.get_logger().warn(f'SbusData monitor disabled: {exc}')

    def _joy_cb(self, msg: Joy) -> None:
        with self._lock:
            self._axes = list(msg.axes)
            self._joy_stamp = msg.header.stamp

    def _sbus_cb(self, msg) -> None:
        with self._lock:
            self._sbus_state = msg

    def _elastic_cb(self, msg: String) -> None:
        parts = msg.data.split(':')
        with self._lock:
            self._elastic_state = parts[0] if parts else 'unknown'
            self._elastic_length = float(parts[1]) if len(parts) > 1 else 0.0

    def send_elastic_cmd(self, cmd: str) -> None:
        m = String(); m.data = cmd
        self._elastic_cmd_pub.publish(m)

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return {
                'axes': list(self._axes),
                'stamp': self._joy_stamp,
                'sbus': self._sbus_state,
                'sbus_enabled': self._sbus_available and self._sbus_sub is not None,
                'elastic_state': self._elastic_state,
                'elastic_length': self._elastic_length,
            }


# ---------------------------------------------------------------------------
# TeleopGuiApp  — Tkinter GUI
# ---------------------------------------------------------------------------
class TeleopGuiApp:
    AXIS_WIDGETS = [('y1', 'Y1'), ('x1', 'X1'), ('y2', 'Y2'), ('turn', 'X2')]
    BUTTON_ORDER = ['a', 'b', 'c', 'd']
    SWITCH_OPTIONS = {
        'e': ('up', 'mid', 'down'), 'f': ('up', 'mid', 'down'),
        'g': ('left', 'mid', 'right'), 'h': ('left', 'mid', 'right'),
    }
    BUTTON_FIELDS = ['button_a', 'button_b', 'button_c', 'button_d']
    SWITCH_FIELDS = ['button_e', 'button_f', 'button_g', 'button_h']

    def __init__(self) -> None:
        self._closed = False
        rclpy.init()
        self.teleop_node = TeleopJoyNode(
            enable_gamepad=False, enable_keyboard=False, enable_console=False)
        self.monitor_node = TeleopGuiMonitorNode()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.teleop_node)
        self.executor.add_node(self.monitor_node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        self.root = tk.Tk()
        self.root.title('Tiangong Teleop GUI')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        signal.signal(signal.SIGINT, lambda s, f: self.close())
        self._build_layout()
        self._schedule_update()

    def _build_layout(self) -> None:
        self.root.geometry('820x420')
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill=tk.BOTH, expand=True)

        # ---- Teleop Input ----
        ctrl = ttk.LabelFrame(main, text='Teleop Input')
        ctrl.pack(fill=tk.X)

        self.axis_scales: Dict[str, tk.Scale] = {}
        for idx, (axis, label) in enumerate(self.AXIS_WIDGETS):
            s = tk.Scale(ctrl, from_=1.0, to=-1.0, orient=tk.VERTICAL,
                         resolution=0.01, length=140, label=label,
                         command=lambda v, n=axis: self.teleop_node.set_axis_override(n, float(v)))
            s.set(0.0)
            s.grid(row=0, column=idx, padx=5, pady=3)
            self.axis_scales[axis] = s

        btn_f = ttk.Frame(ctrl)
        btn_f.grid(row=0, column=len(self.AXIS_WIDGETS), padx=10)
        ttk.Label(btn_f, text='Buttons').pack(anchor='w')
        self.button_vars: Dict[str, tk.BooleanVar] = {}
        for b in self.BUTTON_ORDER:
            var = tk.BooleanVar()
            ttk.Checkbutton(btn_f, text=b.upper(), variable=var,
                            command=lambda n=b, v=var: self.teleop_node.set_button_state(n, v.get())).pack(anchor='w')
            self.button_vars[b] = var

        sw_f = ttk.Frame(ctrl)
        sw_f.grid(row=0, column=len(self.AXIS_WIDGETS) + 1, padx=10)
        ttk.Label(sw_f, text='Switches').grid(row=0, column=0, columnspan=2, sticky='w')
        self.switch_vars: Dict[str, tk.StringVar] = {}
        for row, (key, opts) in enumerate(self.SWITCH_OPTIONS.items(), 1):
            ttk.Label(sw_f, text=key.upper()).grid(row=row, column=0, sticky='e', padx=2, pady=1)
            var = tk.StringVar(value='mid')
            cb = ttk.Combobox(sw_f, values=opts, state='readonly', width=6, textvariable=var)
            cb.grid(row=row, column=1, padx=2, pady=1)
            cb.bind('<<ComboboxSelected>>', lambda _e, k=key: self.teleop_node.set_switch_state(k, self.switch_vars[k].get()))
            self.switch_vars[key] = var

        act_f = ttk.Frame(ctrl)
        act_f.grid(row=0, column=len(self.AXIS_WIDGETS) + 2, padx=10)
        ttk.Button(act_f, text='Center Axes', command=self._center_axes).pack(fill=tk.X, pady=2)
        ttk.Button(act_f, text='Release Btns', command=self._release_buttons).pack(fill=tk.X, pady=2)
        ttk.Button(act_f, text='Switches Mid', command=self._reset_switches).pack(fill=tk.X, pady=2)

        # ---- Elastic Band ----
        el = ttk.LabelFrame(main, text='Elastic Band')
        el.pack(fill=tk.X, pady=(6, 0))
        self._elastic_lbl = ttk.Label(el, text='Status: --   Length: --', font=('TkDefaultFont', 11, 'bold'))
        self._elastic_lbl.grid(row=0, column=0, columnspan=2, padx=8, pady=4, sticky='w')
        self._btn_up = ttk.Button(el, text='▲  UP', width=12,
                                  command=lambda: self.monitor_node.send_elastic_cmd('up'), state='disabled')
        self._btn_up.grid(row=1, column=0, padx=8, pady=3)
        self._btn_down = ttk.Button(el, text='▼  DOWN', width=12,
                                    command=lambda: self.monitor_node.send_elastic_cmd('down'), state='disabled')
        self._btn_down.grid(row=1, column=1, padx=8, pady=3)

        # ---- SBUS Status ----
        sb = ttk.LabelFrame(main, text='SBUS Status')
        sb.pack(fill=tk.X, pady=(6, 0))
        self.sbus_status = ttk.Label(sb, text='Waiting for /sbus_data/event...')
        self.sbus_status.pack(anchor='w', padx=6, pady=1)
        self.sbus_axes_label = ttk.Label(sb, text='x1: --  y1: --  x2: --  y2: --')
        self.sbus_axes_label.pack(anchor='w', padx=6)
        self.button_label = ttk.Label(sb, text='Buttons A-D: --')
        self.button_label.pack(anchor='w', padx=6)
        self.switch_label = ttk.Label(sb, text='Switches E-H: --')
        self.switch_label.pack(anchor='w', padx=6)
        self.key_event_label = ttk.Label(sb, text='Key Event: --')
        self.key_event_label.pack(anchor='w', padx=6, pady=(0, 3))

    def _center_axes(self) -> None:
        for axis, s in self.axis_scales.items():
            s.set(0.0); self.teleop_node.set_axis_override(axis, 0.0)

    def _release_buttons(self) -> None:
        for b, v in self.button_vars.items():
            v.set(False); self.teleop_node.set_button_state(b, False)

    def _reset_switches(self) -> None:
        for k, v in self.switch_vars.items():
            v.set('mid'); self.teleop_node.set_switch_state(k, 'mid')

    def _schedule_update(self) -> None:
        self._refresh()
        self.root.after(100, self._schedule_update)

    def _refresh(self) -> None:
        snap = self.monitor_node.snapshot()

        # elastic band
        state = snap.get('elastic_state', 'unknown')
        length = snap.get('elastic_length', 0.0)
        self._elastic_lbl.configure(text=f'Status: {state.capitalize()}   Length: {length:.2f} m')
        bs = 'normal' if state == 'suspended' else 'disabled'
        self._btn_up.configure(state=bs); self._btn_down.configure(state=bs)

        # sbus
        sbus_msg = snap['sbus']
        if not snap['sbus_enabled']:
            self.sbus_status.configure(text='SBUS monitor disabled')
            return
        if sbus_msg is None:
            self.sbus_status.configure(text='Waiting for /sbus_data/event...')
            return
        self.sbus_status.configure(text='SBUS OK')
        self.sbus_axes_label.configure(
            text=f'x1:{sbus_msg.x1:+.2f}  y1:{sbus_msg.y1:+.2f}  x2:{sbus_msg.x2:+.2f}  y2:{sbus_msg.y2:+.2f}')
        self.button_label.configure(
            text='Buttons: ' + '  '.join(f'{n[-1]}:{getattr(sbus_msg, n):+d}' for n in self.BUTTON_FIELDS))
        self.switch_label.configure(
            text='Switches: ' + '  '.join(f'{n[-1]}:{getattr(sbus_msg, n):+d}' for n in self.SWITCH_FIELDS))
        self.key_event_label.configure(
            text=f'Key Event new={sbus_msg.key_event_new}  old={sbus_msg.key_event_old}')

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.executor.shutdown(cancel_tasks=True)
        except Exception:
            pass
        self.teleop_node.destroy_node()
        self.monitor_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.quit()
        sys.exit(0)

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.close()


def main(args=None) -> None:
    TeleopGuiApp().run()


def main_joy(args=None) -> None:
    """Standalone teleop node — 物理手柄直驱，无 GUI。

    用法: ros2 run sim_joy teleop_joy
    支持 Xbox / PS / 云卓等标准游戏手柄（通过 inputs 库读取），
    发布 /sbus_data (sensor_msgs/Joy)。
    """
    rclpy.init(args=args)
    node = TeleopJoyNode(enable_gamepad=True, enable_keyboard=False, enable_console=False)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

