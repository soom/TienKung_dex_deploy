"""FSMStateMimic — playlist motion-imitation policy using policy_merged.onnx.

Observation layout (216 dims = 214 raw + 2 phase one-hot):
  [0:29]   q_ref          — reference joint pos (ONNX seq order)
  [29:58]  qd_ref         — reference joint vel
  [58:64]  anchor_rot6d   — ref-pelvis orientation relative to robot pelvis (rot6d)
  [64:67]  base_ang_vel   — robot angular velocity in body frame
  [67:96]  q_err          — q_cur − q_ref
  [96:125] qd_err         — qd_cur − qd_ref
  [125:154] last_action
  [154:156] motion_phase  — [sin, cos] of phase = step / total_steps
  [156:185] qd_lookahead  — reference vel at step+2
  [185:214] q_lookahead   — reference pos at step+2
  [214:216] phase_one_hot — [1,0] standing / [0,1] motion playback
"""
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import onnxruntime

from FSM.fsm_base import FSMState, FSMStateName
from common.joystick import ControlFlag
from common.robot_data import RobotData

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

# ── joint ordering ──────────────────────────────────────────────────────────

# XML / robot order (29 joints, matches robot_data q_a_ / q_d_ layout)
_JOINT_XML_29 = [
    "hip_pitch_l_joint", "hip_roll_l_joint", "hip_yaw_l_joint",
    "knee_pitch_l_joint", "ankle_pitch_l_joint", "ankle_roll_l_joint",
    "hip_pitch_r_joint", "hip_roll_r_joint", "hip_yaw_r_joint",
    "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
    "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint", "wrist_roll_r_joint",
]

LOOKAHEAD_STEPS = 2


def _build_mj2lab(seq_names: list[str]) -> np.ndarray:
    """seq → xml index mapping (same convention as FSMStateBeyondMimic)."""
    return np.array([_JOINT_XML_29.index(j) for j in seq_names], dtype=np.int32)


def _build_npz2seq(npz_names: list[str], seq_names: list[str]) -> np.ndarray:
    """npz joint order → seq order reindex array."""
    return np.array([npz_names.index(j) for j in seq_names], dtype=np.int32)


# ── quaternion helpers (no scipy dependency) ─────────────────────────────────

def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-8 else np.array([1.0, 0.0, 0.0, 0.0])


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _quat_normalize(q)
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def _rotmat_to_rot6d(R: np.ndarray) -> np.ndarray:
    return np.array([R[0,0], R[0,1], R[1,0], R[1,1], R[2,0], R[2,1]], dtype=np.float32)


def _yaw_from_quat(q: np.ndarray) -> float:
    w, x, y, z = _quat_normalize(q)
    return float(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def _quat_from_yaw(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw/2), 0.0, 0.0, np.sin(yaw/2)])


def _smoothstep(alpha: float) -> float:
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


# ── NpzMotionClip ─────────────────────────────────────────────────────────────

class NpzMotionClip:
    """Loads a single NPZ file and provides frame-indexed reference data in ONNX seq order."""

    def __init__(self, npz_path: str, seq_names: list[str]):
        data = np.load(npz_path, allow_pickle=False)
        npz_names = [str(n) for n in data["joint_names"].tolist()]
        npz2seq = _build_npz2seq(npz_names, seq_names)

        self.T: int = int(data["joint_pos"].shape[0])
        self.fps: float = float(np.asarray(data["fps"]).item())
        self.name: str = Path(npz_path).stem

        # joint data already in seq order
        self._joint_pos = data["joint_pos"][:, npz2seq].astype(np.float32)
        self._joint_vel = data["joint_vel"][:, npz2seq].astype(np.float32)

        # body data (all 30 bodies, world frame)
        npz_body_names = [str(n) for n in data["body_names"].tolist()]
        self._body_pos_w  = data["body_pos_w"].astype(np.float64)   # (T, 30, 3)
        self._body_quat_w = data["body_quat_w"].astype(np.float64)  # (T, 30, 4) wxyz

        print(f"[NpzMotionClip] {self.name}  T={self.T}  fps={self.fps}  dur={self.T/self.fps:.1f}s")

    def joint_pos_at(self, step: int) -> np.ndarray:
        return self._joint_pos[min(step, self.T - 1)]

    def joint_vel_at(self, step: int) -> np.ndarray:
        return self._joint_vel[min(step, self.T - 1)]

    def anchor_quat_at(self, step: int) -> np.ndarray:
        """Pelvis (body index 0) world-frame quaternion wxyz."""
        return self._body_quat_w[min(step, self.T - 1), 0]


# ── FSMStateMimic ─────────────────────────────────────────────────────────────

class FSMStateMimic(FSMState):
    """Playlist motion-imitation using policy_merged.onnx + NPZ reference clips."""

    def __init__(self, robot_data: RobotData, config_path: Optional[str] = None):
        super().__init__(robot_data)
        self._current_dir = os.path.dirname(os.path.abspath(__file__))

        if config_path is None:
            config_path = os.path.join(self._current_dir, "config", "mimic.yaml")
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        onnx_rel = cfg["onnx_path"]
        onnx_path = onnx_rel if os.path.isabs(onnx_rel) else os.path.join(self._current_dir, onnx_rel)
        npz_dir = cfg["npz_dir"]
        if not os.path.isabs(npz_dir):
            npz_dir = os.path.join(self._current_dir, npz_dir)

        self.physical_dt: float = float(cfg["physical_dt"])
        self.decimation: int = int(cfg["decimation"])
        self.num_actions: int = int(cfg["num_actions"])
        self.motor_nums: int = int(cfg["motor_nums"])
        self.warm_start_time: float = float(cfg.get("warm_start_time", 0.5))
        self.stand_duration_s: float = float(cfg.get("stand_duration_s", 1.0))
        self.hold_final_reference: bool = bool(cfg.get("hold_final_reference", False))
        _mts_s = cfg.get("max_target_step_standing", None)
        _mts_m = cfg.get("max_target_step_motion", None)
        self.max_target_step_standing: Optional[float] = float(_mts_s) if _mts_s is not None else None
        self.max_target_step_motion: Optional[float] = float(_mts_m) if _mts_m is not None else None

        # ── ONNX session ────────────────────────────────────────────────────
        self.ort_session = onnxruntime.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        inputs = self.ort_session.get_inputs()
        self.input_obs_name: str = inputs[0].name
        self.input_step_name: str = inputs[1].name
        self.output_names: list[str] = [o.name for o in self.ort_session.get_outputs()]

        meta = self.ort_session.get_modelmeta().custom_metadata_map
        self.joint_seq: list[str] = meta["joint_names"].split(",")
        self.default_joint_pos_seq = np.array([float(x) for x in meta["default_joint_pos"].split(",")], dtype=np.float32)
        self.action_scale_seq      = np.array([float(x) for x in meta["action_scale"].split(",")],      dtype=np.float32)
        # ONNX seq indices where action_scale == 0 — these joints get no policy
        # correction and rely purely on PD. During standing we soften their gains.
        self._action_zero_seq_idx = np.where(self.action_scale_seq == 0.0)[0]
        kps_seq = np.array([float(x) for x in meta["joint_stiffness"].split(",")], dtype=np.float32)
        kds_seq = np.array([float(x) for x in meta["joint_damping"].split(",")],   dtype=np.float32)

        self.obs_dim: int = int(self.ort_session.get_inputs()[0].shape[1])

        # ── joint order mappings ─────────────────────────────────────────────
        # mj2lab[i] = xml index for the i-th ONNX seq joint  (seq → xml)
        self.mj2lab = _build_mj2lab(self.joint_seq)
        # joints held at default during standing (arms + waist = no vibration, no rotation drift)
        self._stand_fixed_seq_idx = np.array([i for i, n in enumerate(self.joint_seq)
                                              if any(k in n for k in ('shoulder', 'elbow', 'wrist', 'waist'))],
                                             dtype=np.int32)
        # kp/kd in xml order for robot_data (motion tracking — from ONNX)
        self.kps_xml = np.zeros(self.motor_nums, dtype=np.float32)
        self.kds_xml = np.zeros(self.motor_nums, dtype=np.float32)
        self.kps_xml[self.mj2lab] = kps_seq
        self.kds_xml[self.mj2lab] = kds_seq

        # Conservative kp/kd for standing / return-stand phases (XML order).
        # Read from mimic.yaml; falls back to validated BEYONDZERO-matching values.
        # The ONNX model's gains are tuned for motion tracking with policy
        # inference — several arm joints (elbow_yaw, wrist_*) have action_scale=0
        # and rely purely on high-gain PD. During static standing those high gains
        # cause arm oscillation.
        self.kps_stand_xml = np.array(cfg.get("kps_stand"), dtype=np.float32)
        self.kds_stand_xml = np.array(cfg.get("kds_stand"), dtype=np.float32)

        # default joint pos in xml order (for FSMStateMimicDefault & warm-start)
        self.default_joint_pos_xml = np.zeros(self.motor_nums, dtype=np.float32)
        self.default_joint_pos_xml[self.mj2lab] = self.default_joint_pos_seq

        # ── NPZ playlist ─────────────────────────────────────────────────────
        npz_paths = sorted(Path(npz_dir).glob("*.npz"))
        if not npz_paths:
            raise RuntimeError(f"[FSMStateMimic] No NPZ files found in {npz_dir}")
        self.clips: list[NpzMotionClip] = [NpzMotionClip(str(p), self.joint_seq) for p in npz_paths]
        print(f"[FSMStateMimic] Loaded {len(self.clips)} motion clips:")
        for i, c in enumerate(self.clips):
            print(f"  [{i:02d}] {c.name}")

        # ── runtime state ─────────────────────────────────────────────────────
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)
        self._stand_total_steps: int = max(1, int(round(self.stand_duration_s / self.physical_dt)))
        self._warm_start_steps: int = 0

        # playlist phase: "default_pose" | "pre_stand" | "play_motion" | "return_stand"
        self._playback_phase: str = "default_pose"
        self._active_clip_idx: int = -1   # -1 = never played; set on first play
        self._next_clip_idx: int = 0
        self._stand_step_count: int = 0
        self._motion_step: int = 0
        self._pending_next: bool = False

        # warm-start state
        self._ws_from_seq = np.zeros(self.num_actions, dtype=np.float32)
        self._ws_to_seq   = np.zeros(self.num_actions, dtype=np.float32)
        self._ws_prev_seq = np.zeros(self.num_actions, dtype=np.float32)
        self._ws_counter: int = 0

        # yaw alignment: set at play_motion entry
        self._ref_yaw_inv = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        self._last_run_time = time.perf_counter()

    # ── FSMState interface ────────────────────────────────────────────────────

    def on_enter(self):
        print("[FSMStateMimic] enter — starting at default_pose")
        print(f"  obs_dim={self.obs_dim}  num_actions={self.num_actions}  motor_nums={self.motor_nums}")
        print(f"  physical_dt={self.physical_dt}  decimation={self.decimation}  warm_start_steps={self._warm_start_steps}")
        print(f"  stand_total_steps={self._stand_total_steps}")
        print(f"  default_joint_pos_xml={np.round(self.default_joint_pos_xml, 3).tolist()}")
        print(f"  kps_xml={self.kps_xml[:15].tolist()} ...")
        print(f"  kds_xml={self.kds_xml[:15].tolist()} ...")
        self._playback_phase = "default_pose"
        self._active_clip_idx = -1   # reset: next B will start from clip 0
        self._pending_next = False
        self._stand_step_count = 0
        self._motion_step = 0
        self.last_action[:] = 0.0
        self._ws_counter = 0
        if self.warm_start_time > 0:
            self._warm_start_steps = max(1, int(self.warm_start_time / (self.decimation * self.physical_dt)))
        else:
            self._warm_start_steps = 0
        self._ref_yaw_inv[:] = [1.0, 0.0, 0.0, 0.0]
        self._log_counter: int = 0
        self._call_counter: int = 0

    def on_exit(self):
        self.last_action[:] = 0.0
        self._playback_phase = "default_pose"
        self._pending_next = False
        self._stand_step_count = 0
        self._motion_step = 0
        self._ws_counter = 0
        self._call_counter: int = 0
        print("[FSMStateMimic] exit")

    def run(self, flag: ControlFlag):
        self._call_counter += 1

        # Edge-detect nextMotion at full rate (100Hz) so quick B presses aren't missed
        _prev_motion_cmd = getattr(self, "_prev_motion_cmd", "")
        if flag.motion_cmd == "nextMotion" and _prev_motion_cmd != "nextMotion":
            self._pending_next = True
            print(f"\n[Mimic] received nextMotion → _pending_next=True  (phase={self._playback_phase})")
        self._prev_motion_cmd = flag.motion_cmd

        if self._call_counter % self.decimation != 0:
            self._set_kp_kd()
            return

        now = time.perf_counter()
        print(f"\r[Mimic] hz={1/(now-self._last_run_time):.1f}  phase={self._playback_phase}  step={self._motion_step}", end=" ", flush=True)
        self._last_run_time = now

        self._step_playlist()
        self._set_kp_kd()

    def check_transition(self, flag: ControlFlag) -> Optional[FSMStateName]:
        cmd = flag.fsm_state_command
        mapping = {
            "gotoSTOP":        FSMStateName.STOP,
            "gotoZERO":        FSMStateName.ZERO,
            "gotoWALKAMP":     FSMStateName.WALKAMP,
            "gotoBEYONDZERO":  FSMStateName.BEYONDZERO,
            "gotoBEYONDMIMIC": FSMStateName.BEYONDMIMIC,
            "gotoMIMIC":       FSMStateName.MIMIC,
            "gotoMIMICDEFAULT": FSMStateName.MIMICDEFAULT,
        }
        return mapping.get(cmd)

    # ── playlist logic ────────────────────────────────────────────────────────

    def _step_playlist(self):
        if self._pending_next:
            if self._playback_phase in ("default_pose", "play_motion", "return_stand"):
                self._pending_next = False
                print(f"[Mimic] _pending_next consumed → starting pre_stand (was phase={self._playback_phase})")
                self._start_pre_stand()
            else:
                print(f"[Mimic] _pending_next=True but phase={self._playback_phase} — not consumed yet")

        if self._playback_phase == "default_pose":
            self._run_standing_step()
        elif self._playback_phase == "pre_stand":
            self._run_standing_step()
            self._stand_step_count += 1
            if self._stand_step_count >= self._stand_total_steps:
                self._start_play_motion()

        elif self._playback_phase == "play_motion":
            self._run_motion_step()

        elif self._playback_phase == "return_stand":
            self._run_return_stand_step()
            self._stand_step_count += 1
            if self._stand_step_count >= self._stand_total_steps:
                self._playback_phase = "default_pose"
                print("\n[Mimic] return stand complete → default_pose")

    def _start_pre_stand(self):
        if self._active_clip_idx < 0:
            self._next_clip_idx = 0
        else:
            self._next_clip_idx = (self._active_clip_idx + 1) % len(self.clips)
        self._stand_step_count = 0
        self._playback_phase = "pre_stand"
        clip_name = self.clips[self._next_clip_idx].name
        print(f"\n[Mimic] pre-stand → [{self._next_clip_idx}] {clip_name}")

    def _start_play_motion(self):
        self._active_clip_idx = self._next_clip_idx
        clip = self.clips[self._active_clip_idx]
        self._motion_step = 0
        self._motion_substep = 0
        self._playback_phase = "play_motion"
        self.last_action[:] = 0.0
        self._ws_counter = 0

        # Align reference yaw to robot's current heading
        robot_quat = np.array(self.robot_data_.get_robot_quat(), dtype=np.float64)
        ref_quat0  = clip.anchor_quat_at(0)
        robot_yaw  = _yaw_from_quat(robot_quat)
        ref_yaw0   = _yaw_from_quat(ref_quat0)
        delta_yaw  = robot_yaw - ref_yaw0
        self._ref_yaw_inv = _quat_from_yaw(delta_yaw)

        # warm-start: from current pose to first frame (both in seq order)
        q_cur_xml = self.robot_data_.get_joint_pos().copy()
        self._ws_from_seq = q_cur_xml[self.mj2lab].astype(np.float32)
        self._ws_to_seq   = clip.joint_pos_at(0).copy()
        self._ws_prev_seq = self._ws_from_seq.copy()

        print(f"\n[Mimic] playing [{self._active_clip_idx}] {clip.name}")

    # ── per-step runners ─────────────────────────────────────────────────────

    def _run_standing_step(self):
        """Run policy with yaw-aligned upright reference (same as sim2sim _yaw_aligned_default_ref)."""
        ref_pos_seq = self.default_joint_pos_seq
        ref_vel_seq = np.zeros(self.num_actions, dtype=np.float32)
        robot_quat = np.array(self.robot_data_.get_robot_quat(), dtype=np.float64)
        ref_anchor_quat = _quat_from_yaw(_yaw_from_quat(robot_quat))
        self._run_policy(ref_pos_seq, ref_vel_seq, ref_pos_seq, ref_vel_seq,
                         motion_step=0, total_steps=1, phase_mode_idx=0,
                         ref_anchor_quat=ref_anchor_quat)

    def _run_motion_step(self):
        clip = self.clips[self._active_clip_idx]
        step = self._motion_step  # current NPZ frame index (0, 0, 1, 1, 2, 2, ...)
        total = clip.T

        ref_pos = clip.joint_pos_at(step)
        ref_vel = clip.joint_vel_at(step)
        la_step = min(step + LOOKAHEAD_STEPS, total - 1)
        la_pos  = clip.joint_pos_at(la_step)
        la_vel  = clip.joint_vel_at(la_step)

        use_ws = self._ws_counter < self._warm_start_steps
        if use_ws:
            blend = (self._ws_counter + 1) / self._warm_start_steps
            ref_pos = (1.0 - blend) * self._ws_from_seq + blend * self._ws_to_seq
            ref_vel = (ref_pos - self._ws_prev_seq) / self.physical_dt
            self._ws_prev_seq = ref_pos.copy()
            self._ws_counter += 1

        self._run_policy(ref_pos, ref_vel, la_pos, la_vel,
                         motion_step=step, total_steps=total, phase_mode_idx=1,
                         ref_anchor_quat=clip.anchor_quat_at(step))

        # Check end: last NPZ frame and its second policy call done
        if step >= total - 1:
            print(f"\n[Mimic] finished [{self._active_clip_idx}] {clip.name}")
            self._start_return_stand()
            return

        # NPZ at 50fps, policy at 100Hz: advance frame every 2 policy calls
        self._motion_substep += 1
        if self._motion_substep % 2 == 0:
            self._motion_step = min(self._motion_step + 1, total - 1)

    def _start_return_stand(self):
        self._stand_step_count = 0
        q_from_xml = self.robot_data_.get_joint_pos().copy()
        self._return_stand_q_from_seq = q_from_xml[self.mj2lab].astype(np.float32)
        self._playback_phase = "return_stand"
        self.last_action[:] = 0.0

    def _run_return_stand_step(self):
        alpha = _smoothstep((self._stand_step_count + 1) / max(self._stand_total_steps, 1))
        ref_pos = (1.0 - alpha) * self._return_stand_q_from_seq + alpha * self.default_joint_pos_seq
        ref_vel = np.zeros(self.num_actions, dtype=np.float32)
        robot_quat = np.array(self.robot_data_.get_robot_quat(), dtype=np.float64)
        ref_anchor_quat = _quat_from_yaw(_yaw_from_quat(robot_quat))
        self._run_policy(ref_pos, ref_vel, ref_pos, ref_vel,
                         motion_step=0, total_steps=1, phase_mode_idx=0,
                         ref_anchor_quat=ref_anchor_quat)

    def _run_policy(
        self,
        ref_pos_seq: np.ndarray,
        ref_vel_seq: np.ndarray,
        la_pos_seq:  np.ndarray,
        la_vel_seq:  np.ndarray,
        motion_step: int,
        total_steps: int,
        phase_mode_idx: int,
        ref_anchor_quat: Optional[np.ndarray] = None,
    ):
        robot_quat = np.array(self.robot_data_.get_robot_quat(), dtype=np.float64)
        ang_vel    = self.robot_data_.get_angular_velocity().astype(np.float32)

        # current joint state in seq order: mj2lab[i] = xml index of seq joint i
        q_cur_xml  = self.robot_data_.get_joint_pos()
        qd_cur_xml = self.robot_data_.get_joint_vel()
        q_cur_seq  = q_cur_xml[self.mj2lab].astype(np.float32)
        qd_cur_seq = qd_cur_xml[self.mj2lab].astype(np.float32)

        # anchor rot6d: relative orientation of ref pelvis to robot pelvis
        if ref_anchor_quat is not None:
            aligned_ref_quat = _quat_mul(self._ref_yaw_inv, ref_anchor_quat)
        else:
            aligned_ref_quat = robot_quat
        anchor_quat_rel = _quat_mul(_quat_conjugate(robot_quat), aligned_ref_quat)
        anchor_rot6d = _rotmat_to_rot6d(_quat_to_rotmat(anchor_quat_rel))

        # phase
        total = max(total_steps - 1, 1)
        phase = float(np.clip(motion_step, 0, total)) / float(total)
        phase_angle = 2.0 * np.pi * phase
        phase_sincos = np.array([np.sin(phase_angle), np.cos(phase_angle)], dtype=np.float32)

        # phase one-hot
        one_hot = np.zeros(2, dtype=np.float32)
        one_hot[int(np.clip(phase_mode_idx, 0, 1))] = 1.0

        obs = np.concatenate([
            ref_pos_seq,
            ref_vel_seq,
            anchor_rot6d,
            ang_vel,
            (q_cur_seq  - ref_pos_seq).astype(np.float32),
            (qd_cur_seq - ref_vel_seq).astype(np.float32),
            self.last_action,
            phase_sincos,
            la_vel_seq,
            la_pos_seq,
            one_hot,
        ]).astype(np.float32)

        if obs.shape[0] != self.obs_dim:
            raise RuntimeError(f"[FSMStateMimic] obs dim mismatch: expected {self.obs_dim}, got {obs.shape[0]}")

        out = self.ort_session.run(
            self.output_names,
            {
                self.input_obs_name:  obs.reshape(1, -1),
                self.input_step_name: np.array([[motion_step]], dtype=np.float32),
            }
        )
        action_seq = np.asarray(out[0][0], dtype=np.float32)  # (29,) in seq order

        # policy action provides balance corrections in both standing and motion modes
        target_seq = ref_pos_seq + action_seq * self.action_scale_seq

        self.last_action = action_seq.copy()

        target_xml = np.zeros(self.motor_nums, dtype=np.float32)
        target_xml[self.mj2lab] = target_seq

        # Per-step joint target rate limit (sim2sim --max-target-step)
        # Skip clipping during return_stand — it needs large joint changes to return to standing
        _skip_clip = (self._playback_phase == "return_stand")
        _mts = None if _skip_clip else (self.max_target_step_motion if phase_mode_idx == 1 else self.max_target_step_standing)
        if _mts is not None:
            prev = getattr(self, "_prev_target_xml", None)
            if prev is not None:
                target_xml = np.clip(target_xml, prev - _mts, prev + _mts)
            self._prev_target_xml = target_xml.copy()

        base = 35 - self.motor_nums
        self.robot_data_.q_d_[base:]    = target_xml
        self.robot_data_.q_dot_d_[base:] = 0.0
        self.robot_data_.tau_d_[base:]   = 0.0

        # self._log_counter = getattr(self, "_log_counter", 0) + 1
        # if self._log_counter % 50 == 1:
        #     q_err_max = float(np.max(np.abs(q_cur_seq - ref_pos_seq)))
        #     action_max = float(np.max(np.abs(action_seq * self.action_scale_seq)))
        #     clip_info = ""
        #     if _mts is not None:
        #         raw = np.zeros(self.motor_nums, dtype=np.float32)
        #         raw[self.mj2lab] = ref_pos_seq + action_seq * self.action_scale_seq
        #         clip_info = f"  clipped={float(np.max(np.abs(target_xml - raw))):.4f}"
        #     print(f"\n[Mimic][{self._log_counter:05d}] phase={self._playback_phase}"
        #           f"  q_err_max={q_err_max:.4f}  action_max={action_max:.4f}"
        #           f"{clip_info}"
        #           f"  target_xml[0:6]={np.round(target_xml[:6], 3).tolist()}"
        #           f"  q_cur_xml[0:6]={np.round(q_cur_xml[:6], 3).tolist()}")

    def _set_kp_kd(self):
        # Start from ONNX gains, then soften only the joints that have
        # action_scale=0 — they get no policy correction and rely purely
        # on PD, so the ONNX high gains cause arm oscillation in all phases.
        kp = self.kps_xml.copy()
        kd = self.kds_xml.copy()
        for si in self._action_zero_seq_idx:
            xi = self.mj2lab[si]
            kp[xi] = self.kps_stand_xml[xi]
            kd[xi] = self.kds_stand_xml[xi]
        self.robot_data_.joint_kp_p_[:self.motor_nums] = kp
        self.robot_data_.joint_kd_p_[:self.motor_nums] = kd


# ── FSMStateMimicDefault ──────────────────────────────────────────────────────

class FSMStateMimicDefault(FSMState):
    """Hold the mimic policy's default standing pose with smooth interpolation."""

    def __init__(self, robot_data: RobotData, config_path: Optional[str] = None):
        super().__init__(robot_data)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        if config_path is None:
            config_path = os.path.join(current_dir, "config", "mimic.yaml")
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        onnx_rel  = cfg["onnx_path"]
        onnx_path = onnx_rel if os.path.isabs(onnx_rel) else os.path.join(current_dir, onnx_rel)

        self.motor_nums: int = int(cfg["motor_nums"])
        self.interp_step: float = float(cfg.get("interp_step", 0.01))

        session = onnxruntime.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        meta = session.get_modelmeta().custom_metadata_map
        joint_seq = meta["joint_names"].split(",")
        mj2lab = _build_mj2lab(joint_seq)

        default_seq = np.array([float(x) for x in meta["default_joint_pos"].split(",")], dtype=np.float32)

        self.default_xml = np.zeros(self.motor_nums, dtype=np.float32)
        self.default_xml[mj2lab] = default_seq

        # Conservative kp/kd for static pose holding (XML order, 29 joints).
        # Read from mimic.yaml; falls back to validated BEYONDZERO-matching values.
        self.kps_xml = np.array(cfg.get("kps_stand"), dtype=np.float32)
        self.kds_xml = np.array(cfg.get("kds_stand"), dtype=np.float32)

        self._q_factor: float = 0.0
        self._start_pose = np.zeros(self.motor_nums, dtype=np.float32)

    def on_enter(self):
        print("[FSMStateMimicDefault] enter")
        self._q_factor = 0.0
        self._start_pose = self.robot_data_.get_joint_pos().copy()

    def on_exit(self):
        print("[FSMStateMimicDefault] exit")

    def run(self, flag: ControlFlag):
        if self._q_factor < 1.0:
            pos = (1.0 - self._q_factor) * self._start_pose + self._q_factor * self.default_xml
            self._q_factor = min(self._q_factor + self.interp_step, 1.0)
        else:
            pos = self.default_xml
        base = 35 - self.motor_nums
        self.robot_data_.q_d_[base:]     = pos
        self.robot_data_.q_dot_d_[base:] = 0.0
        self.robot_data_.tau_d_[base:]   = 0.0
        self.robot_data_.joint_kp_p_[:self.motor_nums] = self.kps_xml
        self.robot_data_.joint_kd_p_[:self.motor_nums] = self.kds_xml

    def check_transition(self, flag: ControlFlag) -> Optional[FSMStateName]:
        cmd = flag.fsm_state_command
        mapping = {
            "gotoSTOP":         FSMStateName.STOP,
            "gotoZERO":         FSMStateName.ZERO,
            "gotoWALKAMP":      FSMStateName.WALKAMP,
            "gotoBEYONDZERO":   FSMStateName.BEYONDZERO,
            "gotoBEYONDMIMIC":  FSMStateName.BEYONDMIMIC,
            "gotoMIMIC":        FSMStateName.MIMIC,
            "gotoMIMICDEFAULT": FSMStateName.MIMICDEFAULT,
        }
        return mapping.get(cmd)
