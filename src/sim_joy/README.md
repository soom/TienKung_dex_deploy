# sim_joy

Joystick teleoperation package for Tiangong robot simulation.

## Features

- **Teleop Node**: Publishes `sensor_msgs/Joy` and `bodyctrl_msgs/SbusData`.
- **Input Sources**: Gamepad (via `inputs`), Keyboard (via `pynput`), Console, and GUI.
- **GUI Monitor**: Tkinter-based GUI for visualization and control.

## Usage

### Run Teleop Node (Console/Gamepad/Keyboard)
```bash
ros2 run sim_joy teleop_joy
```

### Run GUI Control
```bash
ros2 run sim_joy teleop_gui
```
The GUI will automatically start the teleop logic internally.

## Dependencies
- `inputs`
- `pynput`
- `python3-tk`
