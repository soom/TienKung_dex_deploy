# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 仓库布局

`src/` 下三个 ament_python 包：

- `src/rl_control/` — xMIGCS 主控制节点，FSM + 多策略。可执行：`rl_control_node`（真机）、`rl_control_node_sim`（仿真）。`.git` 子目录是早期 vendored 进来的遗留，不影响构建。
- `src/mujoco_node/` — MuJoCo EVT2 仿真器，可执行：`mujoco_node`（即 `mujoco_node.simulator_view_asyn:main`）。同样有 vendored 的 `.git`。
- `src/sim_joy/` — Tiangong 遥操作（Tkinter GUI / 手柄 / 键盘），可执行：`teleop_joy`、`teleop_gui`。

整个 workspace 通过单次 `colcon build --symlink-install` 构建。根目录提供两个一键脚本：`run_real.sh`、`run_sim.sh`，自动按 Ubuntu 版本选择 humble/jazzy、构建并 launch。

## 环境与依赖

- **Python 解释器**：本机用 venv `.venv`（Python 3.10，与 ROS humble 同 minor）跑 ROS humble。venv 用 `--system-site-packages` 创建，能继承系统 apt 装的 `python3-tk`，并能 import 系统 `rclpy` / `bodyctrl_msgs`（venv 与 ROS 同为 Python 3.10，`PYTHONPATH` 能直接消费 `/opt/ros/humble/lib/python3.10/site-packages`）。PyPI 依赖清单按 ROS 版本切分：humble 用 `src/rl_control/requirements_22.txt`，jazzy 用 `src/rl_control/requirements_24.txt`。
- **首次使用必须执行 `./init_env.sh`**：脚本按 `ROS_DISTRO` 自动选 Python 3.10/3.12 与对应 requirements，创建 `.venv`、装齐 PyPI 依赖、装本地 sptlib_python wheel；缺 `bodyctrl_msgs` / `python3-tk` 时打印 `sudo apt install` 命令让用户手动执行（脚本本身不调 sudo）。
- **双 ROS 发行版支持**：22.04/humble 和 24.04/jazzy，由 `init_env.sh` 与 `run_*.sh` 自动检测；可用 `ROS_DISTRO` 覆盖，可用 `VENV_DIR` 覆盖默认 `.venv` 路径。
- **自定义二进制依赖**（必须从 `src/rl_control/lib/` 安装，不在 PyPI / apt 上）：
  - `bodyctrl_msgs`：自定义 ROS2 消息包，从 `lib/{22,24}/ros-{humble,jazzy}-bodyctrl-msgs_*.deb` 安装。
  - `sptlib_python`：串并联转换库（`from sptlib_python import funcSPTrans`），由 `init_env.sh` 从对应的 `.whl` 装到 venv。
- 用户名为 `ubuntu` 时，`sim` 配置必须为 `false`，否则启动即抛错（`rl_control_node.py` 中的有意真机保护）。

### 为什么必须先 activate venv 再 source ROS
ROS humble 把节点的 entry-point shebang 写成 `#!<configure 时的 python>`。先 `source .venv/bin/activate` 再 `source /opt/ros/humble/setup.bash` 然后 `colcon build`，生成的 entry-point 才会指向 venv python（如 `/home/path/ros_dex/.venv/bin/python`）；否则会落到 `/usr/bin/python3`，进而触发系统 numpy 与 scipy 的 ABI 冲突，且 `mujoco` 模块也找不到。`run_*.sh` 已自动判断，shebang 不对时清掉 `install/build/log` 重建。

## 运行命令

```bash
# 仿真（拉起 mujoco_node + sim_joy GUI + rl_control_node_sim）
./run_sim.sh
# 真机（仅 rl_control_node）
./run_real.sh

# 直接 launch（前提是已 colcon build + source install/setup.bash）
ros2 launch rl_control sim.launch.py
ros2 launch rl_control real.launch.py

# 切换配置文件（默认 dex_config_real.yaml / dex_config_sim.yaml）
ros2 launch rl_control sim.launch.py config_file:=my_custom.yaml

# 手动构建（先激活 venv 再 source ROS）
source .venv/bin/activate && source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## FSM 架构

控制逻辑以有限状态机为核心，状态类在 `src/rl_control/FSM/` 和 `src/rl_control/policy/` 中实现。

**状态枚举**（`FSM/fsm_base.py`）：`STOP`、`ZERO`、`WALKAMP`、`BEYONDZERO`、`BEYONDMIMIC`、`MIMIC`、`MIMICDEFAULT`

**状态-策略映射**：

| FSM 状态 | 策略类 | 文件 |
|---|---|---|
| STOP | FSMStateStop | `policy/stop/fsm_stop.py` |
| ZERO | FSMStateZero | `policy/zero/fsm_zero.py` |
| WALKAMP | FSMStateWALKAMP | `policy/walk_amp/fsm_walkamp.py` |
| BEYONDZERO | FSMStateBeyondZero | `policy/beyondzero/fsm_beyondzero.py` |
| BEYONDMIMIC | FSMStateBeyondMimic | `policy/niukua/fsm_beyond_mimic.py` |
| MIMIC | FSMStateMimic | `policy/mimic/out/sim2sim_dex.py` |

每个状态实现 `FSMState` 的四个方法：`on_enter()`、`run()`、`on_exit()`、`check_transition()`。状态切换由 `ControlFlag.fsm_state_command` 字符串触发（如 `"gotoZERO"`、`"gotoWALKAMP"`）。`RobotFSMImpl`（`FSM/robot_fsm.py`）管理所有状态对象并驱动主循环。

### 当前 Mimic 动作库

动作文件位于 `policy/mimic/out/npz/`，按文件名字母序自动加载为 playlist：

| 序号 | 文件名 | 描述 |
|---|---|---|
| 00 | `cigarette_pick_up_R_002__A458_with_transition` | 右手拾烟 |
| 01 | `crouch_idle_002__A244_with_transition` | 蹲姿待机 |
| 02 | `dance_freedom_wheels_001__A466_with_transition` | 自由轮舞 |
| 03 | `dance_hang_loose_celebration_003__A467_with_transition` | 庆祝舞蹈 |

**手柄触发**（`common/joystick.py`）：
- `E 上拨 + 长按 A`（≥30 帧，约 300 ms）→ `gotoMIMIC`，进入 playlist 第一个动作
- `B 键`（边沿检测，仅 MIMIC 激活时有效）→ `motion_cmd = "nextMotion"`，切到下一个动作
- `E 上拨 + D` → `gotoMIMICDEFAULT`（默认 mimic 姿态）

**sim2sim 独立运行**（`policy/mimic/out/sim2sim_dex.sh`）：
```bash
cd src/rl_control/policy/mimic/out
bash sim2sim_dex.sh               # 默认加载 npz/ 目录，MuJoCo 窗口按 B 切换动作
NPZ_DIR=npz/my_motion.npz bash sim2sim_dex.sh   # 单文件
```

### 添加新策略

1. 在 `policy/<name>/` 下新建目录，创建 `fsm_<name>.py`（继承 `FSMState`）和 `<name>.yaml`。
2. 在 `FSM/fsm_base.py` 的 `FSMStateName` 枚举中添加新状态。
3. 在 `FSM/robot_fsm.py` 的 `_init_states()` 中实例化并注册。
4. 在 `common/joystick.py` 或 `common/stdin_keyboard_control.py` 中添加触发逻辑。

## 控制数据流

```
Joy 消息 (/sbus_data)
  → JoystickHumanoid (common/joystick.py)  解析按键/摇杆
  → ControlFlag  (fsm_state_command + 速度/高度参数)
  → RobotFSMImpl.run()  执行当前状态
  → FSMState.run()  写入 RobotData.q_d_ / q_dot_d_ / tau_d_
  → RobotInterfaceImpl (common/robot_interface.py)  施加关节限位 + SPTrans 踝关节变换
  → CmdMotorCtrl 发布 (真机) / MuJoCo 步进 (仿真)
  → 传感器反馈 → RobotData.q_a_ / q_dot_a_ / tau_a_ 更新
```

**关键数据容器** `RobotData`（`common/robot_data.py`）：35 DOF（6 浮动基座 + 29 电机），存储实际值（`q_a_`/`q_dot_a_`/`tau_a_`）与期望值（`q_d_`/`q_dot_d_`/`tau_d_`）。

## 配置文件结构

主配置在 `src/rl_control/config/dex_config_{sim,real}.yaml`，关键字段：

- `sim: true/false` — 真机保护：用户名为 `ubuntu` 时必须为 `false`
- `control_tool: joystick|xbox|keyboard` — 输入设备
- `robot_interface.waist/legs/arms_control_status` — 各关节组在哪些 FSM 状态下受控（空列表 = 所有状态）
- `robot_interface.joint_limits` — 每个关节的 min/max（弧度）
- `robot_interface.zero_pos` — 29 轴零位（直接影响 ZERO 状态插值目标）

各策略目录下有独立 YAML（如 `policy/walk_amp/walk_amp.yaml`），存储观测维度、decimation、动作缩放等超参数。

## ROS 话题约定

| 话题 | 类型 | 方向 |
|---|---|---|
| `/sbus_data` | `sensor_msgs/Joy` | sim_joy → rl_control |
| 腿/臂/腰 motor cmd | `bodyctrl_msgs/CmdMotorCtrl` | rl_control → 硬件/仿真 |
| motor status | `bodyctrl_msgs/MotorStatusMsg` | 硬件/仿真 → rl_control |
| `/imu/data` | `sensor_msgs/Imu` | 仿真 → rl_control |

`ROS_DOMAIN_ID=42`（`run_*.sh` 设置，防止多机串话）。
