# ros_dex_deploy — DEX 机器人策略部署框架

将强化学习策略从仿真训练到真机部署的完整工具链，面向**天工（Tiangong dex）全尺寸人形机器人**。

- **MuJoCo 物理仿真**：高保真 EVT2 模型，支持 sim2sim 策略验证
- **FSM 控制架构**：有限状态机驱动的多策略切换（行走、动作模仿）
- **真机部署**：400Hz 实时状态反馈 + 100Hz 策略闭环 + 并联脚踝 SPTrans 转换
- **一键脚本**：`init_env.sh` 初始化环境、`run_sim.sh` / `run_real.sh` 一键启动

支持 Ubuntu 22.04（ROS humble）和 Ubuntu 24.04（ROS jazzy）。

https://github.com/user-attachments/assets/db7537c0-8645-4799-b08a-abf08c58df76

https://github.com/user-attachments/assets/6557ac95-e8d5-4511-88d3-0c14027218e1

https://github.com/user-attachments/assets/6ed96ef1-5e8b-4313-9dd4-2f585072df10

## 仓库结构

```
ros_dex_deploy/
├── init_env.sh              # 一键初始化 venv 环境
├── run_sim.sh               # 一键启动仿真栈
├── run_real.sh              # 一键启动真机栈
└── src/
    ├── rl_control/          # xMIGCS 主控制节点（FSM + 多策略）
    │   ├── FSM/             # 有限状态机框架
    │   ├── common/          # 通用模块（joystick、robot_data 等）
    │   ├── config/          # YAML 配置文件
    │   ├── lib/             # 离线依赖（bodyctrl_msgs deb、sptlib wheel）
    │   ├── policy/          # 控制策略（walk_amp、beyondzero、niukua…）
    │   ├── rl_control_node.py      # 真机入口
    │   └── rl_control_node_sim.py  # 仿真入口
    ├── mujoco_node/         # MuJoCo EVT2 物理仿真节点
    │   ├── mujoco_node/     # 仿真器脚本（simulator_view_asyn.py）
    │   └── resources/evt2/  # 天工3.0 机器人模型文件
    └── sim_joy/             # 遥操作包（Tkinter GUI / 手柄 / 键盘）
```

## 快速开始

### 1. 初始化环境（首次必须执行）

```bash
./init_env.sh
```

脚本会自动：
- 检测 Ubuntu 版本并选择对应 ROS 发行版（22.04→humble，24.04→jazzy）
- 创建 `.venv`（`--system-site-packages`，继承系统 `python3-tk`）
- 安装 PyPI 依赖（`requirements_22.txt` 或 `requirements_24.txt`）
- 安装 `sptlib_python` 本地 wheel
- 检查 `bodyctrl_msgs` 是否已安装，缺失时打印 `sudo apt install` 命令

若提示缺少 apt 包，按提示手动执行后重跑 `./init_env.sh`。

### 2. 启动仿真

```bash
./run_sim.sh
```

启动三个节点：`mujoco_node`（物理仿真）+ `teleop_gui`（Tkinter GUI）+ `rl_control_node_sim`（控制）。

### 3. 启动真机

```bash
./run_real.sh
```

仅启动 `rl_control_node`（真机控制节点）。

## 依赖说明

| 依赖 | 来源 | 说明 |
|------|------|------|
| `rclpy`、ROS2 基础包 | `apt`（ros-humble/jazzy-desktop） | 系统级安装 |
| `bodyctrl_msgs` | `src/rl_control/lib/{22,24}/*.deb` | 自定义 ROS2 消息包，必须 `sudo apt install` |
| `sptlib_python` | `src/rl_control/lib/{22,24}/*.whl` | 串并联转换库，由 `init_env.sh` 装入 venv |
| `mujoco`、`onnxruntime`、`numpy` 等 | PyPI（清华镜像） | 由 `init_env.sh` 装入 venv |
| `python3-tk` | `apt` | GUI 依赖，通过 `--system-site-packages` 继承 |

### 手动安装 bodyctrl_msgs

```bash
# Ubuntu 22.04 / ROS humble
sudo apt install -y src/rl_control/lib/22/ros-humble-bodyctrl-msgs_*.deb

# Ubuntu 24.04 / ROS jazzy
sudo apt install -y src/rl_control/lib/24/ros-jazzy-bodyctrl-msgs_*.deb
```

## 配置

控制行为通过 YAML 文件配置，默认路径：

| 场景 | 配置文件 |
|------|---------|
| 真机 | `src/rl_control/config/dex_config_real.yaml` |
| 仿真 | `src/rl_control/config/dex_config_sim.yaml` |

可在启动时指定自定义配置：

```bash
ros2 launch rl_control sim.launch.py config_file:=my_custom.yaml
```

### 仿真控制方式

仿真支持两种遥控方式，均发布 `/sbus_data` 话题供控制节点消费：

| 方式 | 启动命令 | 说明 |
|------|---------|------|
| **GUI 遥控**（默认） | `ros2 launch sim_joy teleop_gui.launch.py` | Tkinter 窗口，鼠标操作摇杆/按钮/开关 |
| **手柄遥控** | `ros2 launch sim_joy teleop_joy.launch.py` | 物理游戏手柄（Xbox/PS/云卓），通过 `inputs` 库读取 |

`control_tool: joystick` 时两种方式通用，节点订阅 `/sbus_data`，无需改配置。

> **注意**：用户名为 `ubuntu` 时，配置中 `sim` 必须为 `false`，否则节点会抛出保护性错误（真机保护逻辑）。

## 控制器说明

### 云卓12手柄（默认）

| 操作 | 功能 |
|------|------|
| C | 切换到 STOP 状态 |
| 所有键回中 → D | 切换到 ZERO（回零位） |
| 所有键回中 → A | 切换到 WALKAMP（行走策略） |
| F 下拨 + 长按 A | 切换到 MIMIC（动作回放） |
| F 下拨 + D | 切换到 MIMICDEFAULT（默认 mimic 姿态） |
| B（MIMIC 状态下） | 切换到下一个动作 |
| E 下拨 + B | 切换到 BEYONDMIMIC |
| E 下拨 + D | 切换到 BEYONDZERO |
| 左摇杆 Y1 | 前后移动 |
| 左摇杆 X1 | 左右移动 |
| 右摇杆 X2 | 机身旋转 |


## 如何添加新的控制策略

1. 在 `src/rl_control/policy/` 下创建策略目录，添加 `fsm_mypolicy.py`
2. 继承 `FSMState` 并实现 `on_enter`、`run`、`on_exit`、`check_transition`
3. 在 `FSMStateName` 枚举中注册新状态，并在 `FSM/robot_fsm.py` 的 `_init_states()` 中初始化
4. 在 `common/robot_interface.py` 的 `_load_control_status()` 中添加字符串→枚举映射
5. 更新配置文件中的 `waist_control_status` / `arms_control_status` 列表

详细示例参见 `src/rl_control/README.md`。

## 高级用法

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROS_DISTRO` | 自动检测 | 覆盖 ROS 发行版（humble/jazzy） |
| `VENV_DIR` | `.venv` | 覆盖 venv 路径 |
| `ROS_DOMAIN_ID` | `42`（仿真） | ROS2 域 ID，防止局域网冲突 |

### 手动构建与启动

```bash
# 激活 venv 必须在 source ROS 之前（保证 entry-point shebang 正确）
source .venv/bin/activate
source /opt/ros/humble/setup.bash   # 或 jazzy

colcon build --symlink-install
source install/setup.bash

# 仿真
ros2 launch rl_control sim.launch.py

# 真机
ros2 launch rl_control real.launch.py
```

## 项目参考 
* https://github.com/Open-X-Humanoid/Deploy_Tienkung
