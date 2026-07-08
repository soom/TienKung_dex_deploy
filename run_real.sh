#!/usr/bin/env bash
# 启动 rl_control 真机栈：activate venv → source ROS → colcon build → ros2 launch real.launch.py
# 不开 set -u —— ROS 的 setup.bash 引用未定义变量。
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

VENV_DIR="${VENV_DIR:-$WS_DIR/.venv}"
if [[ ! -r "$VENV_DIR/bin/activate" ]]; then
  echo "[run_real] 找不到 $VENV_DIR/bin/activate；请先在仓库根目录运行 ./init_env.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
echo "[run_real] venv: $VIRTUAL_ENV ($(python -V))"

if ! python -c "import colcon_core" >/dev/null 2>&1; then
  echo "[run_real] venv 缺 colcon；请重跑 ./init_env.sh" >&2
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -r /etc/os-release ]]; then
    . /etc/os-release
    case "${VERSION_ID:-}" in
      22.04) ROS_DISTRO=humble ;;
      24.04) ROS_DISTRO=jazzy ;;
      *) echo "[run_real] 未识别的 Ubuntu 版本 $VERSION_ID，请显式 export ROS_DISTRO=" >&2; exit 1 ;;
    esac
  fi
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -r "$ROS_SETUP" ]]; then
  echo "[run_real] 找不到 $ROS_SETUP，请先安装 ros-${ROS_DISTRO}-desktop" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [[ -f "$WS_DIR/install/rl_control/lib/rl_control/rl_control_node" ]]; then
  shebang=$(head -n1 "$WS_DIR/install/rl_control/lib/rl_control/rl_control_node")
  venv_py="$VIRTUAL_ENV/bin/python"
  if [[ "$shebang" != "#!$venv_py" ]]; then
    echo "[run_real] 检测到旧 install/ 的 shebang 指向 $shebang ；清理后重建以使用 $venv_py"
    rm -rf "$WS_DIR/install" "$WS_DIR/build" "$WS_DIR/log"
  fi
fi

echo "[run_real] colcon build (--symlink-install)"
colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE="$(which python)"


source "/home/ubuntu/xos/setup.bash"

# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

export PYTHONUNBUFFERED=1

echo "[run_real] ros2 launch rl_control real.launch.py $*"
MIMIC_DIAG=1 exec ros2 launch rl_control real.launch.py "$@"
