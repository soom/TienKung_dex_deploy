#!/usr/bin/env bash
# 一键初始化 venv 环境：创建 .venv → pip 装 requirements_<22|24>.txt → 装 sptlib_python wheel
# 设计为 idempotent：可重复执行；已存在的 venv 与已装包会被尊重。
# 不在脚本里跑 sudo —— bodyctrl_msgs / python3-tk 缺失时仅打印命令让用户手动执行。
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# ---- 检测 ROS 发行版 / Python 版本 / lib 路径 ----
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -r /etc/os-release ]]; then
    . /etc/os-release
    case "${VERSION_ID:-}" in
      22.04) ROS_DISTRO=humble ;;
      24.04) ROS_DISTRO=jazzy ;;
      *) echo "[init_env] 未识别的 Ubuntu 版本 ${VERSION_ID:-?}，请显式 export ROS_DISTRO=humble|jazzy" >&2; exit 1 ;;
    esac
  else
    echo "[init_env] 缺 /etc/os-release，无法识别系统版本；请显式 export ROS_DISTRO=" >&2
    exit 1
  fi
fi

case "$ROS_DISTRO" in
  humble) PY_CMD=python3.10; LIB_DIR="$WS_DIR/src/rl_control/lib/22"; REQ_FILE="$WS_DIR/src/rl_control/requirements_22.txt" ;;
  jazzy)  PY_CMD=python3.12; LIB_DIR="$WS_DIR/src/rl_control/lib/24"; REQ_FILE="$WS_DIR/src/rl_control/requirements_24.txt" ;;
  *) echo "[init_env] 不支持的 ROS_DISTRO=$ROS_DISTRO（仅 humble / jazzy）" >&2; exit 1 ;;
esac

if ! command -v "$PY_CMD" >/dev/null 2>&1; then
  echo "[init_env] PATH 里找不到 $PY_CMD；请先 sudo apt install $PY_CMD $PY_CMD-venv" >&2
  exit 1
fi

# 仅有 $PY_CMD 还不够：apt 上 venv 模块单独打在 $PY_CMD-venv 包里
if ! "$PY_CMD" -c "import ensurepip, venv" >/dev/null 2>&1; then
  echo "[init_env] $PY_CMD 缺 venv / ensurepip 模块；请先执行：" >&2
  echo "         sudo apt install -y $PY_CMD-venv" >&2
  exit 1
fi

echo "[init_env] ROS_DISTRO=$ROS_DISTRO  python=$($PY_CMD -V)  lib=$LIB_DIR"

# ---- 创建 venv（继承 apt 装的 python3-tk 等系统包） ----
VENV_DIR="${VENV_DIR:-$WS_DIR/.venv}"
need_create=0
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  need_create=1
else
  cur_ver="$("$VENV_DIR/bin/python" -c 'import sys;print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"
  if [[ "$cur_ver" != "$PY_CMD" ]]; then
    echo "[init_env] 现存 venv 是 $cur_ver，与目标 $PY_CMD 不符；删除重建：$VENV_DIR"
    rm -rf "$VENV_DIR"
    need_create=1
  fi
fi

if (( need_create )); then
  echo "[init_env] 创建 venv：$VENV_DIR"
  "$PY_CMD" -m venv "$VENV_DIR" 
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
echo "[init_env] venv: $VIRTUAL_ENV ($(python -V))"

python -m pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple

# ---- 装 PyPI 依赖 ----
if [[ ! -r "$REQ_FILE" ]]; then
  echo "[init_env] 找不到 $REQ_FILE" >&2
  exit 1
fi
echo "[init_env] pip install -r $REQ_FILE"
pip install -r "$REQ_FILE" -i https://pypi.tuna.tsinghua.edu.cn/simple

# ---- 装本地 sptlib_python wheel ----
shopt -s nullglob
sptlib_wheels=("$LIB_DIR"/sptlib_python-*.whl)
shopt -u nullglob
if (( ${#sptlib_wheels[@]} == 0 )); then
  echo "[init_env] 在 $LIB_DIR 下找不到 sptlib_python-*.whl" >&2
  exit 1
fi
echo "[init_env] pip install ${sptlib_wheels[0]}"
pip install --force-reinstall --no-deps "${sptlib_wheels[0]}"

# ---- 验证关键 import ----
python - <<'PY'
import importlib, sys
mods = ['numpy', 'scipy', 'mujoco', 'onnxruntime', 'yaml', 'transforms3d', 'pynput', 'inputs', 'tkinter']
missing = []
for m in mods:
    try:
        importlib.import_module(m)
    except Exception as e:
        missing.append(f"{m}: {e}")
try:
    from sptlib_python import funcSPTrans  # noqa: F401
except Exception as e:
    missing.append(f"sptlib_python: {e}")
if missing:
    print("[init_env] 以下依赖 import 失败：", file=sys.stderr)
    for m in missing:
        print(f"  - {m}", file=sys.stderr)
    sys.exit(1)
print("[init_env] 依赖 import 验证通过")
PY

# ---- 检查 apt 系统依赖（不自动 sudo，仅提示） ----
echo
echo "[init_env] 检查 apt 系统依赖："

deb_files=("$LIB_DIR"/ros-${ROS_DISTRO}-bodyctrl-msgs_*.deb)
if dpkg -l 2>/dev/null | grep "^ii  ros-${ROS_DISTRO}-bodyctrl-msgs"; then
  echo "  [OK] ros-${ROS_DISTRO}-bodyctrl-msgs 已安装"
elif [[ -e "${deb_files[0]:-}" ]]; then
  echo "  [缺失] ros-${ROS_DISTRO}-bodyctrl-msgs —— 自定义 ROS2 消息包，请执行："
  echo "         sudo apt install -y ${deb_files[0]}"
else
  echo "  [警告] 未在 $LIB_DIR 找到 ros-${ROS_DISTRO}-bodyctrl-msgs_*.deb"
fi

cat <<EOF

[init_env] Done. 下一步：
  source $VENV_DIR/bin/activate
  source /opt/ros/${ROS_DISTRO}/setup.bash
然后 ./run_sim.sh 或 ./run_real.sh 即可。
EOF
