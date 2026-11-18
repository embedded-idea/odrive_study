#!/usr/bin/env bash
# ============================================================================
# ODrive 开发环境一键配置脚本
#
# 功能：
#   1. 检查/安装系统依赖（arm-none-eabi 工具链、make、openocd、gdb-multiarch）
#   2. 创建独立虚拟环境 ~/.odrive-venv 并安装 odrivetool 0.5.4
#      （同时装 jinja2/jsonschema/pyyaml，供固件代码生成使用）
#   3. 把 odrivetool 软链接到 ~/.local/bin（PATH 内可直接运行）
#   4. （可选 --with-udev）安装 ODrive USB 权限规则
#
# 用法：
#   bash setup_env.sh                # 基础配置
#   bash setup_env.sh --with-udev    # 基础配置 + udev 规则（需要 sudo）
#
# 脚本可重复执行（幂等），已完成的步骤会自动跳过。
# ============================================================================
set -euo pipefail

VENV="$HOME/.odrive-venv"
BIN_DIR="$HOME/.local/bin"

info() { echo -e "\033[32m[INFO]\033[0m $*"; }
warn() { echo -e "\033[33m[WARN]\033[0m $*"; }
fail() { echo -e "\033[31m[FAIL]\033[0m $*"; exit 1; }

# ---------------- 参数解析 ----------------
WITH_UDEV=0
for arg in "$@"; do
    case "$arg" in
        --with-udev) WITH_UDEV=1 ;;
        *)           warn "未知参数: $arg（忽略）" ;;
    esac
done

need_cmd() { command -v "$1" >/dev/null 2>&1; }

# ---------------- 1. 系统依赖 ----------------
info "步骤 1/4：检查系统依赖"
MISSING_PKGS=()
ensure() { # $1=命令  $2=apt 包名
    if need_cmd "$1"; then
        info "  已安装: $1"
    else
        warn "  缺少:   $1 （apt 包: $2）"
        MISSING_PKGS+=("$2")
    fi
}
ensure make make
ensure arm-none-eabi-gcc gcc-arm-none-eabi
ensure openocd openocd
ensure gdb-multiarch gdb-multiarch
ensure python3 python3
python3 -m pip --version >/dev/null 2>&1 || MISSING_PKGS+=("python3-pip")
python3 -m venv --help >/dev/null 2>&1 || MISSING_PKGS+=("python3-venv")

if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
    info "安装缺失的系统包: ${MISSING_PKGS[*]} （需要 sudo）"
    sudo apt-get update -qq
    sudo apt-get install -y "${MISSING_PKGS[@]}"
else
    info "系统依赖齐全"
fi

# ---------------- 2. 选择 Python ----------------
info "步骤 2/4：选择 Python 版本"
if need_cmd python3.8; then
    PY=python3.8
    info "使用 python3.8（odrive 0.5.4 兼容性最好）"
else
    PY=python3
    warn "未找到 python3.8，回退到 $PY"
    warn "odrive 0.5.4 在 Python 3.11+ 上可能有兼容问题，若失败请安装 python3.8"
fi

# ---------------- 3. 虚拟环境 + odrivetool 0.5.4 ----------------
info "步骤 3/4：配置虚拟环境 $VENV"
if [ ! -d "$VENV" ]; then
    "$PY" -m venv "$VENV"
fi
# 每次都执行安装（pip 对已安装的包会秒级跳过，保证依赖齐全）
"$VENV/bin/pip" install -q "odrive==0.5.4" jinja2 jsonschema pyyaml
"$VENV/bin/odrivetool" --version || fail "odrivetool 运行失败，请检查上方报错"

# ---------------- 4. 软链接 + udev 规则 ----------------
info "步骤 4/4：配置命令与 USB 权限"
mkdir -p "$BIN_DIR"
ln -sf "$VENV/bin/odrivetool" "$BIN_DIR/odrivetool"
info "odrivetool -> $BIN_DIR/odrivetool （已在 PATH 中）"

if [ "$WITH_UDEV" = 1 ]; then
    info "安装 ODrive udev 规则（需要 sudo）"
    sudo bash -c 'curl -fsSL https://cdn.odriverobotics.com/files/odrive-udev-rules.rules -o /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger'
    info "udev 规则已安装，重新插拔 USB 后生效"
else
    info "跳过 udev 规则（首次连接 ODrive 前需要执行："
    echo '  sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"'
    echo "  或运行: bash $0 --with-udev）"
fi

# ---------------- 完成提示 ----------------
echo
info "环境配置完成！"
echo "  - 编译固件:  source $VENV/bin/activate && make"
echo "  - 烧录固件:  openocd -s /usr/share/openocd/scripts -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c 'reset halt' -c 'flash write_image erase build/odrive_min.elf' -c 'reset run' -c exit"
echo "  - 连接 ODrive: odrivetool"

if grep -qi microsoft /proc/version 2>/dev/null; then
    warn "检测到 WSL2：ODrive 的 USB 设备需要先在 Windows 管理员 PowerShell 中挂载："
    echo "    usbipd list                      # 找到 ODrive（VID 1209:0d32）的 BUSID"
    echo "    usbipd bind --busid <x-x>"
    echo "    usbipd attach --wsl --busid <x-x>"
fi
