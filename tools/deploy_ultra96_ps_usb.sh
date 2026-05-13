#!/usr/bin/env bash
set -euo pipefail

HOST="${ULTRA96_HOST:-192.168.3.1}"
USER="${ULTRA96_USER:-xilinx}"
REMOTE_ROOT="${ULTRA_REMOTE_ROOT:-/home/xilinx/ultra_yubin}"
SERVICE="${ULTRA_SERVICE:-ultra-yubin.service}"
PORT="${ULTRA_YUBIN_PORT:-5016}"
BASE="${ULTRA_YUBIN_BASE:-0xA0000000}"
PAN_ID="${PAN_ID:-1}"
TILT_ID="${TILT_ID:-2}"
SERIAL="${ULTRA_YUBIN_SERIAL:-/dev/ttyUSB0}"
BAUD="${ULTRA_YUBIN_BAUD:-57600}"
DRY_RUN="${ULTRA_YUBIN_DRY_RUN:-auto}"
NO_PL="${ULTRA_YUBIN_NO_PL:-1}"
RESTART="${ULTRA_YUBIN_RESTART:-0}"
SKIP_CHECK="${ULTRA_YUBIN_SKIP_CHECK:-1}"
SKIP_PL_LOAD="${ULTRA_YUBIN_SKIP_PL_LOAD:-1}"
SKIP_PL_INIT="${ULTRA_YUBIN_SKIP_PL_INIT:-1}"
LAZY_PL_OPEN="${ULTRA_YUBIN_LAZY_PL_OPEN:-1}"
SUDO_PASS="${ULTRA96_SUDO_PASSWORD:-xilinx}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BIT="${ROOT}/bitstream/ultra_yubin.bit"
HWH="${ROOT}/bitstream/ultra_yubin.hwh"
BRIDGE="${ROOT}/hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c"

test -s "${BIT}" || { echo "missing ${BIT}" >&2; exit 2; }
test -s "${BRIDGE}" || { echo "missing ${BRIDGE}" >&2; exit 2; }

echo "[ultra-yubin] target=${USER}@${HOST} remote=${REMOTE_ROOT} port=${PORT} base=${BASE}"
ssh "${USER}@${HOST}" "mkdir -p '${REMOTE_ROOT}'"

echo "[ultra-yubin] stopping previous ultra_yubin service/process"
ssh "${USER}@${HOST}" "printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl disable --now '${SERVICE}' >/dev/null 2>&1 || true
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' rm -f \
  /etc/systemd/system/'${SERVICE}' \
  /etc/systemd/system/multi-user.target.wants/'${SERVICE}'
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl daemon-reload"

SERVICE_ACTIVE="$(ssh "${USER}@${HOST}" "systemctl is-active '${SERVICE}' 2>/dev/null || true" | tr -d '\r')"
SERIAL_PRESENT="$(ssh "${USER}@${HOST}" "test -e '${SERIAL}' && echo 1 || echo 0" | tr -d '\r')"

EFFECTIVE_DRY_RUN="${DRY_RUN}"
if [ "${DRY_RUN}" = "auto" ]; then
  if [ "${SERIAL_PRESENT}" = "1" ]; then
    EFFECTIVE_DRY_RUN="0"
  else
    EFFECTIVE_DRY_RUN="1"
  fi
fi

EFFECTIVE_NO_PL="${NO_PL}"
if [ "${NO_PL}" = "auto" ]; then
  EFFECTIVE_NO_PL="${EFFECTIVE_DRY_RUN}"
fi

echo "[ultra-yubin] service=${SERVICE_ACTIVE:-inactive} serial_present=${SERIAL_PRESENT} dry=${EFFECTIVE_DRY_RUN} no_pl=${EFFECTIVE_NO_PL} restart=${RESTART}"
echo "[ultra-yubin] skip_pl_load=${SKIP_PL_LOAD} skip_pl_init=${SKIP_PL_INIT} lazy_pl_open=${LAZY_PL_OPEN}"
if [ "${SERVICE_ACTIVE}" = "active" ] && [ "${RESTART}" != "1" ]; then
  echo "[ultra-yubin] service already active; updating files without restart."
  echo "[ultra-yubin] set ULTRA_YUBIN_RESTART=1 to force service restart."
fi

scp "${BIT}" "${USER}@${HOST}:${REMOTE_ROOT}/ultra_yubin.bit"
if [ -s "${HWH}" ]; then
  scp "${HWH}" "${USER}@${HOST}:${REMOTE_ROOT}/ultra_yubin.hwh"
fi
scp "${BRIDGE}" "${USER}@${HOST}:${REMOTE_ROOT}/pl_udp_usb_dxl_bridge.c"

ssh "${USER}@${HOST}" "cd '${REMOTE_ROOT}' && gcc -O2 -Wall -o pl_udp_usb_dxl_bridge pl_udp_usb_dxl_bridge.c"

dry_arg=""
if [ "${EFFECTIVE_DRY_RUN}" = "1" ]; then
  dry_arg=" --dry-run"
fi
no_pl_arg=""
if [ "${EFFECTIVE_NO_PL}" = "1" ]; then
  no_pl_arg=" --no-pl"
fi
skip_pl_init_arg=""
if [ "${SKIP_PL_INIT}" = "1" ]; then
  skip_pl_init_arg=" --skip-pl-init"
fi
lazy_pl_open_arg=""
if [ "${LAZY_PL_OPEN}" = "1" ]; then
  lazy_pl_open_arg=" --lazy-pl-open"
fi

ssh "${USER}@${HOST}" "printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl disable --now '${SERVICE}' >/dev/null 2>&1 || true
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' rm -f '/etc/systemd/system/${SERVICE}'
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl daemon-reload
for pid in \$(pidof pl_udp_usb_dxl_bridge 2>/dev/null || true); do
  ppid=\$(awk '/^PPid:/ {print \$2}' /proc/\"\$pid\"/status 2>/dev/null || true)
  printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' kill -9 \"\$pid\" 2>/dev/null || true
  if [ -n \"\$ppid\" ] && [ \"\$ppid\" != '1' ]; then
    printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' kill -9 \"\$ppid\" 2>/dev/null || true
  fi
done
if [ '${SKIP_PL_LOAD}' = '1' ]; then
  echo 'skip-pl-load mode: keeping current FPGA image'
elif [ '${EFFECTIVE_NO_PL}' != '1' ]; then
  printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' cp '${REMOTE_ROOT}/ultra_yubin.bit' /lib/firmware/ultra_yubin.bit
  printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' sh -c 'echo ultra_yubin.bit > /sys/class/fpga_manager/fpga0/firmware'
  sleep 1
  cat /sys/class/fpga_manager/fpga0/state
else
  echo 'no-pl mode: skipping FPGA manager reload'
fi
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' setsid '${REMOTE_ROOT}/pl_udp_usb_dxl_bridge' --base '${BASE}' --port '${PORT}' --serial '${SERIAL}' --baud '${BAUD}' --pan-id '${PAN_ID}' --tilt-id '${TILT_ID}'${dry_arg}${no_pl_arg}${skip_pl_init_arg}${lazy_pl_open_arg} >/tmp/ultra_yubin_manual.log 2>&1 < /dev/null &
sleep 1
pidof pl_udp_usb_dxl_bridge || { cat /tmp/ultra_yubin_manual.log; exit 1; }"

if [ "${SKIP_CHECK}" = "1" ]; then
  echo "[ultra-yubin] skipping bring-up check because ULTRA_YUBIN_SKIP_CHECK=1"
else
  cd "${ROOT}"
  python3 tools/pl_bringup_check.py --host "${HOST}" --port "${PORT}"
fi
