#!/usr/bin/env bash
set -euo pipefail

HOST="${ULTRA96_HOST:-192.168.3.1}"
USER="${ULTRA96_USER:-xilinx}"
REMOTE_ROOT="${ULTRA_REMOTE_ROOT:-/home/xilinx/ultra_yubin}"
SUDO_PASS="${ULTRA96_SUDO_PASSWORD:-xilinx}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIT="${ROOT}/bitstream/ultra_yubin.bit"
HWH="${ROOT}/bitstream/ultra_yubin.hwh"

test -s "${BIT}" || { echo "missing ${BIT}" >&2; exit 2; }

echo "[load-bit] target=${USER}@${HOST}"
ssh "${USER}@${HOST}" "mkdir -p '${REMOTE_ROOT}'"
scp "${BIT}" "${USER}@${HOST}:${REMOTE_ROOT}/ultra_yubin.bit"
if [ -s "${HWH}" ]; then
  scp "${HWH}" "${USER}@${HOST}:${REMOTE_ROOT}/ultra_yubin.hwh"
fi

ssh "${USER}@${HOST}" "set -e
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl disable --now ultra-yubin.service >/dev/null 2>&1 || true
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' rm -f \
  /etc/systemd/system/ultra-yubin.service \
  /etc/systemd/system/multi-user.target.wants/ultra-yubin.service
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl daemon-reload
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' cp '${REMOTE_ROOT}/ultra_yubin.bit' /lib/firmware/ultra_yubin.bit
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' sh -c 'echo ultra_yubin.bit > /sys/class/fpga_manager/fpga0/firmware'
sleep 1
cat /sys/class/fpga_manager/fpga0/state
ip -br addr"
