#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

echo "===== DEPLOY BITSTREAM + BRIDGE ====="
ULTRA_YUBIN_NO_PL=0 \
ULTRA_YUBIN_SKIP_PL_LOAD=0 \
ULTRA_YUBIN_SKIP_PL_INIT=0 \
ULTRA_YUBIN_SKIP_DXL_INIT=0 \
ULTRA_YUBIN_SKIP_CHECK=1 \
ULTRA_YUBIN_RESTART=1 \
./tools/deploy_ultra96_ps_usb.sh

echo "===== PLTEST / PLPING ====="
python3 -c "import socket,sys,time
host='192.168.3.1'
port=5016
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.settimeout(2)
time.sleep(0.5)
s.sendto(b'PLTEST\n',(host,port))
pltest=s.recvfrom(2048)[0].decode().strip()
print(pltest)
s.sendto(b'PLPING\n',(host,port))
plping=s.recvfrom(2048)[0].decode().strip()
print(plping)
if not pltest.startswith('PLTEST,1') or not plping.startswith('PONG,PL'):
    sys.exit(2)
"

echo "===== READY ====="
