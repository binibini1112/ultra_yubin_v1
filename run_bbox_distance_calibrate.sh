#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

if [ "$#" -lt 1 ]; then
  echo "usage: $0 <distance_m> [extra args]" >&2
  echo "example: $0 2.5" >&2
  exit 2
fi

DISTANCE_M="$1"
shift

# Match the final demo detector settings so bbox height means the same thing
# during calibration and during the real PL-drive run.
MODEL="${YOLO_MODEL_PATH:-${ROOT}/models/drone_best_augmented_0518.engine}"
CONF="${YOLO_CONF:-0.35}"
IMGSZ="${YOLO_IMGSZ:-640}"
OUT="${DISTANCE_MODEL_PATH:-${ROOT}/models/laser_distance_calibration.json}"

# Match the final PL-drive path so calibration is measured while the pan/tilt
# is actually tracking the drone. Skip PL reload by default; run the final demo
# once after a full board power-cycle if the bitstream is not already loaded.
export ULTRA_YUBIN_V1_TRACK_DIRECT_PS=0
export ULTRA_YUBIN_V1_TRACK_PL_SHADOW=0
export ULTRA_YUBIN_V1_RESTART=1
export ULTRA_YUBIN_V1_SKIP_PL_LOAD="${ULTRA_YUBIN_V1_SKIP_PL_LOAD:-1}"
export ULTRA_CHAN_ASYNC_SEND="${ULTRA_CHAN_ASYNC_SEND:-1}"
export ULTRA_YUBIN_V1_PROFILE_ACCEL="${ULTRA_YUBIN_V1_PROFILE_ACCEL:-170}"
export ULTRA_YUBIN_V1_PROFILE_VELOCITY="${ULTRA_YUBIN_V1_PROFILE_VELOCITY:-370}"
export ULTRA_YUBIN_V1_TRACK_PAN_LIMIT="${ULTRA_YUBIN_V1_TRACK_PAN_LIMIT:-4095}"
export ULTRA_YUBIN_V1_TRACK_TILT_LIMIT="${ULTRA_YUBIN_V1_TRACK_TILT_LIMIT:-360}"
export ULTRA_CHAN_CONTROL_PERIOD_SEC="${ULTRA_CHAN_CONTROL_PERIOD_SEC:-0.007}"
export ULTRA_CHAN_DEADBAND_PX="${ULTRA_CHAN_DEADBAND_PX:-1}"
export ULTRA_CHAN_DEADBAND_X_PX="${ULTRA_CHAN_DEADBAND_X_PX:-1}"
export ULTRA_CHAN_DEADBAND_Y_PX="${ULTRA_CHAN_DEADBAND_Y_PX:-1}"
export ULTRA_CHAN_SMOOTH_ALPHA_X="${ULTRA_CHAN_SMOOTH_ALPHA_X:-1.00}"
export ULTRA_CHAN_SMOOTH_ALPHA_Y="${ULTRA_CHAN_SMOOTH_ALPHA_Y:-0.98}"

echo "[bbox_distance_cal] distance=${DISTANCE_M}m model=${MODEL} conf=${CONF} imgsz=${IMGSZ} output=${OUT} drive=pl"
./tools/deploy_ultra96_ps_usb.sh
exec "${PYTHON:-/home/jetson/yubin/.venv/bin/python3}" \
  tools/bbox_distance_calibrate.py \
  --distance-m "${DISTANCE_M}" \
  --model "${MODEL}" \
  --conf "${CONF}" \
  --imgsz "${IMGSZ}" \
  --output "${OUT}" \
  --drive \
  "$@"
