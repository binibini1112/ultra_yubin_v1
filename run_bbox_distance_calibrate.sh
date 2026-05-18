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

echo "[bbox_distance_cal] distance=${DISTANCE_M}m model=${MODEL} conf=${CONF} imgsz=${IMGSZ} output=${OUT}"
exec "${PYTHON:-/home/jetson/yubin/.venv/bin/python3}" \
  tools/bbox_distance_calibrate.py \
  --distance-m "${DISTANCE_M}" \
  --model "${MODEL}" \
  --conf "${CONF}" \
  --imgsz "${IMGSZ}" \
  --output "${OUT}" \
  "$@"
