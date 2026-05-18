#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <distance_m> [extra laser_drone_c_calibrate.py args]" >&2
  echo "example: $0 1.0" >&2
  echo "example: $0 1.5 --stage-spec center:0:3,up:-140:2,down:140:2" >&2
  exit 2
fi

DISTANCE_M="$1"
shift

# Match the final demo camera/YOLO geometry used by run_demo_pl_drive.sh.
# Calibration holds the pan/tilt at fixed poses, but the image path must stay
# identical so bbox_h/cy samples mean the same thing during the real demo.
export YOLO_CONF="${YOLO_CONF:-0.35}"
export YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
export YOLO_SKIP_FRAMES="${YOLO_SKIP_FRAMES:-1}"
export YOLO_FAST_DETECT="${YOLO_FAST_DETECT:-1}"
export YOLO_MIN_BOX_HEIGHT="${YOLO_MIN_BOX_HEIGHT:-4}"
export YOLO_DEVICE="${YOLO_DEVICE:-cuda:0}"
export CAMERA_ROTATE_180="${CAMERA_ROTATE_180:-0}"
export CAMERA_FLIP_VERTICAL="${CAMERA_FLIP_VERTICAL:-0}"
export CAMERA_FLIP_HORIZONTAL="${CAMERA_FLIP_HORIZONTAL:-0}"
export CAMERA_CENTER_CROP="${CAMERA_CENTER_CROP:-0}"
export CAMERA_APPLY_SETTINGS="${CAMERA_APPLY_SETTINGS:-0}"
export CAMERA_APPLY_GLARE_DEFAULTS="${CAMERA_APPLY_GLARE_DEFAULTS:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS="${CAMERA_FPS:-60}"
export CAMERA_FOURCC="${CAMERA_FOURCC:-MJPG}"

if [[ "${LASER_CAL_SKIP_DEPLOY:-0}" != "1" ]]; then
  # Match the final demo Ultra96 bridge limits/profile. The calibration tool
  # uses direct G/D commands, so lead compensation itself is irrelevant here.
  export ULTRA_YUBIN_V1_TRACK_DIRECT_PS="${ULTRA_YUBIN_V1_TRACK_DIRECT_PS:-0}"
  export ULTRA_YUBIN_V1_TRACK_PL_SHADOW="${ULTRA_YUBIN_V1_TRACK_PL_SHADOW:-0}"
  export ULTRA_YUBIN_V1_RESTART="${ULTRA_YUBIN_V1_RESTART:-1}"
  export ULTRA_YUBIN_V1_PROFILE_ACCEL="${ULTRA_YUBIN_V1_PROFILE_ACCEL:-170}"
  export ULTRA_YUBIN_V1_PROFILE_VELOCITY="${ULTRA_YUBIN_V1_PROFILE_VELOCITY:-370}"
  export ULTRA_YUBIN_V1_TRACK_PAN_LIMIT="${ULTRA_YUBIN_V1_TRACK_PAN_LIMIT:-4095}"
  export ULTRA_YUBIN_V1_TRACK_TILT_LIMIT="${ULTRA_YUBIN_V1_TRACK_TILT_LIMIT:-360}"
  set +e
  ./tools/deploy_ultra96_ps_usb.sh
  deploy_status=$?
  set -e
  if [[ "${deploy_status}" -ne 0 ]]; then
    echo "[laser-cal] deploy exited with ${deploy_status}; checking whether bridge is already usable..."
    if ! python3 tools/pl_bringup_check.py --host "${ULTRA_CHAN_HOST:-${ULTRA_YUBIN_HOST:-192.168.3.1}}" --port "${ULTRA_CHAN_PORT:-${ULTRA_YUBIN_PORT:-5016}}" >/tmp/laser_cal_bringup_check.log 2>&1; then
      cat /tmp/laser_cal_bringup_check.log >&2 || true
      exit "${deploy_status}"
    fi
    cat /tmp/laser_cal_bringup_check.log
  fi
fi

echo "[laser-cal] starting staged calibration distance=${DISTANCE_M}m camera=${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS}/${CAMERA_FOURCC} fast=${YOLO_FAST_DETECT} crop=${CAMERA_CENTER_CROP}"
exec /home/jetson/yubin/.venv/bin/python3 -u tools/laser_drone_c_calibrate.py \
  --distance-m "${DISTANCE_M}" \
  --conf "${YOLO_CONF}" \
  --imgsz "${YOLO_IMGSZ}" \
  --base-pan "${LASER_CAL_PAN_TICK:-2048}" \
  --base-tilt "${LASER_CAL_TILT_TICK:-2772}" \
  --stage-spec "${LASER_CAL_STAGE_SPEC:-center:0:3,up:-160:2,down:160:2}" \
  "$@"
