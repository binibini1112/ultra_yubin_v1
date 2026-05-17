#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

# Keep ultra_yubin_v1 camera orientation/image path.
# Do not import crane_jh rotate/flip behavior.
export CAMERA_ROTATE_180="${CAMERA_ROTATE_180:-0}"
export CAMERA_FLIP_VERTICAL="${CAMERA_FLIP_VERTICAL:-0}"
export CAMERA_FLIP_HORIZONTAL="${CAMERA_FLIP_HORIZONTAL:-0}"
export CAMERA_CENTER_CROP="${CAMERA_CENTER_CROP:-0}"

# Keep the current yubin camera stream quality unless explicitly overridden.
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS="${CAMERA_FPS:-60}"
export CAMERA_FOURCC="${CAMERA_FOURCC:-MJPG}"

# Recognition test profile copied from the better-recognizing crane_jh path.
export YOLO_CONF="${YOLO_CONF:-0.35}"
export YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
export YOLO_SKIP_FRAMES="${YOLO_SKIP_FRAMES:-1}"
export YOLO_FAST_DETECT="${YOLO_FAST_DETECT:-0}"
export YOLO_MIN_BOX_HEIGHT="${YOLO_MIN_BOX_HEIGHT:-4}"
export YOLO_DEVICE="${YOLO_DEVICE:-cuda:0}"

# Make target selection less fragile while testing single-drone detection.
export TRACK_TARGET_MIN_CONF="${TRACK_TARGET_MIN_CONF:-0.50}"
export TRACK_STICKY_MAX_DIST_PX="${TRACK_STICKY_MAX_DIST_PX:-10000}"
export TRACK_REACQUIRE_MIN_CONF="${TRACK_REACQUIRE_MIN_CONF:-0.60}"
export TRACK_WEAK_REACQUIRE_CONF="${TRACK_WEAK_REACQUIRE_CONF:-0.45}"

echo "[yolo-crane-test] yubin camera kept: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS} ${CAMERA_FOURCC}, rotate=0 flip=0"
echo "[yolo-crane-test] YOLO: conf=${YOLO_CONF} imgsz=${YOLO_IMGSZ} fast_detect=${YOLO_FAST_DETECT} skip=${YOLO_SKIP_FRAMES}"
echo "[yolo-crane-test] tracking: target_min_conf=${TRACK_TARGET_MIN_CONF} sticky_px=${TRACK_STICKY_MAX_DIST_PX}"

exec ./run_demo.sh "$@"
