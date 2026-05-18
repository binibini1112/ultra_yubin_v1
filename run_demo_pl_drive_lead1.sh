#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

# Compatibility wrapper. The lead profile is now the default:
#   ./run_demo_pl_drive.sh --pipeline-echo --pipeline-echo-every 30
#
# This wrapper keeps old commands working and then delegates to run_demo_pl_drive.sh.
export TRACK_LEAD_FRAMES="${TRACK_LEAD_FRAMES:-1.0}"
export TRACK_LEAD_MAX_PX="${TRACK_LEAD_MAX_PX:-70}"
export TRACK_LEAD_MIN_CONF="${TRACK_LEAD_MIN_CONF:-0.65}"
export TRACK_LEAD_VELOCITY_ALPHA="${TRACK_LEAD_VELOCITY_ALPHA:-0.55}"
export TRACK_LEAD_RESET_JUMP_PX="${TRACK_LEAD_RESET_JUMP_PX:-220}"

# Slightly reduce false motor commands from weak boxes while preserving reacquire.
export TRACK_TARGET_MIN_CONF="${TRACK_TARGET_MIN_CONF:-0.52}"
export TRACK_MOTOR_MIN_CONF="${TRACK_MOTOR_MIN_CONF:-0.60}"
export TRACK_REACQUIRE_MIN_CONF="${TRACK_REACQUIRE_MIN_CONF:-0.65}"
export TRACK_WEAK_REACQUIRE_CONF="${TRACK_WEAK_REACQUIRE_CONF:-0.50}"
export TRACK_WEAK_REACQUIRE_AREA="${TRACK_WEAK_REACQUIRE_AREA:-80}"

# One notch more responsive than the stable PL-drive profile.
export ULTRA_CHAN_CONTROL_PERIOD_SEC="${ULTRA_CHAN_CONTROL_PERIOD_SEC:-0.007}"
export ULTRA_YUBIN_V1_PROFILE_ACCEL="${ULTRA_YUBIN_V1_PROFILE_ACCEL:-170}"
export ULTRA_YUBIN_V1_PROFILE_VELOCITY="${ULTRA_YUBIN_V1_PROFILE_VELOCITY:-370}"
# Mechanical pan window for this rig is -90 <= pan < 270 degrees from front.
# That is nearly one full turn; keep the hard DXL 0..4095 range as the last
# guard so the cable cannot be driven past the wrap boundary.
export ULTRA_YUBIN_V1_TRACK_PAN_LIMIT="${ULTRA_YUBIN_V1_TRACK_PAN_LIMIT:-4095}"

# Always keep drone-audio fallback enabled for this demo profile. When YOLO has
# no active target, Jetson sends the detected audio direction to the Ultra96 PS
# audio command path, which turns pan toward the sound while PL-drive remains
# responsible for bbox tracking.
export TELLO_AUDIO_FALLBACK=1
export TELLO_AUDIO_MODE="${TELLO_AUDIO_MODE:-junmo}"
export TELLO_AUDIO_JUNMO_MODEL="${TELLO_AUDIO_JUNMO_MODEL:-/home/jetson/junmoyolo26/tello_detector.tflite}"
export TELLO_AUDIO_THRESHOLD="${TELLO_AUDIO_THRESHOLD:-0.86}"
export TELLO_AUDIO_MIN_AVG_SCORE="${TELLO_AUDIO_MIN_AVG_SCORE:-0.90}"
export TELLO_AUDIO_CONSECUTIVE="${TELLO_AUDIO_CONSECUTIVE:-3}"
export TELLO_AUDIO_COOLDOWN_SEC="${TELLO_AUDIO_COOLDOWN_SEC:-0.8}"
export TELLO_AUDIO_LAZY_START="${TELLO_AUDIO_LAZY_START:-1}"
export TELLO_AUDIO_LAZY_START_AFTER_SEC="${TELLO_AUDIO_LAZY_START_AFTER_SEC:-0.8}"
export TELLO_AUDIO_VERBOSE="${TELLO_AUDIO_VERBOSE:-0}"
export TELLO_AUDIO_USB_RESET_ON_ERROR="${TELLO_AUDIO_USB_RESET_ON_ERROR:-1}"
export TELLO_AUDIO_MOTOR_ZERO_DOA_DEG="${TELLO_AUDIO_MOTOR_ZERO_DOA_DEG:-90}"
export TELLO_AUDIO_DOA_SIGN="${TELLO_AUDIO_DOA_SIGN:-1}"
export TELLO_AUDIO_STABLE_WINDOW="${TELLO_AUDIO_STABLE_WINDOW:-5}"
export TELLO_AUDIO_STABLE_MIN_VOTES="${TELLO_AUDIO_STABLE_MIN_VOTES:-3}"
export TELLO_AUDIO_STABLE_MAX_SPREAD_DEG="${TELLO_AUDIO_STABLE_MAX_SPREAD_DEG:-25}"
export TELLO_AUDIO_CONTROL_PERIOD_SEC="${TELLO_AUDIO_CONTROL_PERIOD_SEC:-0.45}"
export TELLO_AUDIO_KEEPALIVE_SEC="${TELLO_AUDIO_KEEPALIVE_SEC:-1.0}"
export TELLO_AUDIO_MIN_CHANGE_DEG="${TELLO_AUDIO_MIN_CHANGE_DEG:-12}"
export TELLO_AUDIO_CLAMP_DEG="${TELLO_AUDIO_CLAMP_DEG:-180}"
export TELLO_AUDIO_REJECT_REAR="${TELLO_AUDIO_REJECT_REAR:-0}"
export TELLO_AUDIO_VISION_HOLD_SEC="${TELLO_AUDIO_VISION_HOLD_SEC:-0.15}"
export TELLO_AUDIO_DIRECTION_BIN_DEG="${TELLO_AUDIO_DIRECTION_BIN_DEG:-15}"
export TELLO_AUDIO_DIRECTION_COOLDOWN_SEC="${TELLO_AUDIO_DIRECTION_COOLDOWN_SEC:-1.5}"
export TELLO_AUDIO_MAX_SEARCH_ATTEMPTS="${TELLO_AUDIO_MAX_SEARCH_ATTEMPTS:-0}"
export TELLO_AUDIO_ATTEMPT_RESET_SEC="${TELLO_AUDIO_ATTEMPT_RESET_SEC:-5.0}"
export TELLO_AUDIO_DOA_ONLY_SEARCH="${TELLO_AUDIO_DOA_ONLY_SEARCH:-0}"

echo "[run_demo_pl_drive_lead1] alias -> run_demo_pl_drive lead=${TRACK_LEAD_FRAMES} max_px=${TRACK_LEAD_MAX_PX} motor_min=${TRACK_MOTOR_MIN_CONF} period=${ULTRA_CHAN_CONTROL_PERIOD_SEC} audio=${TELLO_AUDIO_MODE}"
exec ./run_demo_pl_drive.sh "$@"
