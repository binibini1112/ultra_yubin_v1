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

# Audio fallback settings live in run_demo_pl_drive.sh so this wrapper cannot
# accidentally point at an external teammate folder.

echo "[run_demo_pl_drive_lead1] alias -> run_demo_pl_drive lead=${TRACK_LEAD_FRAMES} max_px=${TRACK_LEAD_MAX_PX} motor_min=${TRACK_MOTOR_MIN_CONF} period=${ULTRA_CHAN_CONTROL_PERIOD_SEC}"
exec ./run_demo_pl_drive.sh "$@"
