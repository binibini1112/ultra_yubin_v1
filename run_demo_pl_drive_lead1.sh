#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

# Experimental profile. The stable rollback command remains:
#   ./run_demo_pl_drive.sh --pipeline-echo --pipeline-echo-every 30
#
# This keeps PL-drive enabled, but sends a small predicted bbox center to the
# controller so the gimbal leads moving targets by roughly one frame.
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

echo "[run_demo_pl_drive_lead1] EXPERIMENT lead=${TRACK_LEAD_FRAMES} max_px=${TRACK_LEAD_MAX_PX} motor_min=${TRACK_MOTOR_MIN_CONF} period=${ULTRA_CHAN_CONTROL_PERIOD_SEC}"
exec ./run_demo_pl_drive.sh "$@"
