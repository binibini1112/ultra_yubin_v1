#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

# Comparison profile: motors are still driven by PS direct, but the bridge also
# runs the PL controller on the same target and prints pl_diff_* in replies.
export ULTRA_YUBIN_V1_TRACK_DIRECT_PS=1
export ULTRA_YUBIN_V1_TRACK_PL_SHADOW=1
export ULTRA_YUBIN_V1_RESTART=1
export ULTRA_YUBIN_V1_PROFILE_ACCEL="${ULTRA_YUBIN_V1_PROFILE_ACCEL:-60}"
export ULTRA_YUBIN_V1_PROFILE_VELOCITY="${ULTRA_YUBIN_V1_PROFILE_VELOCITY:-180}"

export ULTRA_CHAN_CONTROL_PERIOD_SEC="${ULTRA_CHAN_CONTROL_PERIOD_SEC:-0.018}"
export ULTRA_CHAN_SMOOTH_ALPHA_X="${ULTRA_CHAN_SMOOTH_ALPHA_X:-0.82}"
export ULTRA_CHAN_SMOOTH_ALPHA_Y="${ULTRA_CHAN_SMOOTH_ALPHA_Y:-0.76}"
export TARGET_CENTER_SMOOTHING_NEAR="${TARGET_CENTER_SMOOTHING_NEAR:-0.70}"
export TARGET_CENTER_SMOOTHING_FAR="${TARGET_CENTER_SMOOTHING_FAR:-0.92}"
export TARGET_MOTOR_MAX_STEP_PX="${TARGET_MOTOR_MAX_STEP_PX:-260}"
export TARGET_MAX_CENTER_STEP_PX="${TARGET_MAX_CENTER_STEP_PX:-360}"
export TRACK_HOLD_MOTOR_ON_LOST="${TRACK_HOLD_MOTOR_ON_LOST:-1}"

./tools/deploy_ultra96_ps_usb.sh
exec ./run_demo.sh "$@"
