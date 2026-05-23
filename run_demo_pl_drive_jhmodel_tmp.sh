#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"

# Temporary profile for JH raw 4ch 1D-CNN direction model.
# It does not change the default run_demo_pl_drive.sh profile.
export TELLO_AUDIO_DIRECTION_MODE="${TELLO_AUDIO_DIRECTION_MODE:-cnn6}"
export TELLO_AUDIO_DIRECTION_MODEL="${TELLO_AUDIO_DIRECTION_MODEL:-${ROOT}/jhmodle/audio_angle_4ch_1dcnn_float32.tflite}"
export TELLO_AUDIO_DIRECTION_FEATURE_CONFIG="${TELLO_AUDIO_DIRECTION_FEATURE_CONFIG:-${ROOT}/jhmodle/audio_angle_4ch_1dcnn_deploy_config.json}"
unset TELLO_AUDIO_DIRECTION_LABEL_MAPPING

echo "[run_demo_pl_drive_jhmodel_tmp] using JH direction model: ${TELLO_AUDIO_DIRECTION_MODEL}"
exec "${ROOT}/run_demo_pl_drive.sh" "$@"
