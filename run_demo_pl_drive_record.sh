#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

stamp="$(date +%Y%m%d_%H%M%S)"
out="${RECORD_VIDEO_PATH:-recordings/drone_track_${stamp}.mp4}"
if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
  out="$1"
  shift
fi

mkdir -p "$(dirname "${out}")"
audio_out="${RECORD_AUDIO_PATH:-${out%.*}_respeaker.wav}"
record_audio="${RECORD_AUDIO:-1}"
audio_args=()

if [ "${record_audio}" = "1" ]; then
  mkdir -p "$(dirname "${audio_out}")"
  audio_device="${RECORD_AUDIO_DEVICE:-plughw:CARD=ArrayUAC10,DEV=0}"
  audio_rate="${RECORD_AUDIO_RATE:-16000}"
  audio_channels="${RECORD_AUDIO_CHANNELS:-4}"
  audio_args=(
    --record-audio "${audio_out}"
    --record-audio-device "${audio_device}"
    --record-audio-rate "${audio_rate}"
    --record-audio-channels "${audio_channels}"
  )
fi

# ReSpeaker cannot be reliably opened twice, so recording mode keeps the normal
# PL vision tracking path and disables drone-audio fallback by default.
export RUN_DEMO_AUDIO_FALLBACK="${RUN_DEMO_AUDIO_FALLBACK:-0}"
export RECORD_VIDEO_FPS="${RECORD_VIDEO_FPS:-30}"

echo "[run_demo_pl_drive_record] video=${out} audio=${audio_out} fps=${RECORD_VIDEO_FPS} audio_fallback=${RUN_DEMO_AUDIO_FALLBACK}"
./run_demo_pl_drive.sh --record-video "${out}" --record-fps "${RECORD_VIDEO_FPS}" "${audio_args[@]}" "$@"
