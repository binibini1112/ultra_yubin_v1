#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

PY="${PYTHON:-/home/jetson/yubin/.venv/bin/python3}"
MODEL="${TELLO_AUDIO_JUNMO_MODEL:-/home/jetson/junmoyolo26/tello_detector.tflite}"
PIPELINE="${TELLO_AUDIO_JUNMO_PIPELINE:-${ROOT}/jetson/src/audio/junmo_pipeline_drone.py}"
pkill -9 arecord >/dev/null 2>&1 || true
export TELLO_AUDIO_USB_RESET_ON_ERROR="${TELLO_AUDIO_USB_RESET_ON_ERROR:-0}"
if [ -n "${TELLO_AUDIO_ALSA_DEVICE:-}" ]; then
  ALSA_DEVICE="${TELLO_AUDIO_ALSA_DEVICE}"
else
  ALSA_DEVICE="$(
    arecord -l | awk '
      /ReSpeaker|ArrayUAC10|Seeed/ {
        card=""; dev="";
        for (i = 1; i <= NF; i++) {
          if ($i == "card") { card=$(i+1); gsub(":", "", card); }
          if ($i == "device") { dev=$(i+1); gsub(":", "", dev); }
        }
        if (card != "" && dev != "") {
          print "plughw:" card "," dev;
          exit;
        }
      }
    '
  )"
  ALSA_DEVICE="${ALSA_DEVICE:-plughw:CARD=ArrayUAC10,DEV=0}"
fi

exec "${PY}" "${PIPELINE}" \
  --channels "${TELLO_AUDIO_CHANNELS:-4}" \
  --model "${MODEL}" \
  --threshold "${TELLO_AUDIO_THRESHOLD:-0.70}" \
  --consecutive "${TELLO_AUDIO_CONSECUTIVE:-2}" \
  --cooldown "${TELLO_AUDIO_COOLDOWN_SEC:-0.60}" \
  --min-rms "${TELLO_AUDIO_MIN_RMS:-0.008}" \
  --doa-method "${TELLO_AUDIO_DOA_METHOD:-gcc}" \
  --audio-backend "${TELLO_AUDIO_BACKEND:-arecord}" \
  --alsa-device "${ALSA_DEVICE}"
