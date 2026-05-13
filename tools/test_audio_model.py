#!/usr/bin/env python3
"""Smoke test for the Tello audio model files copied from junmoyolo26."""

import json
import os
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "models", "tello_audio_config.json")
TFLITE = os.path.join(ROOT, "models", "tello_detector.tflite")
KERAS = os.path.join(ROOT, "models", "tello_detector.keras")


def main():
    for path in (CONFIG, TFLITE, KERAS):
        if not os.path.exists(path):
            raise SystemExit(f"missing: {path}")

    with open(CONFIG, "r", encoding="utf-8") as fp:
        cfg = json.load(fp)
    print(f"config: sr={cfg.get('sr')} clip_len={cfg.get('clip_len')} classes={cfg.get('classes')}")

    try:
        import tflite_runtime.interpreter as tflite
    except ImportError as exc:
        raise SystemExit(f"tflite_runtime not installed: {exc}") from exc

    interpreter = tflite.Interpreter(model_path=TFLITE)
    interpreter.allocate_tensors()
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]
    print(f"tflite input={inp['shape']} output={out['shape']}")
    print("audio model smoke test OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
