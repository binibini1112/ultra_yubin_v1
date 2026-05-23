#!/usr/bin/env python3
import argparse
import math
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from jetson.src.audio_fallback import TelloAudioFallback, estimate_direction_gcc_phat


def _read_audio(path, target_sr):
    sr, data = wavfile.read(str(path))
    data = np.asarray(data)
    if data.ndim == 1:
        data = data[:, None]
    if np.issubdtype(data.dtype, np.integer):
        max_val = float(np.iinfo(data.dtype).max)
        audio = data.astype(np.float32) / max_val
    else:
        audio = data.astype(np.float32)
    if sr != target_sr:
        gcd = math.gcd(int(sr), int(target_sr))
        up = int(target_sr) // gcd
        down = int(sr) // gcd
        audio = resample_poly(audio, up, down, axis=0).astype(np.float32)
        sr = target_sr
    return sr, np.clip(audio, -1.0, 1.0)


def _parse_channels(text, channels):
    if text:
        selected = tuple(int(part.strip()) for part in text.split(",") if part.strip())
    elif channels >= 5:
        selected = (1, 2, 3, 4)
    elif channels >= 4:
        selected = (0, 1, 2, 3)
    else:
        selected = tuple(range(channels))
    if selected and max(selected) >= channels:
        raise ValueError(f"selected channels {selected} exceed wav channel count={channels}")
    return selected


def _section(angle):
    rel = ((float(angle) + 180.0) % 360.0) - 180.0
    if -45 <= rel <= 45:
        return "front"
    if 45 < rel < 135:
        return "right"
    if -135 < rel < -45:
        return "left"
    return "rear"


def evaluate_file(path, args):
    model = TelloAudioFallback(
        args.model,
        args.config,
        alsa_device="offline",
        channels=6,
        threshold=args.threshold,
        min_avg_score=args.min_avg,
        consecutive=args.consecutive,
        min_rms=args.min_rms,
        doa_offset=0,
        doa_method="gcc",
        verbose=False,
    )
    sr, audio = _read_audio(path, model.sample_rate)
    selected = _parse_channels(args.mic_channels, audio.shape[1])
    clip = model.clip_samples
    stride = max(1, int(round(model.sample_rate * args.stride_sec)))
    if audio.shape[0] < clip:
        audio = np.pad(audio, ((0, clip - audio.shape[0]), (0, 0)))

    print(f"\nFILE {path}")
    print(f"  sr={sr} samples={audio.shape[0]} sec={audio.shape[0] / sr:.1f} channels={audio.shape[1]} mic={selected}")
    print(f"  threshold={args.threshold} min_avg={args.min_avg} consecutive={args.consecutive}/{args.window} min_rms={args.min_rms}")
    print(f"  preprocess_target={model.preprocess_target} bandpass={model.bandpass_low:.1f}-{model.bandpass_high:.1f}Hz")

    for gain in args.gains:
        candidates = deque(maxlen=args.window)
        scores = deque(maxlen=args.window)
        hits = 0
        detected = 0
        score_sum = 0.0
        max_score = 0.0
        max_rms = 0.0
        doa_values = []
        best = None
        frames = 0
        for start in range(0, audio.shape[0] - clip + 1, stride):
            raw = np.clip(audio[start : start + clip, :] * gain, -1.0, 1.0)
            if len(selected) >= 4:
                raw_mic = raw[:, selected[:4]]
            else:
                raw_mic = raw[:, selected].mean(axis=1, keepdims=True)
            pre_mic = model._preprocess_mic_audio(raw_mic) if raw_mic.shape[1] >= 4 else raw_mic
            model_mic = raw_mic
            mono = model_mic.mean(axis=1).astype(np.float32)
            rms = float(np.sqrt(np.mean(mono * mono)))
            noise_prob, tello_prob = model._predict_probs(mono)
            candidate = bool(tello_prob >= args.threshold and rms >= args.min_rms)
            candidates.append(candidate)
            scores.append(tello_prob)
            hit_count = int(sum(candidates))
            avg_score = float(np.mean(scores)) if scores else 0.0
            is_detected = len(candidates) >= args.window and hit_count >= args.consecutive and avg_score >= args.min_avg
            if candidate:
                hits += 1
            if is_detected:
                detected += 1
                if pre_mic.shape[1] >= 4:
                    doa = estimate_direction_gcc_phat(
                        pre_mic,
                        fs=model.sample_rate,
                        mic_distance=model.mic_distance,
                        min_peak_ratio=model.gcc_min_peak_ratio,
                    )
                    doa_values.append(float(doa))
            frames += 1
            score_sum += tello_prob
            max_score = max(max_score, tello_prob)
            max_rms = max(max_rms, rms)
            if best is None or tello_prob > best[0]:
                best = (tello_prob, rms, start / sr, avg_score, hit_count)

        avg = score_sum / max(frames, 1)
        doa_text = "-"
        if doa_values:
            unit = np.exp(1j * np.deg2rad(doa_values))
            mean_doa = float(np.rad2deg(np.angle(np.mean(unit))) % 360.0)
            doa_text = f"{mean_doa:.1f}deg/{_section(mean_doa)} n={len(doa_values)}"
        best_text = "-"
        if best is not None:
            best_text = f"t={best[2]:.1f}s score={best[0]:.3f} rms={best[1]:.4f} avg={best[3]:.3f} hits={best[4]}"
        print(
            f"  gain={gain:>5.3f} frames={frames:4d} cand={hits:4d} det={detected:4d} "
            f"avg={avg:.3f} max={max_score:.3f} max_rms={max_rms:.4f} doa={doa_text} best={best_text}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+")
    parser.add_argument("--model", default=str(ROOT / "model" / "tello_detector_cnn_retrained_jetson.tflite"))
    parser.add_argument("--config", default=str(ROOT / "model" / "config.json"))
    parser.add_argument("--threshold", type=float, default=float(os.getenv("TELLO_AUDIO_THRESHOLD", "0.50")))
    parser.add_argument("--min-avg", type=float, default=float(os.getenv("TELLO_AUDIO_MIN_AVG_SCORE", "0.55")))
    parser.add_argument("--consecutive", type=int, default=int(os.getenv("TELLO_AUDIO_CONSECUTIVE", "2")))
    parser.add_argument("--window", type=int, default=int(os.getenv("TELLO_AUDIO_WINDOW_SIZE", "3")))
    parser.add_argument("--min-rms", type=float, default=float(os.getenv("TELLO_AUDIO_MIN_RMS", "0.003")))
    parser.add_argument("--stride-sec", type=float, default=0.25)
    parser.add_argument("--mic-channels", default=os.getenv("TELLO_AUDIO_MIC_CHANNELS", ""))
    parser.add_argument(
        "--gains",
        type=float,
        nargs="+",
        default=[2.0, 1.0, 0.5, 0.25, 0.125, 0.0625],
        help="Amplitude multipliers. Lower gain approximates farther/quieter playback.",
    )
    args = parser.parse_args()
    for item in args.files:
        evaluate_file(Path(item), args)


if __name__ == "__main__":
    main()
