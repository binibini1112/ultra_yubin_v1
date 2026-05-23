#!/usr/bin/env python3
import argparse
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, resample_poly, sosfilt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from jetson.src.audio_fallback import TelloAudioFallback, estimate_direction_gcc_phat


def read_audio(path, target_sr):
    sr, data = wavfile.read(str(path))
    data = np.asarray(data)
    if data.ndim == 1:
        data = data[:, None]
    if np.issubdtype(data.dtype, np.integer):
        audio = data.astype(np.float32) / float(np.iinfo(data.dtype).max)
    else:
        audio = data.astype(np.float32)
    if int(sr) != int(target_sr):
        gcd = math.gcd(int(sr), int(target_sr))
        audio = resample_poly(audio, int(target_sr) // gcd, int(sr) // gcd, axis=0).astype(np.float32)
        sr = target_sr
    return sr, np.clip(audio, -1.0, 1.0)


def parse_channels(text, channels):
    if text:
        selected = tuple(int(part.strip()) for part in text.split(",") if part.strip())
    elif channels >= 5:
        selected = (1, 2, 3, 4)
    elif channels >= 4:
        selected = (0, 1, 2, 3)
    else:
        raise ValueError(f"need at least 4 channels, got {channels}")
    if len(selected) != 4:
        raise ValueError(f"need exactly four mic channels, got {selected}")
    if min(selected) < 0 or max(selected) >= channels:
        raise ValueError(f"selected channels {selected} exceed wav channel count={channels}")
    return selected


def section(angle):
    rel = ((float(angle) + 180.0) % 360.0) - 180.0
    if -45.0 <= rel <= 45.0:
        return "front"
    if 45.0 < rel < 135.0:
        return "right"
    if -135.0 < rel < -45.0:
        return "left"
    return "rear"


def circular_stats(angles):
    if not angles:
        return None, 0.0, 999.0
    unit = np.exp(1j * np.deg2rad(angles))
    mean_vec = np.mean(unit)
    r = float(np.abs(mean_vec))
    mean = float(np.rad2deg(np.angle(mean_vec)) % 360.0)
    circ_std = float(np.sqrt(max(0.0, -2.0 * np.log(max(r, 1e-9)))) * 180.0 / np.pi)
    return mean, r, circ_std


def make_sos(low, high, sr, order):
    if low <= 0 and high <= 0:
        return None
    nyq = 0.5 * float(sr)
    if low <= 0:
        return butter(order, min(high, nyq - 1.0) / nyq, btype="lowpass", output="sos")
    if high <= 0:
        return butter(order, min(low, nyq - 1.0) / nyq, btype="highpass", output="sos")
    low = max(1.0, min(float(low), nyq - 2.0))
    high = max(low + 1.0, min(float(high), nyq - 1.0))
    return butter(order, [low / nyq, high / nyq], btype="bandpass", output="sos")


def preprocess_for_doa(mic, sos, gain_target, noise_floor):
    if sos is None:
        return np.asarray(mic, dtype=np.float32)
    filtered = sosfilt(sos, mic, axis=0).astype(np.float32)
    max_amp = float(np.max(np.abs(filtered))) if filtered.size else 0.0
    if max_amp <= float(noise_floor):
        return np.zeros_like(filtered, dtype=np.float32)
    return np.clip((filtered / max(max_amp, 1e-9)) * float(gain_target), -1.0, 1.0).astype(np.float32)


def evaluate_band(model, audio, selected, args, low, high, gain):
    clip = model.clip_samples
    stride = max(1, int(round(model.sample_rate * args.stride_sec)))
    sos = make_sos(low, high, model.sample_rate, args.order)
    angles = []
    raw_scores = []
    candidates = 0
    quiet = 0
    sections = Counter()
    examples = []
    for frame_idx, start in enumerate(range(0, audio.shape[0] - clip + 1, stride)):
        if args.max_windows and frame_idx >= args.max_windows:
            break
        frame = np.clip(audio[start : start + clip, :] * float(gain), -1.0, 1.0)
        mic = frame[:, selected]
        mono = mic.mean(axis=1).astype(np.float32)
        rms = float(np.sqrt(np.mean(mono * mono)))
        _, score = model._predict_probs(mono)
        raw_scores.append(score)
        if score < args.threshold or rms < args.min_rms:
            if rms < args.min_rms:
                quiet += 1
            continue
        candidates += 1
        doa_mic = preprocess_for_doa(mic, sos, args.gain_target, args.noise_floor)
        if float(np.max(np.abs(doa_mic))) <= 1e-8:
            continue
        try:
            angle = float(
                estimate_direction_gcc_phat(
                    doa_mic,
                    fs=model.sample_rate,
                    mic_distance=args.mic_distance,
                    min_peak_ratio=args.peak_ratio,
                )
            )
        except Exception:
            continue
        angles.append(angle)
        sections[section(angle)] += 1
        if len(examples) < args.examples:
            examples.append(f"t={start / model.sample_rate:.2f}s score={score:.3f} rms={rms:.4f} doa={angle:.1f}/{section(angle)}")
    mean, r, circ_std = circular_stats(angles)
    section_text = "-"
    section_ratio = 0.0
    if sections:
        name, count = sections.most_common(1)[0]
        section_ratio = count / max(1, sum(sections.values()))
        section_text = f"{name}:{count}/{sum(sections.values())}={section_ratio:.2f}"
    score_avg = float(np.mean(raw_scores)) if raw_scores else 0.0
    label = "raw" if low <= 0 and high <= 0 else f"{low:.0f}-{high:.0f}"
    return {
        "label": label,
        "gain": gain,
        "frames": len(raw_scores),
        "candidates": candidates,
        "quiet": quiet,
        "angles": len(angles),
        "mean": mean,
        "r": r,
        "circ_std": circ_std,
        "section": section_text,
        "section_ratio": section_ratio,
        "score_avg": score_avg,
        "examples": examples,
    }


def evaluate_file(path, args):
    model = TelloAudioFallback(
        args.model,
        args.config,
        alsa_device="offline",
        channels=6,
        threshold=args.threshold,
        min_avg_score=args.min_avg,
        consecutive=2,
        min_rms=args.min_rms,
        doa_offset=0,
        doa_method="gcc",
        verbose=False,
    )
    sr, audio = read_audio(path, model.sample_rate)
    selected = parse_channels(args.mic_channels, audio.shape[1])
    if audio.shape[0] < model.clip_samples:
        audio = np.pad(audio, ((0, model.clip_samples - audio.shape[0]), (0, 0)))
    print(f"\nFILE {path}")
    print(f"  sr={sr} sec={audio.shape[0] / sr:.1f} channels={audio.shape[1]} mic={selected}")
    print(f"  threshold={args.threshold} min_rms={args.min_rms} peak_ratio={args.peak_ratio} stride={args.stride_sec}s")
    for gain in args.gains:
        results = []
        for low, high in args.bands:
            results.append(evaluate_band(model, audio, selected, args, low, high, gain))
        results.sort(key=lambda item: (item["section_ratio"], item["r"], item["angles"]), reverse=True)
        print(f"  gain={gain:.3f}")
        for item in results:
            mean = "-" if item["mean"] is None else f"{item['mean']:.1f}"
            print(
                f"    band={item['label']:<9} frames={item['frames']:3d} cand={item['candidates']:3d} "
                f"doa_n={item['angles']:3d} score_avg={item['score_avg']:.3f} "
                f"mean={mean:>5} R={item['r']:.2f} std={item['circ_std']:.1f} sec={item['section']}"
            )
        if args.examples and results:
            best = results[0]
            print(f"    examples best_band={best['label']}")
            for example in best["examples"]:
                print(f"      {example}")


def parse_band(text):
    if text.lower() == "raw":
        return (0.0, 0.0)
    if "-" not in text:
        raise argparse.ArgumentTypeError("band must be raw or LOW-HIGH")
    low, high = text.split("-", 1)
    return (float(low), float(high))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+")
    parser.add_argument("--model", default=str(ROOT / "model" / "tello_detector_cnn_retrained_jetson.tflite"))
    parser.add_argument("--config", default=str(ROOT / "model" / "config.json"))
    parser.add_argument("--threshold", type=float, default=0.50)
    parser.add_argument("--min-avg", type=float, default=0.55)
    parser.add_argument("--min-rms", type=float, default=0.003)
    parser.add_argument("--stride-sec", type=float, default=0.25)
    parser.add_argument("--mic-channels", default="")
    parser.add_argument("--mic-distance", type=float, default=0.045)
    parser.add_argument("--peak-ratio", type=float, default=1.15)
    parser.add_argument("--order", type=int, default=5)
    parser.add_argument("--gain-target", type=float, default=0.90)
    parser.add_argument("--noise-floor", type=float, default=0.003)
    parser.add_argument("--gains", type=float, nargs="+", default=[1.0, 0.5])
    parser.add_argument(
        "--bands",
        type=parse_band,
        nargs="+",
        default=[
            (0.0, 0.0),
            (1000.0, 3000.0),
            (1200.0, 3500.0),
            (1500.0, 3000.0),
            (1656.2, 2656.2),
            (1800.0, 3200.0),
            (2000.0, 4000.0),
        ],
    )
    parser.add_argument("--max-windows", type=int, default=40)
    parser.add_argument("--examples", type=int, default=2)
    args = parser.parse_args()
    for item in args.files:
        evaluate_file(Path(item), args)


if __name__ == "__main__":
    main()
