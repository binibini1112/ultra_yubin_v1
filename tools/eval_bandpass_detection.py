#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import time
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, resample_poly, sosfilt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from jetson.src.audio_fallback import TelloAudioFallback


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
    return np.clip(audio, -1.0, 1.0)


def parse_band(text):
    if str(text).lower() == "raw":
        return ("raw", 0.0, 0.0)
    low, high = str(text).split("-", 1)
    low_f = float(low)
    high_f = float(high)
    return (f"{low_f:.0f}-{high_f:.0f}", low_f, high_f)


def parse_channels(text, channels):
    values = [int(part.strip()) for part in str(text or "").split(",") if part.strip()]
    if len(values) != 4:
        values = [0, 1, 2, 3] if channels == 4 else [1, 2, 3, 4]
    if channels == 4 and max(values) >= 4 and min(values) >= 1:
        values = [v - 1 for v in values]
    if min(values) < 0 or max(values) >= channels:
        raise ValueError(f"mic channels {values} exceed wav channels={channels}")
    return tuple(values)


def make_sos(low, high, sr, order):
    if low <= 0 and high <= 0:
        return None
    nyq = 0.5 * float(sr)
    low = max(1.0, min(float(low), nyq - 2.0))
    high = max(low + 1.0, min(float(high), nyq - 1.0))
    return butter(max(1, int(order)), [low / nyq, high / nyq], btype="band", output="sos")


def preprocess_mic(mic, sos, gain_mode, gain_target, noise_floor):
    if sos is None:
        return mic.astype(np.float32)
    filtered = sosfilt(sos, mic, axis=0).astype(np.float32)
    if gain_mode != "autogain":
        return filtered
    max_amp = float(np.max(np.abs(filtered))) if filtered.size else 0.0
    if max_amp <= float(noise_floor):
        return np.zeros_like(filtered, dtype=np.float32)
    return np.clip((filtered / max(max_amp, 1e-9)) * float(gain_target), -1.0, 1.0).astype(np.float32)


def fast_audio_to_logmel(model, audio):
    y = np.asarray(audio, dtype=np.float32)
    if y.size < model.clip_samples:
        y = np.pad(y, (0, model.clip_samples - y.size))
    elif y.size > model.clip_samples:
        y = y[-model.clip_samples:]
    y = np.pad(y, (model.n_fft // 2, model.n_fft // 2), mode="constant")
    n_frames = 1 + (len(y) - model.n_fft) // model.hop_length
    shape = (n_frames, model.n_fft)
    strides = (y.strides[0] * model.hop_length, y.strides[0])
    frames = np.lib.stride_tricks.as_strided(y, shape=shape, strides=strides)
    windowed = frames * model._hann[np.newaxis, :]
    power = (np.abs(np.fft.rfft(windowed, n=model.n_fft, axis=1)) ** 2).astype(np.float32).T
    mel = np.maximum(np.dot(model._mel_basis, power), 1e-10)
    ref = float(np.max(mel))
    logmel = 10.0 * np.log10(mel) - 10.0 * np.log10(max(ref, 1e-10))
    if logmel.shape[1] < model.target_frames:
        logmel = np.pad(logmel, ((0, 0), (0, model.target_frames - logmel.shape[1])), mode="edge")
    elif logmel.shape[1] > model.target_frames:
        logmel = logmel[:, : model.target_frames]
    logmel = (logmel - np.mean(logmel)) / (np.std(logmel) + 1e-6)
    return logmel.astype(np.float32)[..., np.newaxis]


def predict_probs_fast(model, mono):
    feat = fast_audio_to_logmel(model, mono)
    arr = feat[np.newaxis, ...].astype(np.float32)
    model._interpreter.set_tensor(model._input["index"], arr)
    model._interpreter.invoke()
    out = model._interpreter.get_tensor(model._output["index"])
    probs = np.ravel(out).astype(np.float32)
    return float(probs[0]), float(probs[1])


def window_detect(model, scores, candidates, window_size, consecutive, min_avg):
    if len(candidates) < window_size:
        return False
    return int(sum(candidates)) >= consecutive and float(np.mean(scores)) >= min_avg


def eval_rows(args):
    os.environ.setdefault("TELLO_AUDIO_DIRECTION_MODE", "legacy")
    os.environ.setdefault("TELLO_AUDIO_PREPROCESS", "0")
    model = TelloAudioFallback(
        args.model,
        args.config,
        alsa_device="offline",
        channels=6,
        threshold=args.threshold,
        min_avg_score=args.min_avg,
        consecutive=args.consecutive,
        min_rms=args.min_rms,
        doa_method="gcc",
        verbose=False,
    )

    dataset = Path(args.dataset)
    metadata_path = dataset / "metadata.csv"
    bands = [parse_band(item) for item in args.bands]
    gains = [float(item) for item in args.gains]
    sos_cache = {
        label: make_sos(low, high, model.sample_rate, args.order)
        for label, low, high in bands
    }
    modes = ["raw"] + [f"{label}:band" for label, _, _ in bands if label != "raw"]
    modes += [f"{label}:autogain" for label, _, _ in bands if label != "raw"]

    with metadata_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        meta_rows = list(reader)
    if args.source_types:
        allowed = set(args.source_types)
        meta_rows = [row for row in meta_rows if row.get("source_type", "") in allowed]
    if args.distances:
        allowed_dist = set(str(float(item)) for item in args.distances)
        meta_rows = [row for row in meta_rows if str(float(row.get("distance_m", "0"))) in allowed_dist]

    total_files = len(meta_rows)
    print(f"[eval] files={total_files} dataset={dataset}")
    print(f"[eval] bands={', '.join(label for label, _, _ in bands)} gains={gains}")
    results = []
    for file_idx, row in enumerate(meta_rows, 1):
        wav_path = dataset / row["file_path"]
        if not wav_path.exists():
            print(f"[warn] missing: {wav_path}")
            continue
        audio = read_audio(wav_path, model.sample_rate)
        if audio.ndim != 2 or audio.shape[1] < 4:
            print(f"[warn] skip non-4ch: {wav_path} shape={audio.shape}")
            continue
        selected = parse_channels(row.get("mic_channels_used", ""), audio.shape[1])
        clip = int(model.clip_samples)
        stride = max(1, int(round(model.sample_rate * args.stride_sec)))
        if audio.shape[0] < clip:
            audio = np.pad(audio, ((0, clip - audio.shape[0]), (0, 0)))
        starts = list(range(0, audio.shape[0] - clip + 1, stride))
        if args.max_windows > 0:
            starts = starts[: args.max_windows]
        if file_idx == 1 or file_idx % 6 == 0:
            print(f"[eval] {file_idx:02d}/{total_files} {row['source_type']} {row['angle_deg']}deg {row['distance_m']}m windows={len(starts)}")

        for gain in gains:
            states = {}
            for mode in modes:
                states[mode] = {
                    "scores": deque(maxlen=args.window_size),
                    "cands": deque(maxlen=args.window_size),
                    "frames": 0,
                    "candidate_frames": 0,
                    "detected_frames": 0,
                    "score_sum": 0.0,
                    "score_max": 0.0,
                    "rms_sum": 0.0,
                    "rms_max": 0.0,
                }
            for start in starts:
                frame = np.clip(audio[start : start + clip, :] * gain, -1.0, 1.0)
                mic = frame[:, selected]
                prepared = {"raw": mic}
                for label, _, _ in bands:
                    if label == "raw":
                        continue
                    sos = sos_cache[label]
                    prepared[f"{label}:band"] = preprocess_mic(mic, sos, "band", args.gain_target, args.noise_floor)
                    prepared[f"{label}:autogain"] = preprocess_mic(mic, sos, "autogain", args.gain_target, args.noise_floor)

                for mode, mic_in in prepared.items():
                    mono = np.clip(mic_in.mean(axis=1).astype(np.float32), -1.0, 1.0)
                    rms = float(np.sqrt(np.mean(mono * mono)))
                    _, score = predict_probs_fast(model, mono)
                    candidate = bool(score >= args.threshold and rms >= args.min_rms)
                    state = states[mode]
                    state["frames"] += 1
                    state["candidate_frames"] += int(candidate)
                    state["score_sum"] += score
                    state["score_max"] = max(state["score_max"], float(score))
                    state["rms_sum"] += rms
                    state["rms_max"] = max(state["rms_max"], rms)
                    state["scores"].append(float(score))
                    state["cands"].append(candidate)
                    state["detected_frames"] += int(
                        window_detect(
                            model,
                            state["scores"],
                            state["cands"],
                            args.window_size,
                            args.consecutive,
                            args.min_avg,
                        )
                    )

            for mode, state in states.items():
                frames = max(1, int(state["frames"]))
                results.append(
                    {
                        "file_path": row["file_path"],
                        "angle_deg": row.get("angle_deg", ""),
                        "distance_m": row.get("distance_m", ""),
                        "source_type": row.get("source_type", ""),
                        "positive": "0" if row.get("source_type", "") == "background" else "1",
                        "gain": gain,
                        "mode": mode,
                        "frames": state["frames"],
                        "candidate_frames": state["candidate_frames"],
                        "detected_frames": state["detected_frames"],
                        "candidate_rate": state["candidate_frames"] / frames,
                        "detected_rate": state["detected_frames"] / frames,
                        "score_avg": state["score_sum"] / frames,
                        "score_max": state["score_max"],
                        "rms_avg": state["rms_sum"] / frames,
                        "rms_max": state["rms_max"],
                    }
                )
    return results


def summarize(results):
    grouped = defaultdict(list)
    for row in results:
        key = (row["mode"], row["gain"], row["source_type"], row["distance_m"])
        grouped[key].append(row)

    summary = []
    for (mode, gain, source_type, distance_m), rows in grouped.items():
        frames = sum(int(r["frames"]) for r in rows)
        cand = sum(int(r["candidate_frames"]) for r in rows)
        det = sum(int(r["detected_frames"]) for r in rows)
        summary.append(
            {
                "mode": mode,
                "gain": gain,
                "source_type": source_type,
                "distance_m": distance_m,
                "files": len(rows),
                "frames": frames,
                "candidate_rate": cand / max(1, frames),
                "detected_rate": det / max(1, frames),
                "score_avg": float(np.mean([float(r["score_avg"]) for r in rows])),
                "score_max": max(float(r["score_max"]) for r in rows),
                "rms_avg": float(np.mean([float(r["rms_avg"]) for r in rows])),
            }
        )
    return summary


def print_top(summary, gain, source_type, distance_m=None, limit=12):
    rows = [
        r for r in summary
        if abs(float(r["gain"]) - float(gain)) < 1e-9
        and r["source_type"] == source_type
        and (distance_m is None or str(r["distance_m"]) == str(distance_m))
    ]
    rows.sort(key=lambda r: (r["detected_rate"], r["candidate_rate"], r["score_avg"]), reverse=True)
    title_dist = "all" if distance_m is None else f"{distance_m}m"
    print(f"\n[summary] source={source_type} distance={title_dist} gain={gain}")
    for r in rows[:limit]:
        print(
            f"  {r['mode']:<21} files={r['files']:2d} det={r['detected_rate']:.3f} "
            f"cand={r['candidate_rate']:.3f} score={r['score_avg']:.3f} rms={r['rms_avg']:.4f}"
        )


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="/home/jetson/jy2/dataset_audio_angle")
    parser.add_argument("--model", default=str(ROOT / "model" / "tello_detector_cnn_retrained_jetson.tflite"))
    parser.add_argument("--config", default=str(ROOT / "model" / "config.json"))
    parser.add_argument("--threshold", type=float, default=float(os.getenv("TELLO_AUDIO_THRESHOLD", "0.50")))
    parser.add_argument("--min-avg", type=float, default=float(os.getenv("TELLO_AUDIO_MIN_AVG_SCORE", "0.55")))
    parser.add_argument("--min-rms", type=float, default=float(os.getenv("TELLO_AUDIO_MIN_RMS", "0.003")))
    parser.add_argument("--window-size", type=int, default=int(os.getenv("TELLO_AUDIO_WINDOW_SIZE", "3")))
    parser.add_argument("--consecutive", type=int, default=int(os.getenv("TELLO_AUDIO_CONSECUTIVE", "2")))
    parser.add_argument("--stride-sec", type=float, default=0.25)
    parser.add_argument("--order", type=int, default=5)
    parser.add_argument("--noise-floor", type=float, default=0.003)
    parser.add_argument("--gain-target", type=float, default=0.90)
    parser.add_argument("--max-windows", type=int, default=120)
    parser.add_argument("--source-types", nargs="*", default=[])
    parser.add_argument("--distances", nargs="*", default=[])
    parser.add_argument("--gains", nargs="+", default=["1.0", "0.5", "0.25", "0.125"])
    parser.add_argument(
        "--bands",
        nargs="+",
        default=["raw", "1000-4000", "1200-3500", "1400-3000", "1656.2-2656.2", "1800-3200"],
    )
    args = parser.parse_args()
    args.gains = [float(x) for x in args.gains]

    started = time.strftime("%Y%m%d_%H%M%S")
    rows = eval_rows(args)
    summary = summarize(rows)
    detail_path = ROOT / "benchmark_logs" / f"bandpass_detection_detail_{started}.csv"
    summary_path = ROOT / "benchmark_logs" / f"bandpass_detection_summary_{started}.csv"
    write_csv(detail_path, rows)
    write_csv(summary_path, summary)

    print_top(summary, 1.0, "real_drone", "3.0")
    print_top(summary, 0.5, "real_drone", "3.0")
    print_top(summary, 0.25, "real_drone", "3.0")
    print_top(summary, 1.0, "background")
    print(f"\n[output] detail={detail_path}")
    print(f"[output] summary={summary_path}")


if __name__ == "__main__":
    raise SystemExit(main())
