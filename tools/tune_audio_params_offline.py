#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import sys
import time
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from jetson.src.audio_fallback import TelloAudioFallback
from tools.eval_bandpass_detection import predict_probs_fast


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


def parse_channels(text, channels):
    values = [int(part.strip()) for part in str(text or "").split(",") if part.strip()]
    if len(values) != 4:
        values = [0, 1, 2, 3] if channels == 4 else [1, 2, 3, 4]
    if channels == 4 and max(values) >= 4 and min(values) >= 1:
        values = [v - 1 for v in values]
    if min(values) < 0 or max(values) >= channels:
        raise ValueError(f"mic channels {values} exceed wav channels={channels}")
    return tuple(values)


def circular_diff_deg(a, b):
    return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)


def make_detector(args):
    os.environ.setdefault("TELLO_AUDIO_PREPROCESS", "0")
    os.environ["TELLO_AUDIO_DIRECTION_MODE"] = "legacy"
    return TelloAudioFallback(
        args.detector_model,
        args.detector_config,
        alsa_device="offline",
        channels=6,
        threshold=0.5,
        min_avg_score=0.55,
        consecutive=2,
        min_rms=0.003,
        doa_method="gcc",
        verbose=False,
    )


def make_direction_model(name, model_path, config_path):
    old = {key: os.environ.get(key) for key in (
        "TELLO_AUDIO_DIRECTION_MODE",
        "TELLO_AUDIO_DIRECTION_MODEL",
        "TELLO_AUDIO_DIRECTION_FEATURE_CONFIG",
        "TELLO_AUDIO_DIRECTION_LABEL_MAPPING",
    )}
    os.environ["TELLO_AUDIO_DIRECTION_MODE"] = "cnn6"
    os.environ["TELLO_AUDIO_DIRECTION_MODEL"] = str(model_path)
    os.environ["TELLO_AUDIO_DIRECTION_FEATURE_CONFIG"] = str(config_path)
    os.environ.pop("TELLO_AUDIO_DIRECTION_LABEL_MAPPING", None)
    label_path = Path(config_path).with_name("label_mapping.json")
    if label_path.exists():
        os.environ["TELLO_AUDIO_DIRECTION_LABEL_MAPPING"] = str(label_path)
    model = TelloAudioFallback(
        str(ROOT / "model" / "tello_detector_cnn_retrained_jetson.tflite"),
        str(ROOT / "model" / "config.json"),
        alsa_device="offline",
        channels=6,
        threshold=0.5,
        min_avg_score=0.55,
        consecutive=2,
        min_rms=0.003,
        doa_method="gcc",
        verbose=False,
    )
    for key, value in old.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value
    return name, model


def load_metadata(dataset):
    with (dataset / "metadata.csv").open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def precompute_windows(args):
    dataset = Path(args.dataset)
    detector = make_detector(args)
    direction_models = []
    if args.direction in ("both", "junyoung"):
        direction_models.append(
            make_direction_model(
                "junyoung",
                ROOT / "models" / "audio_direction" / "junyoung_cnn6" / "audio_angle_cnn_final.tflite",
                ROOT / "models" / "audio_direction" / "junyoung_cnn6" / "feature_config.json",
            )
        )
    if args.direction in ("both", "jh"):
        direction_models.append(
            make_direction_model(
                "jh",
                ROOT / "jhmodle" / "audio_angle_4ch_1dcnn_float32.tflite",
                ROOT / "jhmodle" / "audio_angle_4ch_1dcnn_deploy_config.json",
            )
        )

    rows = load_metadata(dataset)
    if args.source_types:
        allowed = set(args.source_types)
        rows = [row for row in rows if row.get("source_type", "") in allowed]
    clip = int(detector.clip_samples)
    stride = max(1, int(round(detector.sample_rate * args.stride_sec)))
    records = []
    print(f"[tune] files={len(rows)} direction_models={[name for name, _ in direction_models]}")
    for file_idx, row in enumerate(rows, 1):
        wav_path = dataset / row["file_path"]
        if not wav_path.exists():
            print(f"[warn] missing {wav_path}")
            continue
        audio = read_audio(wav_path, detector.sample_rate)
        if audio.shape[1] < 4:
            print(f"[warn] skip non-4ch {wav_path} shape={audio.shape}")
            continue
        selected = parse_channels(row.get("mic_channels_used", ""), audio.shape[1])
        if audio.shape[0] < clip:
            audio = np.pad(audio, ((0, clip - audio.shape[0]), (0, 0)))
        starts = list(range(0, audio.shape[0] - clip + 1, stride))
        if args.max_windows > 0:
            starts = starts[: args.max_windows]
        print(
            f"[tune] {file_idx:02d}/{len(rows)} {row['source_type']} "
            f"{row['angle_deg']}deg {row['distance_m']}m windows={len(starts)}"
        )
        file_key = row["file_path"]
        for win_idx, start in enumerate(starts):
            frame = audio[start : start + clip, :]
            mic = frame[:, selected]
            mono = np.clip(mic.mean(axis=1).astype(np.float32), -1.0, 1.0)
            rms = float(np.sqrt(np.mean(mono * mono)))
            _, score = predict_probs_fast(detector, mono)
            directions = {}
            for model_name, model in direction_models:
                try:
                    pred = model._predict_learned_direction(mic, mic)
                except Exception as exc:
                    pred = {"angle": None, "confidence": 0.0, "margin": 0.0, "error": str(exc)}
                directions[model_name] = pred
            records.append(
                {
                    "file_key": file_key,
                    "window_index": win_idx,
                    "source_type": row.get("source_type", ""),
                    "positive": row.get("source_type", "") != "background",
                    "angle_deg": float(row.get("angle_deg", 0.0)),
                    "distance_m": float(row.get("distance_m", 0.0)),
                    "score": float(score),
                    "rms": rms,
                    "directions": directions,
                }
            )
    return records


def parse_float_list(text):
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def evaluate_combo(records, model_name, threshold, min_rms, min_avg, cnn_conf, cnn_margin, args):
    by_file = defaultdict(list)
    for rec in records:
        by_file[rec["file_key"]].append(rec)

    metrics = {
        "total_pos": 0,
        "real_pos": 0,
        "phone_pos": 0,
        "bg_total": 0,
        "pos_detect": 0,
        "real_detect": 0,
        "phone_detect": 0,
        "bg_detect": 0,
        "pos_accept": 0,
        "real_accept": 0,
        "phone_accept": 0,
        "bg_accept": 0,
        "dir_checked": 0,
        "dir_correct": 0,
        "dir_real_checked": 0,
        "dir_real_correct": 0,
    }
    for _, file_recs in by_file.items():
        file_recs.sort(key=lambda item: item["window_index"])
        candidates = deque(maxlen=args.window_size)
        scores = deque(maxlen=args.window_size)
        for rec in file_recs:
            positive = bool(rec["positive"])
            source = rec["source_type"]
            if positive:
                metrics["total_pos"] += 1
                metrics["real_pos" if source == "real_drone" else "phone_pos"] += 1
            else:
                metrics["bg_total"] += 1
            candidate = bool(rec["score"] >= threshold and rec["rms"] >= min_rms)
            candidates.append(candidate)
            scores.append(rec["score"])
            detected = (
                len(candidates) >= args.window_size
                and int(sum(candidates)) >= args.consecutive
                and float(np.mean(scores)) >= min_avg
            )
            if detected:
                if positive:
                    metrics["pos_detect"] += 1
                    metrics["real_detect" if source == "real_drone" else "phone_detect"] += 1
                else:
                    metrics["bg_detect"] += 1

            pred = rec["directions"].get(model_name, {})
            angle = pred.get("angle")
            conf = float(pred.get("confidence") or 0.0)
            margin = float(pred.get("margin") or 0.0)
            accepted = bool(detected and angle is not None and conf >= cnn_conf and margin >= cnn_margin)
            if accepted:
                if positive:
                    metrics["pos_accept"] += 1
                    metrics["real_accept" if source == "real_drone" else "phone_accept"] += 1
                    metrics["dir_checked"] += 1
                    correct = circular_diff_deg(angle, rec["angle_deg"]) <= args.correct_tolerance_deg
                    metrics["dir_correct"] += int(correct)
                    if source == "real_drone":
                        metrics["dir_real_checked"] += 1
                        metrics["dir_real_correct"] += int(correct)
                else:
                    metrics["bg_accept"] += 1

    total_pos = max(1, metrics["total_pos"])
    real_pos = max(1, metrics["real_pos"])
    phone_pos = max(1, metrics["phone_pos"])
    bg_total = max(1, metrics["bg_total"])
    dir_checked = max(1, metrics["dir_checked"])
    dir_real_checked = max(1, metrics["dir_real_checked"])
    real_accept_rate = metrics["real_accept"] / real_pos
    bg_accept_rate = metrics["bg_accept"] / bg_total
    dir_acc = metrics["dir_correct"] / dir_checked if metrics["dir_checked"] else 0.0
    real_dir_acc = metrics["dir_real_correct"] / dir_real_checked if metrics["dir_real_checked"] else 0.0
    objective = (
        2.0 * real_accept_rate
        + 0.8 * real_dir_acc
        + 0.5 * (metrics["phone_accept"] / phone_pos)
        - 4.0 * bg_accept_rate
    )
    return {
        "model": model_name,
        "threshold": threshold,
        "min_rms": min_rms,
        "min_avg": min_avg,
        "cnn_conf": cnn_conf,
        "cnn_margin": cnn_margin,
        "objective": objective,
        "real_detect_rate": metrics["real_detect"] / real_pos,
        "real_accept_rate": real_accept_rate,
        "phone_accept_rate": metrics["phone_accept"] / phone_pos,
        "background_detect_rate": metrics["bg_detect"] / bg_total,
        "background_accept_rate": bg_accept_rate,
        "direction_acc": dir_acc,
        "real_direction_acc": real_dir_acc,
        "dir_checked": metrics["dir_checked"],
        "real_windows": metrics["real_pos"],
        "background_windows": metrics["bg_total"],
    }


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
    parser.add_argument("--detector-model", default=str(ROOT / "model" / "tello_detector_cnn_retrained_jetson.tflite"))
    parser.add_argument("--detector-config", default=str(ROOT / "model" / "config.json"))
    parser.add_argument("--direction", choices=["both", "junyoung", "jh"], default="both")
    parser.add_argument("--source-types", nargs="*", default=["real_drone", "phone_speaker", "background"])
    parser.add_argument("--max-windows", type=int, default=30)
    parser.add_argument("--stride-sec", type=float, default=0.25)
    parser.add_argument("--window-size", type=int, default=3)
    parser.add_argument("--consecutive", type=int, default=2)
    parser.add_argument("--correct-tolerance-deg", type=float, default=30.0)
    parser.add_argument("--thresholds", default="0.45,0.50,0.55,0.60,0.70")
    parser.add_argument("--min-rms-values", default="0.0015,0.002,0.0025,0.003")
    parser.add_argument("--min-avg-values", default="0.45,0.50,0.55,0.60")
    parser.add_argument("--cnn-conf-values", default="0.0,0.45,0.60,0.75")
    parser.add_argument("--cnn-margin-values", default="0.0,0.10,0.20,0.35")
    parser.add_argument("--top", type=int, default=20)
    args = parser.parse_args()

    records = precompute_windows(args)
    thresholds = parse_float_list(args.thresholds)
    min_rms_values = parse_float_list(args.min_rms_values)
    min_avg_values = parse_float_list(args.min_avg_values)
    cnn_conf_values = parse_float_list(args.cnn_conf_values)
    cnn_margin_values = parse_float_list(args.cnn_margin_values)
    model_names = sorted(records[0]["directions"].keys()) if records else []

    rows = []
    for model_name in model_names:
        for threshold in thresholds:
            for min_rms in min_rms_values:
                for min_avg in min_avg_values:
                    for cnn_conf in cnn_conf_values:
                        for cnn_margin in cnn_margin_values:
                            rows.append(
                                evaluate_combo(
                                    records,
                                    model_name,
                                    threshold,
                                    min_rms,
                                    min_avg,
                                    cnn_conf,
                                    cnn_margin,
                                    args,
                                )
                            )
    rows.sort(
        key=lambda row: (
            row["objective"],
            row["real_accept_rate"],
            row["real_direction_acc"],
            -row["background_accept_rate"],
        ),
        reverse=True,
    )
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = ROOT / "benchmark_logs" / f"audio_param_tuning_{stamp}.csv"
    json_path = ROOT / "benchmark_logs" / f"audio_param_tuning_best_{stamp}.json"
    write_csv(csv_path, rows)
    best = rows[: args.top]
    json_path.write_text(json.dumps(best, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"\n[tune] windows={len(records)} combos={len(rows)}")
    print(f"[tune] output_csv={csv_path}")
    print(f"[tune] output_best={json_path}")
    print("\n[tune] TOP")
    for idx, row in enumerate(best, 1):
        print(
            f"{idx:02d}. model={row['model']:<8} obj={row['objective']:.3f} "
            f"thr={row['threshold']:.2f} rms={row['min_rms']:.4f} avg={row['min_avg']:.2f} "
            f"conf={row['cnn_conf']:.2f} margin={row['cnn_margin']:.2f} "
            f"real_accept={row['real_accept_rate']:.3f} real_dir={row['real_direction_acc']:.3f} "
            f"bg_accept={row['background_accept_rate']:.3f} phone_accept={row['phone_accept_rate']:.3f}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
