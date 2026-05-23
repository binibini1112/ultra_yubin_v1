#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, resample_poly, sosfilt


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = Path("/home/jetson/jy2/dataset_audio_angle")
DEFAULT_OUT = ROOT / "models" / "audio_direction" / "direction_6sector_lr.npz"


def read_metadata(dataset_dir):
    meta_path = dataset_dir / "metadata.jsonl"
    rows = []
    with meta_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            item = json.loads(line)
            wav_path = dataset_dir / item["file_path"]
            if wav_path.exists():
                item["abs_path"] = str(wav_path)
                rows.append(item)
    if not rows:
        raise RuntimeError(f"no wav rows found from {meta_path}")
    return rows


def read_audio(path, target_sr):
    sr, data = wavfile.read(str(path))
    data = np.asarray(data)
    if data.ndim == 1:
        data = data[:, None]
    if np.issubdtype(data.dtype, np.integer):
        audio = data.astype(np.float32) / float(np.iinfo(data.dtype).max)
    else:
        audio = data.astype(np.float32)
    if sr != target_sr:
        gcd = math.gcd(int(sr), int(target_sr))
        audio = resample_poly(audio, int(target_sr) // gcd, int(sr) // gcd, axis=0).astype(np.float32)
        sr = target_sr
    return np.clip(audio, -1.0, 1.0), sr


def gcc_pair_features(a, b, sr, max_tau_sec=0.00025, interp=8):
    n = int(a.shape[0] + b.shape[0])
    nfft = 1 << int(np.ceil(np.log2(max(1, n))))
    fa = np.fft.rfft(a, n=nfft)
    fb = np.fft.rfft(b, n=nfft)
    cross = fa * np.conj(fb)
    cross /= np.maximum(np.abs(cross), 1e-12)
    cc = np.fft.irfft(cross, n=nfft * interp)
    max_shift = min(int(interp * sr * max_tau_sec), cc.shape[0] // 2)
    if max_shift <= 0:
        return 0.0, 0.0, 0.0
    cc = np.concatenate((cc[-max_shift:], cc[: max_shift + 1]))
    abs_cc = np.abs(cc)
    peak_idx = int(np.argmax(abs_cc))
    shift = peak_idx - max_shift
    tau = float(shift) / float(interp * sr)
    peak = float(abs_cc[peak_idx])
    mean = float(np.mean(abs_cc) + 1e-9)
    return tau / max_tau_sec, peak, peak / mean


def circular_gcc_angle(mic_4ch, sr, mic_distance, sound_speed=340.0):
    max_tau = float(mic_distance) / float(sound_speed)
    tx, px, rx = gcc_pair_features(mic_4ch[:, 0], mic_4ch[:, 2], sr, max_tau)
    ty, py, ry = gcc_pair_features(mic_4ch[:, 1], mic_4ch[:, 3], sr, max_tau)
    angle = math.degrees(math.atan2(ty, tx)) % 360.0
    return angle, (tx, ty, px, py, rx, ry)


def bandpass(audio, sr, low, high, order):
    nyq = 0.5 * sr
    sos = butter(order, [low / nyq, high / nyq], btype="band", output="sos")
    return sosfilt(sos, audio, axis=0).astype(np.float32)


def feature_from_window(window, sr, args):
    mic = window[:, :4].astype(np.float32)
    raw_rms = np.sqrt(np.mean(mic * mic, axis=0) + 1e-12)
    filtered = bandpass(mic, sr, args.band_low, args.band_high, args.band_order)
    max_amp = float(np.max(np.abs(filtered)))
    if max_amp >= args.noise_floor:
        filtered = (filtered / max_amp) * args.gain_target
    else:
        filtered = np.zeros_like(filtered)
    rms = np.sqrt(np.mean(filtered * filtered, axis=0) + 1e-12)
    mono = filtered.mean(axis=1)
    mono_rms = float(np.sqrt(np.mean(mono * mono) + 1e-12))

    feats = []
    feats.extend(np.log(raw_rms + 1e-6).tolist())
    feats.extend(np.log(rms + 1e-6).tolist())
    for i in range(4):
        for j in range(i + 1, 4):
            feats.append(float(np.log((rms[i] + 1e-6) / (rms[j] + 1e-6))))

    angle, angle_feats = circular_gcc_angle(filtered, sr, args.mic_distance)
    feats.extend(angle_feats)
    feats.extend([math.sin(math.radians(angle)), math.cos(math.radians(angle))])

    for i in range(4):
        x = filtered[:, i]
        zcr = np.mean(np.abs(np.diff(np.signbit(x).astype(np.float32))))
        feats.append(float(zcr))
    feats.append(mono_rms)
    return np.asarray(feats, dtype=np.float32)


def build_windows(rows, args):
    xs = []
    ys = []
    groups = []
    paths = []
    clip = int(round(args.sample_rate * args.clip_sec))
    stride = int(round(args.sample_rate * args.stride_sec))
    for row in rows:
        audio, sr = read_audio(row["abs_path"], args.sample_rate)
        selected = audio[:, :4]
        if selected.shape[0] < clip:
            continue
        label = int(row["angle_class"])
        distance = float(row.get("distance_m", 0.0))
        for start in range(0, selected.shape[0] - clip + 1, stride):
            xs.append(feature_from_window(selected[start : start + clip], sr, args))
            ys.append(label)
            groups.append(distance)
            paths.append(row["file_path"])
    if not xs:
        raise RuntimeError("no training windows created")
    return np.vstack(xs).astype(np.float32), np.asarray(ys, dtype=np.int64), np.asarray(groups, dtype=np.float32), paths


def standardize(train_x, *arrays):
    mean = train_x.mean(axis=0)
    std = train_x.std(axis=0) + 1e-6
    return (mean, std), tuple((arr - mean) / std for arr in arrays)


def softmax(logits):
    logits = logits - logits.max(axis=1, keepdims=True)
    exp = np.exp(logits)
    return exp / np.maximum(exp.sum(axis=1, keepdims=True), 1e-12)


def train_lr(x, y, num_classes, args):
    rng = np.random.default_rng(args.seed)
    w = rng.normal(0.0, 0.01, size=(x.shape[1], num_classes)).astype(np.float32)
    b = np.zeros((num_classes,), dtype=np.float32)
    onehot = np.eye(num_classes, dtype=np.float32)[y]
    n = x.shape[0]
    for epoch in range(args.epochs):
        order = rng.permutation(n)
        for start in range(0, n, args.batch_size):
            idx = order[start : start + args.batch_size]
            xb = x[idx]
            yb = onehot[idx]
            probs = softmax(xb @ w + b)
            grad = (probs - yb) / max(1, xb.shape[0])
            gw = xb.T @ grad + args.l2 * w
            gb = grad.sum(axis=0)
            w -= args.lr * gw
            b -= args.lr * gb
        if args.verbose and (epoch == 0 or (epoch + 1) % 50 == 0):
            pred = np.argmax(softmax(x @ w + b), axis=1)
            acc = float(np.mean(pred == y))
            print(f"epoch={epoch + 1} train_acc={acc:.3f}")
    return w, b


def evaluate(x, y, w, b, num_classes):
    probs = softmax(x @ w + b)
    pred = np.argmax(probs, axis=1)
    conf = np.max(probs, axis=1)
    acc = float(np.mean(pred == y)) if y.size else 0.0
    confusion = np.zeros((num_classes, num_classes), dtype=np.int64)
    for t, p in zip(y, pred):
        confusion[int(t), int(p)] += 1
    return {
        "accuracy": acc,
        "confidence_mean": float(np.mean(conf)) if conf.size else 0.0,
        "pred": pred,
        "confusion": confusion,
    }


def summarize_split(name, result, angles):
    print(f"{name}: acc={result['accuracy']:.3f} conf={result['confidence_mean']:.3f}")
    print("  confusion rows=true cols=pred")
    header = "      " + " ".join(f"{a:>5}" for a in angles)
    print(header)
    for angle, row in zip(angles, result["confusion"]):
        print(f"  {angle:>3} " + " ".join(f"{int(v):5d}" for v in row))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--clip-sec", type=float, default=1.0)
    parser.add_argument("--stride-sec", type=float, default=0.25)
    parser.add_argument("--holdout-distance", type=float, default=3.0)
    parser.add_argument("--mic-distance", type=float, default=0.045)
    parser.add_argument("--band-low", type=float, default=1656.2)
    parser.add_argument("--band-high", type=float, default=2656.2)
    parser.add_argument("--band-order", type=int, default=5)
    parser.add_argument("--noise-floor", type=float, default=0.003)
    parser.add_argument("--gain-target", type=float, default=0.9)
    parser.add_argument("--epochs", type=int, default=250)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=0.05)
    parser.add_argument("--l2", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    rows = read_metadata(args.dataset)
    angles = sorted({int(row["angle_deg"]) for row in rows})
    class_to_angle = {int(row["angle_class"]): int(row["angle_deg"]) for row in rows}
    angle_labels = [class_to_angle[i] for i in sorted(class_to_angle)]

    x, y, groups, _paths = build_windows(rows, args)
    holdout = np.isclose(groups, float(args.holdout_distance))
    if not np.any(holdout):
        raise RuntimeError(f"holdout distance {args.holdout_distance}m not present")
    train_mask = ~holdout
    val_mask = holdout
    (mean, std), (x_train, x_val, x_all) = standardize(x[train_mask], x[train_mask], x[val_mask], x)

    num_classes = len(angle_labels)
    w, b = train_lr(x_train, y[train_mask], num_classes, args)
    train_result = evaluate(x_train, y[train_mask], w, b, num_classes)
    val_result = evaluate(x_val, y[val_mask], w, b, num_classes)

    summarize_split("train", train_result, angle_labels)
    summarize_split(f"holdout_{args.holdout_distance:g}m", val_result, angle_labels)

    # Retrain on all windows for deployment after reporting the distance holdout score.
    (mean_all, std_all), (x_all_std,) = standardize(x, x)
    w_all, b_all = train_lr(x_all_std, y, num_classes, args)
    all_result = evaluate(x_all_std, y, w_all, b_all, num_classes)
    summarize_split("all_fit", all_result, angle_labels)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.out,
        weights=w_all.astype(np.float32),
        bias=b_all.astype(np.float32),
        mean=mean_all.astype(np.float32),
        std=std_all.astype(np.float32),
        angle_labels=np.asarray(angle_labels, dtype=np.int64),
        feature_dim=np.asarray([x.shape[1]], dtype=np.int64),
        sample_rate=np.asarray([args.sample_rate], dtype=np.int64),
        clip_sec=np.asarray([args.clip_sec], dtype=np.float32),
        band_low=np.asarray([args.band_low], dtype=np.float32),
        band_high=np.asarray([args.band_high], dtype=np.float32),
        band_order=np.asarray([args.band_order], dtype=np.int64),
        mic_distance=np.asarray([args.mic_distance], dtype=np.float32),
    )
    report = {
        "dataset": str(args.dataset),
        "rows": len(rows),
        "windows": int(x.shape[0]),
        "angles": angles,
        "holdout_distance_m": args.holdout_distance,
        "train_accuracy": train_result["accuracy"],
        "holdout_accuracy": val_result["accuracy"],
        "all_fit_accuracy": all_result["accuracy"],
        "model": str(args.out),
    }
    report_path = args.out.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"saved_model={args.out}")
    print(f"saved_report={report_path}")


if __name__ == "__main__":
    main()
