#!/usr/bin/env python3
import argparse
import math
import os
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly

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
    if sr != target_sr:
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
        raise ValueError(f"need at least 4 channels for beam scan, got {channels}")
    if len(selected) != 4:
        raise ValueError(f"need exactly 4 mic channels, got {selected}")
    if max(selected) >= channels:
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


def circular_mean_deg(values):
    if not values:
        return None
    unit = np.exp(1j * np.deg2rad(values))
    return float(np.rad2deg(np.angle(np.mean(unit))) % 360.0)


def fractional_delay(signal, delay_samples):
    x = np.arange(signal.shape[0], dtype=np.float32)
    # Positive delay means read a later sample to compensate a later arrival.
    return np.interp(x + float(delay_samples), x, signal, left=0.0, right=0.0).astype(np.float32)


def delay_and_sum(audio_4ch, angle_deg, sample_rate, mic_distance, sound_speed=340.0):
    theta = math.radians(float(angle_deg))
    direction = np.asarray([math.cos(theta), math.sin(theta)], dtype=np.float32)
    d = float(mic_distance)
    positions = np.asarray(
        [
            [d / 2.0, 0.0],
            [0.0, d / 2.0],
            [-d / 2.0, 0.0],
            [0.0, -d / 2.0],
        ],
        dtype=np.float32,
    )
    delays_sec = -(positions @ direction) / float(sound_speed)
    delays_sec -= float(np.mean(delays_sec))
    aligned = []
    for ch in range(4):
        aligned.append(fractional_delay(audio_4ch[:, ch], delays_sec[ch] * float(sample_rate)))
    beam = np.mean(np.stack(aligned, axis=1), axis=1).astype(np.float32)
    return np.clip(beam, -1.0, 1.0)


def top_scores(score_by_angle, top_n=4):
    return sorted(score_by_angle.items(), key=lambda item: item[1], reverse=True)[:top_n]


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
    sr, audio = read_audio(path, model.sample_rate)
    selected = parse_channels(args.mic_channels, audio.shape[1])
    clip = model.clip_samples
    stride = max(1, int(round(model.sample_rate * args.stride_sec)))
    if audio.shape[0] < clip:
        audio = np.pad(audio, ((0, clip - audio.shape[0]), (0, 0)))
    angles = [float(a) for a in args.angles]

    print(f"\nFILE {path}")
    print(f"  sr={sr} sec={audio.shape[0] / sr:.1f} channels={audio.shape[1]} mic={selected}")
    print(f"  beam_angles={','.join(str(int(a)) for a in angles)} threshold={args.threshold} min_rms={args.min_rms}")

    for gain in args.gains:
        scans = 0
        accepted = 0
        top_counter = Counter()
        section_counter = Counter()
        top_angles = []
        margins = []
        baseline_scores = []
        gcc_angles = []
        examples = []

        for idx, start in enumerate(range(0, audio.shape[0] - clip + 1, stride)):
            if args.max_windows and scans >= args.max_windows:
                break
            raw = np.clip(audio[start : start + clip, :] * float(gain), -1.0, 1.0)
            mic = raw[:, selected]
            mono = mic.mean(axis=1).astype(np.float32)
            rms = float(np.sqrt(np.mean(mono * mono)))
            _, base_score = model._predict_probs(mono)
            baseline_scores.append(base_score)
            if base_score < args.threshold or rms < args.min_rms:
                continue

            try:
                gcc_angle = float(
                    estimate_direction_gcc_phat(
                        model._preprocess_mic_audio(mic) if args.gcc_preprocess else mic,
                        fs=model.sample_rate,
                        mic_distance=model.mic_distance,
                        min_peak_ratio=model.gcc_min_peak_ratio,
                    )
                )
                gcc_angles.append(gcc_angle)
            except Exception:
                pass

            score_by_angle = {}
            power_by_angle = {}
            scan_input = model._preprocess_mic_audio(mic) if args.beam_preprocess else mic
            for angle in angles:
                beam = delay_and_sum(scan_input, angle, model.sample_rate, model.mic_distance)
                _, score = model._predict_probs(beam)
                score_by_angle[angle] = score
                power_by_angle[angle] = float(np.sqrt(np.mean(beam * beam)))
            ranked = top_scores(score_by_angle, top_n=4)
            ranked_power = top_scores(power_by_angle, top_n=4)
            top_angle, top_score = ranked[0]
            second_score = ranked[1][1] if len(ranked) > 1 else 0.0
            margin = float(top_score - second_score)
            scans += 1
            if top_score >= args.beam_threshold and margin >= args.margin:
                accepted += 1
                top_counter[int(round(top_angle)) % 360] += 1
                section_counter[section(top_angle)] += 1
                top_angles.append(top_angle)
                margins.append(margin)
            if len(examples) < args.examples:
                example = " ".join(f"{int(a):03d}:{s:.3f}" for a, s in ranked)
                power_example = " ".join(f"{int(a):03d}:{p:.4f}" for a, p in ranked_power)
                gcc_text = f"{gcc_angles[-1]:.1f}" if gcc_angles else "-"
                examples.append(
                    f"t={start / sr:.2f}s base={base_score:.3f} rms={rms:.4f} "
                    f"gcc={gcc_text} score_top={example} power_top={power_example} margin={margin:.3f}"
                )

        mean_top = circular_mean_deg(top_angles)
        mean_gcc = circular_mean_deg(gcc_angles)
        avg_margin = float(np.mean(margins)) if margins else 0.0
        avg_base = float(np.mean(baseline_scores)) if baseline_scores else 0.0
        top_text = "-"
        if top_counter:
            top_text = " ".join(f"{angle}:{count}" for angle, count in top_counter.most_common(6))
        section_text = "-"
        if section_counter:
            section_text = " ".join(f"{name}:{count}" for name, count in section_counter.most_common())
        print(
            f"  gain={gain:.3f} base_avg={avg_base:.3f} scans={scans} accepted={accepted} "
            f"beam_mean={mean_top if mean_top is not None else '-'} "
            f"gcc_mean={mean_gcc if mean_gcc is not None else '-'} "
            f"avg_margin={avg_margin:.3f} top_bins={top_text} sections={section_text}"
        )
        for example in examples:
            print(f"    {example}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+")
    parser.add_argument("--model", default=str(ROOT / "model" / "tello_detector_cnn_retrained_jetson.tflite"))
    parser.add_argument("--config", default=str(ROOT / "model" / "config.json"))
    parser.add_argument("--threshold", type=float, default=float(os.getenv("TELLO_AUDIO_THRESHOLD", "0.50")))
    parser.add_argument("--min-avg", type=float, default=float(os.getenv("TELLO_AUDIO_MIN_AVG_SCORE", "0.55")))
    parser.add_argument("--consecutive", type=int, default=int(os.getenv("TELLO_AUDIO_CONSECUTIVE", "2")))
    parser.add_argument("--min-rms", type=float, default=float(os.getenv("TELLO_AUDIO_MIN_RMS", "0.003")))
    parser.add_argument("--stride-sec", type=float, default=0.25)
    parser.add_argument("--mic-channels", default=os.getenv("TELLO_AUDIO_MIC_CHANNELS", ""))
    parser.add_argument("--mic-distance", type=float, default=float(os.getenv("TELLO_AUDIO_MIC_DISTANCE", "0.045")))
    parser.add_argument("--angles", type=float, nargs="+", default=list(range(0, 360, 30)))
    parser.add_argument("--gains", type=float, nargs="+", default=[1.0, 0.5])
    parser.add_argument("--beam-threshold", type=float, default=0.50)
    parser.add_argument("--margin", type=float, default=0.10)
    parser.add_argument("--max-windows", type=int, default=0)
    parser.add_argument("--examples", type=int, default=4)
    parser.add_argument("--beam-preprocess", action="store_true")
    parser.add_argument("--gcc-preprocess", action="store_true")
    args = parser.parse_args()
    os.environ["TELLO_AUDIO_MIC_DISTANCE"] = str(args.mic_distance)
    for item in args.files:
        evaluate_file(Path(item), args)


if __name__ == "__main__":
    main()
