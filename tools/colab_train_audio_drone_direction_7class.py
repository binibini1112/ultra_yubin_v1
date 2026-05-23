# Colab script: 7-class drone direction detector for ReSpeaker 4ch WAV.
#
# Goal:
#   class 0 = noise/background
#   class 1 = drone_000deg
#   class 2 = drone_060deg
#   class 3 = drone_120deg
#   class 4 = drone_180deg
#   class 5 = drone_240deg
#   class 6 = drone_300deg
#
# Paste this whole file into Colab, or upload it and run:
#   %run colab_train_audio_drone_direction_7class.py

from pathlib import Path
import json
import math
import os
import random
import shutil
import zipfile

import numpy as np
import pandas as pd
import soundfile as sf
import librosa
import matplotlib.pyplot as plt

from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import train_test_split
from sklearn.utils.class_weight import compute_class_weight

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers


# -------------------------
# User settings
# -------------------------
DATASET_DIR = Path("/content/dataset_audio_angle")
ARTIFACT_DIR = DATASET_DIR / "colab_drone_direction_7class_1s"

SR = 16000
SEGMENT_SEC = 1.0
SEGMENT_SAMPLES = int(SR * SEGMENT_SEC)

N_MELS = 64
N_FFT = 1024
HOP_LENGTH = 256
FMIN = 50
FMAX = 8000

SEED = 42
TRAIN_RATIO = 0.70
VAL_RATIO = 0.15
TEST_RATIO = 0.15

# Augmentation. Keep this enabled: it teaches "drone angle survives background noise".
MIX_BACKGROUND_AUG = True
MIX_PER_DRONE_TRAIN_SEGMENT = 2
SNR_DB_RANGE = (-6.0, 8.0)
GAIN_RANGE = (0.70, 1.30)
GAUSSIAN_NOISE_RANGE = (0.0000, 0.0020)

random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)

CLASS_TO_NAME = {
    0: "noise",
    1: "drone_000deg",
    2: "drone_060deg",
    3: "drone_120deg",
    4: "drone_180deg",
    5: "drone_240deg",
    6: "drone_300deg",
}
ANGLE_TO_CLASS = {0: 1, 60: 2, 120: 3, 180: 4, 240: 5, 300: 6}
CLASS_TO_ANGLE = {0: None, 1: 0, 2: 60, 3: 120, 4: 180, 5: 240, 6: 300}


def ensure_dataset_dir():
    if DATASET_DIR.exists():
        return
    try:
        from google.colab import files
    except Exception as exc:
        raise RuntimeError(f"DATASET_DIR not found: {DATASET_DIR}") from exc
    print("Upload dataset_audio_angle.zip")
    uploaded = files.upload()
    zip_name = next(iter(uploaded.keys()))
    with zipfile.ZipFile(zip_name, "r") as z:
        z.extractall(Path("/content"))
    if not DATASET_DIR.exists():
        raise RuntimeError(f"After unzip, DATASET_DIR still not found: {DATASET_DIR}")


def read_audio(path):
    audio, sr = sf.read(str(path), always_2d=True, dtype="float32")
    if sr != SR:
        audio = librosa.resample(audio.T, orig_sr=sr, target_sr=SR).T.astype(np.float32)
    if audio.shape[1] != 4:
        raise ValueError(f"{path} has {audio.shape[1]} channels, expected 4")
    return np.clip(audio.astype(np.float32), -1.0, 1.0)


def segment_audio(audio, pad_last=False):
    out = []
    total = audio.shape[0]
    for start in range(0, total, SEGMENT_SAMPLES):
        end = start + SEGMENT_SAMPLES
        if end <= total:
            out.append((start, audio[start:end]))
        elif pad_last and total - start >= int(0.5 * SEGMENT_SAMPLES):
            padded = np.pad(audio[start:total], ((0, end - total), (0, 0)))
            out.append((start, padded))
    return out


def common_ref_logmel(audio_4ch):
    mels = []
    ref = 1e-10
    for ch in range(4):
        mel = librosa.feature.melspectrogram(
            y=audio_4ch[:, ch],
            sr=SR,
            n_fft=N_FFT,
            hop_length=HOP_LENGTH,
            n_mels=N_MELS,
            fmin=FMIN,
            fmax=FMAX,
            power=2.0,
        ).astype(np.float32)
        mel = np.maximum(mel, 1e-10)
        mels.append(mel)
        ref = max(ref, float(np.max(mel)))
    feats = [10.0 * np.log10(mel) - 10.0 * np.log10(ref) for mel in mels]
    return np.stack(feats, axis=-1).astype(np.float32)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)) + 1e-12))


def mix_with_background(drone, bg, snr_db):
    if bg.shape[0] != drone.shape[0]:
        if bg.shape[0] > drone.shape[0]:
            start = np.random.randint(0, bg.shape[0] - drone.shape[0] + 1)
            bg = bg[start : start + drone.shape[0]]
        else:
            reps = int(np.ceil(drone.shape[0] / bg.shape[0]))
            bg = np.tile(bg, (reps, 1))[: drone.shape[0]]
    drone_rms = rms(drone)
    bg_rms = rms(bg)
    target_bg_rms = drone_rms / (10.0 ** (snr_db / 20.0))
    bg_gain = target_bg_rms / max(bg_rms, 1e-8)
    mixed = drone + bg * bg_gain
    peak = float(np.max(np.abs(mixed)))
    if peak > 0.98:
        mixed = mixed / peak * 0.98
    return mixed.astype(np.float32)


def augment_audio(audio):
    gain = np.random.uniform(*GAIN_RANGE)
    y = audio * gain
    noise_std = np.random.uniform(*GAUSSIAN_NOISE_RANGE)
    if noise_std > 0:
        y = y + np.random.normal(0.0, noise_std, size=y.shape).astype(np.float32)
    shift = np.random.randint(-800, 801)
    if shift != 0:
        y = np.roll(y, shift, axis=0)
    return np.clip(y, -1.0, 1.0).astype(np.float32)


def load_segments():
    meta = pd.read_csv(DATASET_DIR / "metadata.csv")
    rows = []
    audio_segments = []
    for _, row in meta.iterrows():
        wav_path = DATASET_DIR / str(row["file_path"])
        if not wav_path.exists():
            print("[WARN] missing:", wav_path)
            continue
        try:
            audio = read_audio(wav_path)
        except Exception as exc:
            print("[WARN] skip:", wav_path, exc)
            continue

        source_type = str(row.get("source_type", ""))
        angle_deg = int(row.get("angle_deg", -1))
        if source_type == "background":
            label = 0
        else:
            if angle_deg not in ANGLE_TO_CLASS:
                print("[WARN] unknown angle:", wav_path, angle_deg)
                continue
            label = ANGLE_TO_CLASS[angle_deg]

        for seg_idx, (start_sample, seg) in enumerate(segment_audio(audio, pad_last=False)):
            item = row.to_dict()
            item.update(
                {
                    "source_file_path": str(row["file_path"]),
                    "segment_index": seg_idx,
                    "segment_start_sec": start_sample / SR,
                    "segment_end_sec": (start_sample + SEGMENT_SAMPLES) / SR,
                    "label": label,
                    "label_name": CLASS_TO_NAME[label],
                }
            )
            rows.append(item)
            audio_segments.append(seg)
    seg_meta = pd.DataFrame(rows)
    audio_segments = np.asarray(audio_segments, dtype=np.float32)
    return seg_meta, audio_segments


def split_indices(seg_meta):
    labels = seg_meta["label"].astype(int).to_numpy()
    idx = np.arange(len(seg_meta))

    # Dataset is small and class/source distribution is uneven. Stratified segment
    # split is used intentionally here for a deployable smoke-test model.
    train_idx, temp_idx = train_test_split(
        idx, train_size=TRAIN_RATIO, random_state=SEED, stratify=labels
    )
    temp_labels = labels[temp_idx]
    val_size_in_temp = VAL_RATIO / (VAL_RATIO + TEST_RATIO)
    val_idx, test_idx = train_test_split(
        temp_idx,
        train_size=val_size_in_temp,
        random_state=SEED,
        stratify=temp_labels,
    )
    return train_idx, val_idx, test_idx


def build_feature_set(seg_meta, audio_segments, indices, train_bg_segments=None, augment=False):
    xs = []
    ys = []
    out_rows = []
    for idx in indices:
        row = seg_meta.iloc[int(idx)].to_dict()
        label = int(row["label"])
        audio = audio_segments[int(idx)]
        variants = [audio]

        if augment and label != 0:
            for _ in range(MIX_PER_DRONE_TRAIN_SEGMENT):
                y = augment_audio(audio)
                if MIX_BACKGROUND_AUG and train_bg_segments is not None and len(train_bg_segments):
                    bg = train_bg_segments[np.random.randint(0, len(train_bg_segments))]
                    y = mix_with_background(y, bg, np.random.uniform(*SNR_DB_RANGE))
                variants.append(y)
        elif augment and label == 0:
            variants.append(augment_audio(audio))

        for v_i, variant in enumerate(variants):
            xs.append(common_ref_logmel(variant))
            ys.append(label)
            r = dict(row)
            r["augment_index"] = v_i
            out_rows.append(r)
    return np.asarray(xs, dtype=np.float32), np.asarray(ys, dtype=np.int64), pd.DataFrame(out_rows)


def normalize_features(train_x, val_x, test_x):
    mean = train_x.mean(axis=(0, 1, 2), keepdims=True).astype(np.float32)
    std = (train_x.std(axis=(0, 1, 2), keepdims=True) + 1e-6).astype(np.float32)
    return (train_x - mean) / std, (val_x - mean) / std, (test_x - mean) / std, mean, std


def make_model(input_shape):
    inp = keras.Input(shape=input_shape)
    x = layers.Conv2D(24, (3, 3), padding="same", activation="relu")(inp)
    x = layers.MaxPooling2D((2, 2))(x)
    x = layers.Conv2D(48, (3, 3), padding="same", activation="relu")(x)
    x = layers.MaxPooling2D((2, 2))(x)
    x = layers.Conv2D(64, (3, 3), padding="same", activation="relu")(x)
    x = layers.Dropout(0.25)(x)
    # No Dense layer: this avoids old Jetson tflite FULLY_CONNECTED version issues.
    x = layers.Conv2D(7, (1, 1), padding="same")(x)
    x = layers.GlobalAveragePooling2D()(x)
    out = layers.Activation("softmax")(x)
    model = keras.Model(inp, out)
    model.compile(
        optimizer=keras.optimizers.Adam(1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    return model


def save_plot(history):
    hist = pd.DataFrame(history.history)
    hist.to_csv(ARTIFACT_DIR / "training_history.csv", index=False)
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].plot(hist["accuracy"], label="train")
    ax[0].plot(hist["val_accuracy"], label="val")
    ax[0].set_title("accuracy")
    ax[0].legend()
    ax[1].plot(hist["loss"], label="train")
    ax[1].plot(hist["val_loss"], label="val")
    ax[1].set_title("loss")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(ARTIFACT_DIR / "accuracy_loss.png", dpi=160)
    plt.show()


def evaluate_and_save(model, test_x, test_y, test_meta):
    probs = model.predict(test_x, batch_size=64)
    pred = np.argmax(probs, axis=1)
    acc = float(accuracy_score(test_y, pred))
    labels = list(range(7))
    names = [CLASS_TO_NAME[i] for i in labels]

    print("TEST ACC:", acc)
    cm = confusion_matrix(test_y, pred, labels=labels)
    print(cm)
    report = classification_report(test_y, pred, labels=labels, target_names=names, zero_division=0)
    print(report)

    (ARTIFACT_DIR / "classification_report.txt").write_text(report, encoding="utf-8")
    summary = {"test_accuracy": acc, "confusion_matrix": cm.tolist(), "class_names": CLASS_TO_NAME}
    (ARTIFACT_DIR / "eval_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    out = test_meta.copy().reset_index(drop=True)
    out["pred_class"] = pred
    out["pred_name"] = [CLASS_TO_NAME[int(p)] for p in pred]
    out["correct"] = (pred == test_y)
    for c in labels:
        out[f"prob_{CLASS_TO_NAME[c]}"] = probs[:, c]
    out.to_csv(ARTIFACT_DIR / "test_predictions.csv", index=False)

    print("\naccuracy by true label")
    print(out.groupby("label_name")["correct"].mean())
    if "distance_m" in out.columns:
        print("\naccuracy by distance")
        print(out.groupby("distance_m")["correct"].mean())
    if "source_type" in out.columns:
        print("\naccuracy by source")
        print(out.groupby("source_type")["correct"].mean())

    plt.figure(figsize=(7, 6))
    plt.imshow(cm, cmap="Blues")
    plt.xticks(range(7), names, rotation=45, ha="right")
    plt.yticks(range(7), names)
    plt.xlabel("pred")
    plt.ylabel("true")
    plt.title("7-class confusion matrix")
    for i in range(7):
        for j in range(7):
            plt.text(j, i, str(cm[i, j]), ha="center", va="center", color="black")
    plt.tight_layout()
    plt.savefig(ARTIFACT_DIR / "confusion_matrix.png", dpi=160)
    plt.show()


def export_tflite(model):
    model.save(ARTIFACT_DIR / "best_model.keras")
    saved_model_dir = ARTIFACT_DIR / "saved_model"
    if saved_model_dir.exists():
        shutil.rmtree(saved_model_dir)
    model.export(str(saved_model_dir))

    converter = tf.lite.TFLiteConverter.from_saved_model(str(saved_model_dir))
    converter.optimizations = []
    tflite_model = converter.convert()
    out_path = ARTIFACT_DIR / "drone_direction_7class_1s.tflite"
    out_path.write_bytes(tflite_model)
    print("saved:", out_path)


def main():
    ensure_dataset_dir()
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)

    seg_meta, audio_segments = load_segments()
    seg_meta.to_csv(ARTIFACT_DIR / "segments_1s_metadata_7class.csv", index=False)

    print("[segments]")
    print("segments:", len(seg_meta), "audio:", audio_segments.shape)
    print(pd.crosstab(seg_meta["label_name"], seg_meta["source_type"]))
    print(pd.crosstab(seg_meta["label_name"], seg_meta["distance_m"]))

    train_idx, val_idx, test_idx = split_indices(seg_meta)
    bg_train = audio_segments[train_idx[seg_meta.iloc[train_idx]["label"].to_numpy() == 0]]

    train_x, train_y, train_meta = build_feature_set(
        seg_meta, audio_segments, train_idx, train_bg_segments=bg_train, augment=True
    )
    val_x, val_y, val_meta = build_feature_set(seg_meta, audio_segments, val_idx, augment=False)
    test_x, test_y, test_meta = build_feature_set(seg_meta, audio_segments, test_idx, augment=False)

    train_x, val_x, test_x, mean, std = normalize_features(train_x, val_x, test_x)
    print("[features]")
    print("train:", train_x.shape, np.bincount(train_y, minlength=7))
    print("val:", val_x.shape, np.bincount(val_y, minlength=7))
    print("test:", test_x.shape, np.bincount(test_y, minlength=7))

    np.save(ARTIFACT_DIR / "feature_mean.npy", mean)
    np.save(ARTIFACT_DIR / "feature_std.npy", std)
    (ARTIFACT_DIR / "feature_config.json").write_text(
        json.dumps(
            {
                "sample_rate": SR,
                "segment_sec": SEGMENT_SEC,
                "n_mels": N_MELS,
                "n_fft": N_FFT,
                "hop_length": HOP_LENGTH,
                "fmin": FMIN,
                "fmax": FMAX,
                "channels": 4,
                "classes": 7,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    (ARTIFACT_DIR / "label_mapping.json").write_text(
        json.dumps(
            {
                "class_to_name": CLASS_TO_NAME,
                "class_to_angle": CLASS_TO_ANGLE,
                "angle_to_class": ANGLE_TO_CLASS,
                "noise_class": 0,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    class_weights_arr = compute_class_weight(
        class_weight="balanced",
        classes=np.arange(7),
        y=train_y,
    )
    class_weight = {i: float(class_weights_arr[i]) for i in range(7)}
    print("class_weight:", class_weight)

    model = make_model(train_x.shape[1:])
    model.summary()
    callbacks = [
        keras.callbacks.ModelCheckpoint(
            ARTIFACT_DIR / "best_model.keras",
            monitor="val_accuracy",
            save_best_only=True,
            mode="max",
        ),
        keras.callbacks.EarlyStopping(
            monitor="val_accuracy",
            patience=18,
            mode="max",
            restore_best_weights=True,
        ),
        keras.callbacks.ReduceLROnPlateau(
            monitor="val_loss",
            factor=0.5,
            patience=6,
            min_lr=1e-5,
        ),
    ]
    history = model.fit(
        train_x,
        train_y,
        validation_data=(val_x, val_y),
        epochs=150,
        batch_size=32,
        class_weight=class_weight,
        callbacks=callbacks,
        verbose=1,
    )

    save_plot(history)
    best_model = keras.models.load_model(ARTIFACT_DIR / "best_model.keras", compile=False)
    best_model.compile(
        optimizer=keras.optimizers.Adam(1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    evaluate_and_save(best_model, test_x, test_y, test_meta)
    export_tflite(best_model)

    zip_path = Path("/content/colab_drone_direction_7class_1s.zip")
    if zip_path.exists():
        zip_path.unlink()
    shutil.make_archive(str(zip_path.with_suffix("")), "zip", ARTIFACT_DIR)
    print("zip:", zip_path)


main()
