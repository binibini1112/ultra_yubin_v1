#!/usr/bin/env python3
"""
Drone audio detector pipeline.

ReSpeaker audio input is checked with the local TFLite drone sound model. When
the score stays above threshold for consecutive windows, the current ReSpeaker
DOA is reported through the same on_detect callback shape used by the RTZR STT
pipelines.
"""

import argparse
import math
import os
import queue
import struct
import subprocess
import threading
import time
from collections import deque

import numpy as np


TARGET_SR = 16000
CHUNK_SAMPLES = 8000
CLIP_SAMPLES = 16000
N_FFT = 1024
HOP_LENGTH = 256
N_MELS = 64
FMIN = 50.0
FMAX = 8000.0
DOA_POLL_INTERVAL = 0.1


def doa_to_section(angle: float) -> int:
    return int(((angle + 45) % 360) // 90) + 1


SECTION_LABEL = {1: "front", 2: "right", 3: "rear", 4: "left"}


class ReSpeakerDOA:
    def __init__(self, offset=0):
        import usb.core

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            raise RuntimeError("ReSpeaker not found (idVendor=0x2886, idProduct=0x0018)")
        self._dev = dev
        self._offset = offset
        self._angle = 0
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        time.sleep(0.3)
        print("[DOA] ReSpeaker DOA ready")

    def _loop(self):
        prev = 0
        while self._running:
            try:
                data = self._dev.ctrl_transfer(0xC0, 0, 0xC0, 21, 8, 100000)
                val = struct.unpack(b"ii", bytes(data))[0]
                if 0 <= val <= 359:
                    prev = int((val + self._offset) % 360)
            except Exception:
                try:
                    import usb.core

                    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
                    if dev is not None:
                        self._dev = dev
                except Exception:
                    pass

            with self._lock:
                self._angle = prev
            time.sleep(0.01)

    def read(self) -> int:
        with self._lock:
            return self._angle

    def stop(self):
        self._running = False


class DOAHistory:
    def __init__(self, reader: ReSpeakerDOA, window_sec=1.0):
        self._reader = reader
        self._window_sec = window_sec
        self._samples = deque()
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            now = time.time()
            angle = self._reader.read()
            with self._lock:
                self._samples.append((now, angle))
                cutoff = now - self._window_sec
                while self._samples and self._samples[0][0] < cutoff:
                    self._samples.popleft()
            time.sleep(DOA_POLL_INTERVAL)

    def mean_angle(self):
        with self._lock:
            angles = [angle for _, angle in self._samples]
        if not angles:
            return self._reader.read()

        vec = sum(complex(math.cos(math.radians(a)), math.sin(math.radians(a))) for a in angles)
        if abs(vec) < 1e-6:
            return self._reader.read()
        return float(math.degrees(math.atan2(vec.imag, vec.real)) % 360.0)

    def stop(self):
        self._running = False


def _hz_to_mel(freqs):
    freqs = np.asanyarray(freqs)
    f_sp = 200.0 / 3
    mels = freqs / f_sp

    min_log_hz = 1000.0
    min_log_mel = min_log_hz / f_sp
    logstep = np.log(6.4) / 27.0
    log_t = freqs >= min_log_hz
    mels = np.array(mels, dtype=np.float64)
    mels[log_t] = min_log_mel + np.log(freqs[log_t] / min_log_hz) / logstep
    return mels


def _mel_to_hz(mels):
    mels = np.asanyarray(mels)
    f_sp = 200.0 / 3
    freqs = f_sp * mels

    min_log_hz = 1000.0
    min_log_mel = min_log_hz / f_sp
    logstep = np.log(6.4) / 27.0
    log_t = mels >= min_log_mel
    freqs = np.array(freqs, dtype=np.float64)
    freqs[log_t] = min_log_hz * np.exp(logstep * (mels[log_t] - min_log_mel))
    return freqs


def _mel_filterbank(sr=TARGET_SR, n_fft=N_FFT, n_mels=N_MELS, fmin=FMIN, fmax=FMAX):
    fftfreqs = np.linspace(0, sr / 2, 1 + n_fft // 2)
    min_mel = _hz_to_mel(fmin)
    max_mel = _hz_to_mel(fmax)
    mel_f = _mel_to_hz(np.linspace(min_mel, max_mel, n_mels + 2))

    fdiff = np.diff(mel_f)
    ramps = mel_f[:, np.newaxis] - fftfreqs[np.newaxis, :]
    lower = -ramps[:-2] / fdiff[:-1, np.newaxis]
    upper = ramps[2:] / fdiff[1:, np.newaxis]

    weights = np.maximum(0, np.minimum(lower, upper))
    enorm = 2.0 / (mel_f[2 : n_mels + 2] - mel_f[:n_mels])
    weights *= enorm[:, np.newaxis]
    return weights.astype(np.float32)


_MEL_BASIS = _mel_filterbank()
_HANN = np.hanning(N_FFT + 1)[:-1].astype(np.float32)


def audio_to_logmel(audio):
    y = np.asarray(audio, dtype=np.float32)
    if y.size != CLIP_SAMPLES:
        if y.size < CLIP_SAMPLES:
            y = np.pad(y, (0, CLIP_SAMPLES - y.size))
        else:
            y = y[-CLIP_SAMPLES:]

    peak = float(np.max(np.abs(y)) + 1e-9)
    y = y / peak

    y = np.pad(y, (N_FFT // 2, N_FFT // 2), mode="constant")
    frames = []
    for start in range(0, len(y) - N_FFT + 1, HOP_LENGTH):
        frame = y[start : start + N_FFT] * _HANN
        spectrum = np.fft.rfft(frame, n=N_FFT)
        frames.append(np.abs(spectrum) ** 2)

    power = np.asarray(frames, dtype=np.float32).T
    mel = np.maximum(np.dot(_MEL_BASIS, power), 1e-10)

    ref = float(np.max(mel))
    logmel = 10.0 * np.log10(np.maximum(mel, 1e-10)) - 10.0 * np.log10(max(ref, 1e-10))
    logmel = np.maximum(logmel, -80.0)
    logmel = (logmel + 80.0) / 80.0
    logmel = np.clip(logmel, 0.0, 1.0).astype(np.float32)
    return logmel[..., np.newaxis]


def gcc_phat(sig, refsig, fs=TARGET_SR, max_tau=None, interp=16):
    n = sig.shape[0] + refsig.shape[0]
    sig_fft = np.fft.rfft(sig, n=n)
    ref_fft = np.fft.rfft(refsig, n=n)
    cross = sig_fft * np.conj(ref_fft)
    corr = np.fft.irfft(cross / (np.abs(cross) + 1e-15), n=interp * n)

    max_shift = int(interp * n / 2)
    if max_tau is not None:
        max_shift = min(int(interp * fs * max_tau), max_shift)

    corr = np.concatenate((corr[-max_shift:], corr[: max_shift + 1]))
    shift = int(np.argmax(np.abs(corr)) - max_shift)
    return shift / float(interp * fs)


def estimate_direction_4ch(audio_4ch, fs=TARGET_SR, mic_distance=0.065):
    if audio_4ch.ndim != 2 or audio_4ch.shape[1] < 4:
        raise ValueError("4-channel audio is required for GCC-PHAT DOA")

    c = 343.0
    max_tau = mic_distance / c
    ch0, ch1, ch2, ch3 = [audio_4ch[:, i] for i in range(4)]
    tau_x = gcc_phat(ch0, ch2, fs=fs, max_tau=max_tau)
    tau_y = gcc_phat(ch1, ch3, fs=fs, max_tau=max_tau)

    x = np.clip(tau_x * c / mic_distance, -1.0, 1.0)
    y = np.clip(tau_y * c / mic_distance, -1.0, 1.0)
    return float(np.degrees(np.arctan2(y, x)) % 360.0)


class DroneAudioDetector:
    def __init__(self, model_path):
        try:
            import tflite_runtime.interpreter as tflite
        except ImportError as exc:
            raise RuntimeError(
                "tflite_runtime is required. Install it with: "
                "/home/jetson/yubin/.venv/bin/python3 -m pip install tflite-runtime"
            ) from exc

        if not os.path.exists(model_path):
            raise FileNotFoundError(model_path)

        try:
            self._interpreter = tflite.Interpreter(model_path=model_path)
        except ValueError as exc:
            if "FULLY_CONNECTED" in str(exc):
                raise RuntimeError(
                    "This TFLite model was exported with newer TensorFlow Lite ops than "
                    "the Jetson tflite_runtime can load. Re-export tello_detector.tflite "
                    "from the Keras model without converter.optimizations, or install a "
                    "newer compatible TFLite runtime."
                ) from exc
            raise
        self._interpreter.allocate_tensors()
        self._input = self._interpreter.get_input_details()[0]
        self._output = self._interpreter.get_output_details()[0]

        expected = tuple(self._input["shape"])
        if expected != (1, 64, 63, 1):
            print(f"[AUDIO] warning: unexpected model input shape {expected}")
        print(f"[AUDIO] drone model loaded: {model_path}")

    def predict(self, audio):
        x = audio_to_logmel(audio)[np.newaxis, ...].astype(np.float32)
        self._interpreter.set_tensor(self._input["index"], x)
        self._interpreter.invoke()
        y = self._interpreter.get_tensor(self._output["index"])
        return float(np.ravel(y)[0])


def _find_respeaker_device(sd):
    for i, dev in enumerate(sd.query_devices()):
        if "ReSpeaker" in dev["name"] and dev["max_input_channels"] > 0:
            print(f"[AUDIO] ReSpeaker auto device={i} ({dev['name']})")
            return i
    print("[WARN] ReSpeaker not found by name; using default input device")
    return None


def _usb_reset_respeaker():
    if os.getenv("TELLO_AUDIO_USB_RESET_ON_ERROR", "0") != "1":
        print("[AUDIO] ReSpeaker USB reset skipped (TELLO_AUDIO_USB_RESET_ON_ERROR=0)")
        time.sleep(0.5)
        return
    try:
        import usb.core

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is not None:
            dev.reset()
            print("[AUDIO] ReSpeaker USB reset complete")
            time.sleep(1.5)
    except Exception as exc:
        print(f"[AUDIO] USB reset failed (ignored): {exc}")


def _read_arecord_chunk(proc, n_channels):
    bytes_needed = CHUNK_SAMPLES * n_channels * 2
    chunks = []
    total = 0
    while total < bytes_needed:
        part = proc.stdout.read(bytes_needed - total)
        if not part:
            break
        chunks.append(part)
        total += len(part)
    raw = b"".join(chunks)
    if len(raw) != bytes_needed:
        err = ""
        if proc.stderr is not None:
            err = proc.stderr.read().decode("utf-8", errors="ignore").strip()
        raise RuntimeError(f"arecord stream ended: {err}")
    audio = np.frombuffer(raw, dtype=np.int16).reshape(-1, n_channels)
    return audio.astype(np.float32) / 32768.0


def run(
    device,
    n_channels: int,
    on_detect,
    model_path,
    threshold=0.70,
    consecutive=2,
    cooldown=2.0,
    min_rms=0.008,
    doa_offset=0,
    doa_method="auto",
    mic_distance=0.065,
    audio_backend="arecord",
    alsa_device="plughw:CARD=ArrayUAC10,DEV=0",
    verbose=None,
):
    if verbose is None:
        verbose = os.getenv("TELLO_AUDIO_VERBOSE", "0") == "1"
    detector = DroneAudioDetector(model_path)

    doa_reader = None
    doa_history = None
    if doa_method not in ("auto", "usb", "gcc"):
        raise ValueError("doa_method must be one of: auto, usb, gcc")

    if doa_method in ("auto", "usb"):
        try:
            doa_reader = ReSpeakerDOA(offset=doa_offset)
        except Exception as exc:
            if doa_method == "usb":
                raise
            print(f"[AUDIO-WARNING] USB DOA unavailable: {exc}; falling back to 4ch GCC-PHAT")
    if doa_reader is not None:
        doa_history = DOAHistory(doa_reader)
        active_doa_method = "usb"
    else:
        active_doa_method = "gcc"

    audio_q = queue.Queue()
    audio_buf = np.zeros((0, max(1, int(n_channels))), dtype=np.float32)
    hit_count = 0
    last_detect = 0.0

    if audio_backend not in ("arecord", "sounddevice"):
        raise ValueError("audio_backend must be one of: arecord, sounddevice")

    if audio_backend == "sounddevice":
        import sounddevice as sd

        def callback(indata, frames, time_info, status):
            if status:
                print(f"[AUDIO] input status: {status}")
            audio_q.put(indata.astype(np.float32) / 32768.0)

        if device is None:
            device = _find_respeaker_device(sd)
    else:
        callback = None

    if verbose:
        print("\n[Drone Audio Pipeline] Ctrl+C to stop")
        print(f"  model={model_path}")
        print(f"  threshold={threshold:.2f} consecutive={consecutive} cooldown={cooldown:.1f}s")
        print(f"  rms gate={min_rms:.4f} channels={n_channels} backend={audio_backend}")
        print(f"  device={device if audio_backend == 'sounddevice' else alsa_device}")
        print(f"  doa={active_doa_method} mic_distance={mic_distance:.3f}m\n")

    while True:
        stream = None
        proc = None
        try:
            if audio_backend == "sounddevice":
                stream = sd.InputStream(
                    samplerate=TARGET_SR,
                    channels=n_channels,
                    device=device,
                    blocksize=CHUNK_SAMPLES,
                    dtype="int16",
                    callback=callback,
                )
                stream.start()
            else:
                proc = subprocess.Popen(
                    [
                        "arecord",
                        "-D",
                        alsa_device,
                        "-f",
                        "S16_LE",
                        "-r",
                        str(TARGET_SR),
                        "-c",
                        str(n_channels),
                        "-t",
                        "raw",
                        "-",
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )

            while True:
                if audio_backend == "sounddevice":
                    chunk = audio_q.get(timeout=1.0)
                else:
                    chunk = _read_arecord_chunk(proc, n_channels)
                if chunk.ndim == 1:
                    chunk = chunk[:, np.newaxis]
                audio_buf = np.concatenate((audio_buf, chunk))
                if audio_buf.shape[0] > CLIP_SAMPLES:
                    audio_buf = audio_buf[-CLIP_SAMPLES:]
                if audio_buf.shape[0] < CLIP_SAMPLES:
                    continue

                mono = audio_buf[:, 0]
                rms = float(np.sqrt(np.mean(mono * mono)))
                if doa_history is not None:
                    doa = doa_history.mean_angle()
                elif audio_buf.shape[1] >= 4:
                    doa = estimate_direction_4ch(audio_buf[:, :4], mic_distance=mic_distance)
                else:
                    print("\n[AUDIO-WARNING] GCC-PHAT DOA needs at least 4 channels")
                    time.sleep(1.0)
                    continue
                section = doa_to_section(doa)

                if rms < min_rms:
                    hit_count = 0
                    if verbose:
                        print(
                            f"\r  drone=quiet rms={rms:.4f} doa={doa:6.1f}° "
                            f"section={section}({SECTION_LABEL[section]})      ",
                            end="",
                            flush=True,
                        )
                    continue

                score = detector.predict(mono)
                if score >= threshold:
                    hit_count += 1
                else:
                    hit_count = 0

                if verbose:
                    print(
                        f"\r  drone={score:.3f} hit={hit_count}/{consecutive} rms={rms:.4f} "
                        f"doa={doa:6.1f}° section={section}({SECTION_LABEL[section]})      ",
                        end="",
                        flush=True,
                    )

                now = time.time()
                if hit_count >= consecutive and now - last_detect >= cooldown:
                    last_detect = now
                    action = {"action": "move", "section": section, "confidence": score}
                    print(f"[DRONE-AUDIO] DETECTED score={score:.3f} doa={doa:.1f}° section={section}")
                    on_detect("DRONE_AUDIO", doa, section, False, 1, action)

        except KeyboardInterrupt:
            if stream is not None:
                stream.stop()
                stream.close()
            if proc is not None:
                proc.terminate()
            if doa_history is not None:
                doa_history.stop()
            if doa_reader is not None:
                doa_reader.stop()
            print("\n\n[Drone Audio Pipeline] stopped")
            return
        except Exception as exc:
            print(f"\n[AUDIO-WARNING] mic stream failed; closing stream. error={exc}")
            if stream is not None:
                try:
                    stream.stop()
                    stream.close()
                except Exception:
                    pass
            if proc is not None:
                try:
                    proc.terminate()
                except Exception:
                    pass
            _usb_reset_respeaker()
            time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description="Drone audio detector pipeline")
    parser.add_argument("--device", default=None, help="audio device number or 'list'")
    parser.add_argument("--channels", type=int, default=6)
    parser.add_argument(
        "--model",
        default=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "tello_detector.tflite"),
    )
    parser.add_argument("--threshold", type=float, default=0.70)
    parser.add_argument("--consecutive", type=int, default=2)
    parser.add_argument("--cooldown", type=float, default=2.0)
    parser.add_argument("--min-rms", type=float, default=0.008)
    parser.add_argument("--doa-method", choices=["auto", "usb", "gcc"], default="auto")
    parser.add_argument("--mic-distance", type=float, default=0.065)
    parser.add_argument("--audio-backend", choices=["arecord", "sounddevice"], default="arecord")
    parser.add_argument("--alsa-device", default="plughw:CARD=ArrayUAC10,DEV=0")
    args = parser.parse_args()

    if args.device == "list":
        import sounddevice as sd

        print(sd.query_devices())
        return

    device = int(args.device) if args.device is not None else None

    def on_detect(text, doa, section, from_partial, stage, action):
        print(f"[CALLBACK] text={text} doa={doa:.1f} section={section} action={action}")

    run(
        device,
        args.channels,
        on_detect,
        args.model,
        threshold=args.threshold,
        consecutive=args.consecutive,
        cooldown=args.cooldown,
        min_rms=args.min_rms,
        doa_method=args.doa_method,
        mic_distance=args.mic_distance,
        audio_backend=args.audio_backend,
        alsa_device=args.alsa_device,
    )


if __name__ == "__main__":
    main()
