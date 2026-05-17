import json
import math
import os
import queue
import struct
import subprocess
import sys
import threading
import time

import numpy as np


class ReSpeakerDOA:
    def __init__(self, offset=0):
        import usb.core

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            raise RuntimeError("ReSpeaker Mic Array not found")
        self._dev = dev
        self._offset = int(offset)
        self._angle = 0
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        prev = 0
        while self._running:
            try:
                data = self._dev.ctrl_transfer(0xC0, 0, 0xC0, 21, 8, 100000)
                val = struct.unpack(b"ii", bytes(data))[0]
                if 0 <= val <= 359:
                    prev = int((val + self._offset) % 360)
            except Exception:
                pass
            with self._lock:
                self._angle = prev
            time.sleep(0.02)

    def read(self):
        with self._lock:
            return int(self._angle)

    def stop(self):
        self._running = False


def _load_tflite_interpreter(model_path):
    try:
        from tflite_runtime.interpreter import Interpreter
    except Exception:
        from tensorflow.lite.python.interpreter import Interpreter
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter


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


def _mel_filterbank(sr, n_fft, n_mels, fmin, fmax):
    fftfreqs = np.linspace(0, sr / 2, 1 + n_fft // 2)
    mel_f = _mel_to_hz(np.linspace(_hz_to_mel(fmin), _hz_to_mel(fmax), n_mels + 2))
    fdiff = np.diff(mel_f)
    ramps = mel_f[:, np.newaxis] - fftfreqs[np.newaxis, :]
    lower = -ramps[:-2] / fdiff[:-1, np.newaxis]
    upper = ramps[2:] / fdiff[1:, np.newaxis]
    weights = np.maximum(0, np.minimum(lower, upper))
    weights *= (2.0 / (mel_f[2 : n_mels + 2] - mel_f[:n_mels]))[:, np.newaxis]
    return weights.astype(np.float32)


class TelloAudioFallback:
    def __init__(
        self,
        model_path,
        config_path,
        alsa_device="plughw:CARD=ArrayUAC10,DEV=0",
        channels=6,
        threshold=0.70,
        consecutive=2,
        cooldown_sec=1.2,
        min_rms=0.008,
        doa_offset=0,
    ):
        self.model_path = model_path
        self.config_path = config_path
        self.alsa_device = self._resolve_alsa_device(alsa_device)
        self.channels = int(channels)
        self.threshold = float(threshold)
        self.consecutive = int(consecutive)
        self.cooldown_sec = float(cooldown_sec)
        self.min_rms = float(min_rms)
        self._queue = queue.Queue(maxsize=8)
        self._running = False
        self._latest = None
        self._lock = threading.Lock()
        self._proc = None
        self._load_config()
        self._interpreter = _load_tflite_interpreter(model_path)
        self._input = self._interpreter.get_input_details()[0]
        self._output = self._interpreter.get_output_details()[0]
        self._doa = ReSpeakerDOA(offset=doa_offset)

    def _resolve_alsa_device(self, alsa_device):
        if alsa_device and str(alsa_device).lower() not in ("auto", "default"):
            return alsa_device
        try:
            out = subprocess.check_output(
                ["arecord", "-l"],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=1.5,
            )
        except Exception:
            return "plughw:CARD=ArrayUAC10,DEV=0"
        for line in out.splitlines():
            if "ReSpeaker" not in line and "ArrayUAC10" not in line:
                continue
            card_match = None
            dev_match = None
            for token in line.replace(":", " ").replace(",", " ").split():
                if card_match is None and token.isdigit():
                    card_match = token
                elif card_match is not None and dev_match is None and token.isdigit():
                    dev_match = token
                    break
            if "ArrayUAC10" in line:
                return "plughw:CARD=ArrayUAC10,DEV=0"
            if card_match is not None:
                return f"plughw:{card_match},{dev_match or 0}"
        return "plughw:CARD=ArrayUAC10,DEV=0"

    def _load_config(self):
        with open(self.config_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.sample_rate = int(data.get("sample_rate", data.get("sr", 16000)))
        self.clip_samples = int(data.get("clip_samples", data.get("clip_len", 16000)))
        self.n_fft = int(data.get("n_fft", 1024))
        self.hop_length = int(data.get("hop_length", 256))
        self.n_mels = int(data.get("n_mels", 64))
        self.fmin = float(data.get("fmin", 50.0))
        self.fmax = float(data.get("fmax", 8000.0))
        self._mel_basis = _mel_filterbank(
            self.sample_rate, self.n_fft, self.n_mels, self.fmin, self.fmax
        )
        self._hann = np.hanning(self.n_fft + 1)[:-1].astype(np.float32)

    def _audio_to_logmel(self, audio):
        y = np.asarray(audio, dtype=np.float32)
        if y.size < self.clip_samples:
            y = np.pad(y, (0, self.clip_samples - y.size))
        elif y.size > self.clip_samples:
            y = y[-self.clip_samples:]
        y = y / float(np.max(np.abs(y)) + 1e-9)
        y = np.pad(y, (self.n_fft // 2, self.n_fft // 2), mode="constant")
        frames = []
        for start in range(0, len(y) - self.n_fft + 1, self.hop_length):
            frame = y[start : start + self.n_fft] * self._hann
            frames.append(np.abs(np.fft.rfft(frame, n=self.n_fft)) ** 2)
        power = np.asarray(frames, dtype=np.float32).T
        mel = np.maximum(np.dot(self._mel_basis, power), 1e-10)
        ref = float(np.max(mel))
        logmel = 10.0 * np.log10(mel) - 10.0 * np.log10(max(ref, 1e-10))
        logmel = np.clip((np.maximum(logmel, -80.0) + 80.0) / 80.0, 0.0, 1.0)
        return logmel.astype(np.float32)[..., np.newaxis]

    def _predict_score(self, mono):
        feat = self._audio_to_logmel(mono)
        arr = feat[np.newaxis, ...].astype(self._input["dtype"])
        self._interpreter.set_tensor(self._input["index"], arr)
        self._interpreter.invoke()
        out = self._interpreter.get_tensor(self._output["index"])
        return float(np.ravel(out)[-1])

    def start(self):
        if self._running:
            return self
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._infer_loop, daemon=True).start()
        return self

    def _capture_loop(self):
        cmd = [
            "arecord", "-q",
            "-D", self.alsa_device,
            "-f", "S16_LE",
            "-r", str(self.sample_rate),
            "-c", str(self.channels),
            "-t", "raw",
        ]
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        bytes_per_chunk = int(self.sample_rate * 0.25) * self.channels * 2
        while self._running and self._proc.stdout:
            data = self._proc.stdout.read(bytes_per_chunk)
            if not data:
                break
            samples = np.frombuffer(data, dtype=np.int16)
            if samples.size < self.channels:
                continue
            samples = samples[: (samples.size // self.channels) * self.channels]
            audio = samples.reshape(-1, self.channels).astype(np.float32) / 32768.0
            try:
                self._queue.put_nowait(audio[:, 0].copy())
            except queue.Full:
                pass

    def _infer_loop(self):
        buf = np.zeros((0,), dtype=np.float32)
        hits = 0
        last_emit = 0.0
        while self._running:
            try:
                chunk = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            buf = np.concatenate([buf, chunk])
            if buf.size < self.clip_samples:
                continue
            mono = buf[-self.clip_samples :]
            buf = buf[-self.clip_samples :]
            rms = float(np.sqrt(np.mean(mono * mono)))
            if rms < self.min_rms:
                hits = 0
                continue
            score = self._predict_score(mono)
            hits = hits + 1 if score >= self.threshold else 0
            now = time.time()
            if hits >= self.consecutive and now - last_emit >= self.cooldown_sec:
                last_emit = now
                doa = self._normalize_relative_angle(self._doa.read())
                with self._lock:
                    self._latest = {"angle": doa, "score": score, "rms": rms, "time": now}

    def _normalize_relative_angle(self, angle):
        angle = ((float(angle) + 180.0) % 360.0) - 180.0
        return max(-90.0, min(90.0, angle))

    def get_detection(self, max_age_sec=1.5):
        with self._lock:
            latest = dict(self._latest) if self._latest else None
        if latest and time.time() - latest["time"] <= max_age_sec:
            return latest
        return None

    def stop(self):
        self._running = False
        self._doa.stop()
        if self._proc is not None:
            try:
                self._proc.terminate()
            except Exception:
                pass


class JunmoDroneAudioFallback:
    """Run junmoyolo26's drone-audio detector and expose recent DOA detections.

    The original junmoyolo26 path directly controlled the Jetson-side Dynamixel
    motor. In ultra_yubin we keep the detection pipeline on Jetson, but forward
    its detected DOA to Ultra96 PS through the normal `A angle conf valid` path.
    """

    def __init__(
        self,
        model_path,
        project_root="/home/jetson/junmoyolo26",
        alsa_device="auto",
        channels=6,
        threshold=0.70,
        consecutive=2,
        cooldown_sec=0.6,
        min_rms=0.008,
        doa_offset=0,
        doa_method="auto",
        mic_distance=0.065,
        audio_backend="arecord",
        verbose=False,
    ):
        self.project_root = project_root
        self.model_path = model_path
        self.alsa_device = self._resolve_alsa_device(alsa_device)
        self.channels = int(channels)
        self.threshold = float(threshold)
        self.consecutive = int(consecutive)
        self.cooldown_sec = float(cooldown_sec)
        self.min_rms = float(min_rms)
        self.doa_offset = float(doa_offset)
        self.doa_method = str(doa_method)
        self.mic_distance = float(mic_distance)
        self.audio_backend = str(audio_backend)
        self.verbose = bool(verbose)
        self._latest = None
        self._lock = threading.Lock()
        self._thread = None

    def _resolve_alsa_device(self, alsa_device):
        if alsa_device and str(alsa_device).lower() not in ("auto", "default"):
            return alsa_device
        try:
            out = subprocess.check_output(
                ["arecord", "-l"],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=1.5,
            )
        except Exception as exc:
            raise RuntimeError(f"arecord -l failed while finding ReSpeaker: {exc}") from exc
        for line in out.splitlines():
            text = line.lower()
            if not any(key in text for key in ("respeaker", "arrayuac10", "seeed")):
                continue
            parts = line.replace(":", " ").replace(",", " ").split()
            card = None
            dev = None
            for idx, token in enumerate(parts):
                if token == "card" and idx + 1 < len(parts) and parts[idx + 1].isdigit():
                    card = parts[idx + 1]
                if token == "device" and idx + 1 < len(parts) and parts[idx + 1].isdigit():
                    dev = parts[idx + 1]
            if card is not None:
                return f"plughw:{card},{dev or 0}"
        raise RuntimeError("ReSpeaker ALSA capture card not found in `arecord -l`")

    def start(self):
        if self._thread and self._thread.is_alive():
            return self
        local_audio_dir = os.path.join(os.path.dirname(__file__), "audio")
        local_pipeline = os.path.join(local_audio_dir, "junmo_pipeline_drone.py")
        audio_dir = local_audio_dir if os.path.exists(local_pipeline) else os.path.join(self.project_root, "src", "audio")
        pipeline_name = "junmo_pipeline_drone" if os.path.exists(local_pipeline) else "pipeline_drone"
        if not os.path.exists(os.path.join(audio_dir, f"{pipeline_name}.py")):
            raise FileNotFoundError(os.path.join(audio_dir, f"{pipeline_name}.py"))
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)
        if audio_dir not in sys.path:
            sys.path.insert(0, audio_dir)
        pipeline_drone = __import__(pipeline_name)

        def on_detect(text, doa, section, from_partial, stage, action):
            score = 0.0
            if isinstance(action, dict):
                score = float(action.get("confidence", 0.0))
            with self._lock:
                self._latest = {
                    "angle": float(doa),
                    "raw_angle": float(doa),
                    "section": int(section),
                    "score": score,
                    "rms": 0.0,
                    "mode": "junmo",
                    "time": time.time(),
                    "text": text,
                }

        def runner():
            pipeline_drone.run(
                None,
                self.channels,
                on_detect,
                self.model_path,
                threshold=self.threshold,
                consecutive=self.consecutive,
                cooldown=self.cooldown_sec,
                min_rms=self.min_rms,
                doa_offset=self.doa_offset,
                doa_method=self.doa_method,
                mic_distance=self.mic_distance,
                audio_backend=self.audio_backend,
                alsa_device=self.alsa_device,
                verbose=self.verbose,
            )

        self._thread = threading.Thread(target=runner, daemon=True)
        self._thread.start()
        return self

    def get_detection(self, max_age_sec=1.5):
        with self._lock:
            latest = dict(self._latest) if self._latest else None
        if latest and time.time() - latest["time"] <= max_age_sec:
            return latest
        return None

    def stop(self):
        # junmoyolo26's pipeline has no cooperative stop flag. The worker is a
        # daemon thread, so process shutdown stops it. This keeps Ctrl+C fast.
        pass
