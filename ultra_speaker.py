#!/usr/bin/env python3
"""Send Jetson ReSpeaker DOA angle to the Ultra96 PS bridge.

Flow:
    Jetson ReSpeaker -> DOA angle -> UDP "A angle conf valid" -> Ultra96 PS

The Ultra96 PS bridge handles the received audio angle and converts it to
the pan goal for the motor side.
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time


PREFERRED_PYTHON = "/home/jetson/yubin/.venv/bin/python3"
ULTRA96_DEFAULT_HOST = "192.168.3.1"
ULTRA96_DEFAULT_PORT = 5016
RESPEAKER_VENDOR_ID = 0x2886
RESPEAKER_PRODUCT_ID = 0x0018


def reexec_preferred_python():
    if os.getenv("ULTRA_SPEAKER_NO_REEXEC") == "1":
        return
    if not os.path.exists(PREFERRED_PYTHON):
        return
    if os.path.abspath(sys.executable) == os.path.abspath(PREFERRED_PYTHON):
        return
    os.execv(PREFERRED_PYTHON, [PREFERRED_PYTHON] + sys.argv)


class ReSpeakerDOA:
    def __init__(self, offset=0):
        try:
            import usb.core
        except Exception as exc:
            raise RuntimeError("pyusb is required to read ReSpeaker DOA") from exc

        dev = usb.core.find(
            idVendor=RESPEAKER_VENDOR_ID,
            idProduct=RESPEAKER_PRODUCT_ID,
        )
        if dev is None:
            raise RuntimeError("ReSpeaker Mic Array not found on Jetson USB")

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
                angle = struct.unpack(b"ii", bytes(data))[0]
                if 0 <= angle <= 359:
                    prev = int((angle + self._offset) % 360)
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


class Ultra96PSClient:
    def __init__(self, host, port, timeout_sec=0.08):
        self.host = host
        self.port = int(port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(float(timeout_sec))

    def request(self, cmd):
        started = time.perf_counter()
        self._sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
        data, _ = self._sock.recvfrom(512)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return data.decode("ascii", errors="replace").strip(), elapsed_ms

    def ping(self):
        return self.request("PLPING")

    def send_angle(self, angle, conf=1000, valid=1):
        return self.request(f"A {int(angle)} {int(conf)} {int(valid)}")

    def close(self):
        self._sock.close()


def normalize_relative_angle(angle, clamp_deg=90.0):
    rel = ((float(angle) + 180.0) % 360.0) - 180.0
    if clamp_deg is not None:
        limit = abs(float(clamp_deg))
        rel = max(-limit, min(limit, rel))
    return rel


def angle_delta(a, b):
    return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.getenv("ULTRA_YUBIN_HOST", ULTRA96_DEFAULT_HOST))
    parser.add_argument("--port", type=int, default=int(os.getenv("ULTRA_YUBIN_PORT", ULTRA96_DEFAULT_PORT)))
    parser.add_argument("--timeout-sec", type=float, default=float(os.getenv("ULTRA_YUBIN_TIMEOUT_SEC", "0.08")))
    parser.add_argument("--offset", type=int, default=int(os.getenv("ULTRA_SPEAKER_OFFSET", "0")))
    parser.add_argument("--period-sec", type=float, default=0.10)
    parser.add_argument("--min-change-deg", type=float, default=4.0)
    parser.add_argument("--keepalive-sec", type=float, default=1.0)
    parser.add_argument("--clamp-deg", type=float, default=90.0)
    parser.add_argument("--no-clamp", action="store_true")
    parser.add_argument("--conf", type=int, default=1000)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    reexec_preferred_python()
    args = parse_args()
    clamp_deg = None if args.no_clamp else args.clamp_deg

    doa = ReSpeakerDOA(offset=args.offset)
    ps = None

    try:
        if not args.dry_run:
            ps = Ultra96PSClient(args.host, args.port, timeout_sec=args.timeout_sec)
            reply, rtt_ms = ps.ping()
            if not reply.startswith("PONG,PL,ULTRA_YUBIN"):
                print(f"[ultra_speaker] Ultra96 PS unexpected reply: {reply}")
                return 2
            print(f"[ultra_speaker] Ultra96 PS connected rtt={rtt_ms:.2f}ms {reply}")

        print(
            f"[ultra_speaker] ReSpeaker ready offset={args.offset} "
            f"period={args.period_sec}s clamp={'off' if clamp_deg is None else clamp_deg}"
        )

        last_sent = None
        next_keepalive = 0.0

        while True:
            now = time.monotonic()
            raw_angle = doa.read()
            rel_angle = normalize_relative_angle(raw_angle, clamp_deg=clamp_deg)
            should_send = (
                last_sent is None
                or angle_delta(rel_angle, last_sent) >= args.min_change_deg
                or now >= next_keepalive
                or args.once
            )

            if should_send:
                send_angle = int(round(rel_angle))
                if args.dry_run:
                    print(f"[ultra_speaker] raw={raw_angle:3d} rel={send_angle:4d} dry-run")
                else:
                    reply, rtt_ms = ps.send_angle(send_angle, conf=args.conf, valid=1)
                    print(
                        f"[ultra_speaker] raw={raw_angle:3d} rel={send_angle:4d} "
                        f"rtt={rtt_ms:.2f}ms reply={reply}"
                    )
                last_sent = rel_angle
                next_keepalive = now + max(0.1, args.keepalive_sec)

            if args.once:
                break
            time.sleep(max(0.01, args.period_sec))

    except KeyboardInterrupt:
        print("[ultra_speaker] stopping")
    finally:
        doa.stop()
        if ps is not None:
            ps.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
