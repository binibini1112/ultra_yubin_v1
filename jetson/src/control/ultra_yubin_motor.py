import os
import socket
import time


def _get_config_value(name, default):
    try:
        import src.config as config
        return getattr(config, name, default)
    except Exception:
        return default


class UltraYubinMotorController:
    """MotorController-compatible adapter for the ultra_yubin architecture.

    The Jetson keeps running YOLO and audio detection. Instead of writing to a
    local U2D2, this adapter sends bbox/audio cues to Ultra96 PS. Ultra96 writes
    them into PL, reads the PL-computed pan/tilt goal, then sends Dynamixel
    commands through its own USB/U2D2 port.
    """

    def __init__(self):
        self.host = os.getenv("ULTRA_YUBIN_HOST", "192.168.3.1")
        self.port = int(os.getenv("ULTRA_YUBIN_PORT", "5016"))
        self.timeout = float(os.getenv("ULTRA_YUBIN_TIMEOUT_SEC", "0.08"))
        self.default_conf = int(os.getenv("ULTRA_YUBIN_DEFAULT_CONF", "1000"))
        self.default_bbox_w = int(os.getenv("ULTRA_YUBIN_DEFAULT_BBOX_W", "0"))
        self.default_bbox_h = int(os.getenv("ULTRA_YUBIN_DEFAULT_BBOX_H", "0"))
        self.cur_pan = int(_get_config_value("PAN_CENTER", 2048))
        self.cur_tilt = int(os.getenv("ULTRA_YUBIN_CENTER_TILT", "2772"))
        self.pan_min = int(_get_config_value("PAN_MIN", 0))
        self.pan_max = int(_get_config_value("PAN_MAX", 4095))
        self.tilt_min = int(_get_config_value("TILT_MIN", 2340))
        self.tilt_max = int(_get_config_value("TILT_MAX", 3048))
        self.pan_dir = int(_get_config_value("PAN_DIR", 1))
        self.tilt_dir = int(_get_config_value("TILT_DIR", 1))
        self.ready = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self.timeout)
        self.last_telemetry = {
            "ready": False,
            "mode": "ultra_yubin",
            "err_x": 0,
            "err_y": 0,
            "cmd_x": 0.0,
            "cmd_y": 0.0,
            "pan": self.cur_pan,
            "tilt": self.cur_tilt,
            "usb_ok": 0,
            "fpga_reply": "",
            "motor_ms": 0.0,
            "target_active": 0,
        }

    def _request(self, cmd):
        started = time.perf_counter()
        self._sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
        data, _ = self._sock.recvfrom(512)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return data.decode("ascii", errors="replace").strip(), elapsed_ms

    def _parse_reply(self, reply):
        parsed = {}
        for item in reply.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            parsed[key.strip()] = value.strip()
        return parsed

    def _update_from_reply(self, reply, elapsed_ms, err_x=0, err_y=0, active=1):
        parsed = self._parse_reply(reply)
        pan = int(parsed.get("pan", self.cur_pan))
        tilt = int(parsed.get("tilt", self.cur_tilt))
        self.cur_pan = pan
        self.cur_tilt = tilt
        self.last_telemetry.update({
            "ready": True,
            "mode": "ultra_yubin",
            "err_x": int(err_x),
            "err_y": int(err_y),
            "cmd_x": float(err_x),
            "cmd_y": float(err_y),
            "pan": pan,
            "tilt": tilt,
            "usb_ok": int(parsed.get("usb", "0")) if parsed.get("usb", "0").isdigit() else 0,
            "compute_count": int(parsed.get("count", "0")) if parsed.get("count", "0").isdigit() else 0,
            "fpga_reply": reply,
            "motor_ms": float(elapsed_ms),
            "target_active": int(active),
            "pan_limited": pan <= self.pan_min or pan >= self.pan_max,
            "tilt_limited": tilt <= self.tilt_min or tilt >= self.tilt_max,
        })
        return dict(self.last_telemetry)

    def start(self):
        try:
            reply, elapsed_ms = self._request("PLPING")
            if not reply.startswith("PONG,PL,ULTRA_YUBIN"):
                print(f"[ultra_yubin] unexpected PLPING reply: {reply}")
                return self
            self.ready = True
            self._update_from_reply(reply, elapsed_ms, active=0)
            print(f"[ultra_yubin] connected {self.host}:{self.port} {reply}")
        except Exception as exc:
            print(f"[ultra_yubin] connection failed host={self.host} port={self.port}: {exc}")
        self.last_telemetry["ready"] = self.ready
        return self

    def control(self, cx, cy, img_width=1280, img_height=720):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)

        img_width = max(1, int(img_width))
        img_height = max(1, int(img_height))
        cx = int(cx)
        cy = int(cy)
        err_x = cx - img_width // 2
        err_y = cy - img_height // 2
        cmd = (
            f"T {cx} {cy} {self.default_bbox_w} {self.default_bbox_h} "
            f"{img_width} {img_height} {self.default_conf} 1"
        )
        try:
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, err_x, err_y, active=1)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({
                "ready": False,
                "fpga_reply": f"ERR,{exc}",
                "target_active": 0,
            })
            return dict(self.last_telemetry)

    def turn_to_doa(self, doa_angle):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        angle = int(round(float(doa_angle)))
        try:
            reply, elapsed_ms = self._request(f"A {angle} {self.default_conf} 1")
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=1)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def manual_move(self, dx, dy):
        pan = max(self.pan_min, min(self.pan_max, self.cur_pan + int(dx) * self.pan_dir))
        tilt = max(self.tilt_min, min(self.tilt_max, self.cur_tilt + int(dy) * self.tilt_dir))
        return self._send_goal(pan, tilt)

    def center(self):
        return self._send_goal(2048, int(os.getenv("ULTRA_YUBIN_CENTER_TILT", "2772")))

    def _send_goal(self, pan, tilt):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        try:
            reply, elapsed_ms = self._request(f"G {int(pan)} {int(tilt)}")
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def read_status(self):
        if not self.ready:
            return dict(self.last_telemetry)
        try:
            reply, elapsed_ms = self._request("PLPING")
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def stop(self):
        try:
            self._sock.close()
        except Exception:
            pass


MotorController = UltraYubinMotorController
