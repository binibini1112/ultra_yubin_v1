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
        self.control_period_sec = float(os.getenv("ULTRA_YUBIN_CONTROL_PERIOD_SEC", "0.02"))
        self.deadband_px = int(os.getenv("ULTRA_YUBIN_DEADBAND_PX", "5"))
        self.smooth_alpha = float(os.getenv("ULTRA_YUBIN_SMOOTH_ALPHA", "1.0"))
        self.aim_offset_x = int(os.getenv("ULTRA_YUBIN_AIM_OFFSET_X", "0"))
        self.aim_offset_y = int(os.getenv("ULTRA_YUBIN_AIM_OFFSET_Y", "0"))
        self.invert_x_to_pl = os.getenv("ULTRA_YUBIN_INVERT_X_TO_PL", "0") == "1"
        self.invert_y_to_pl = os.getenv("ULTRA_YUBIN_INVERT_Y_TO_PL", "1") == "1"
        self.center_on_start = os.getenv("ULTRA_YUBIN_CENTER_ON_START", "1") == "1"
        self.async_send = os.getenv("ULTRA_YUBIN_ASYNC_SEND", "0") == "1"
        self.cur_pan = int(_get_config_value("PAN_CENTER", 2048))
        self.cur_tilt = int(os.getenv("ULTRA_YUBIN_CENTER_TILT", "2772"))
        self.pan_min = int(_get_config_value("PAN_MIN", 0))
        self.pan_max = int(_get_config_value("PAN_MAX", 4095))
        self.tilt_min = int(_get_config_value("TILT_MIN", 2340))
        self.tilt_max = int(_get_config_value("TILT_MAX", 3048))
        self.pan_dir = int(_get_config_value("PAN_DIR", 1))
        self.tilt_dir = int(_get_config_value("TILT_DIR", 1))
        self.ready = False
        self._last_control_sent = 0.0
        self._smooth_cx = None
        self._smooth_cy = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self.timeout)
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_telemetry = {
            "ready": False,
            "mode": "ultra_yubin",
            "err_x": 0,
            "err_y": 0,
            "cmd_x": 0.0,
            "cmd_y": 0.0,
            "aim_cx": 0,
            "aim_cy": 0,
            "send_cx": 0,
            "send_cy": 0,
            "aim_offset_x": self.aim_offset_x,
            "aim_offset_y": self.aim_offset_y,
            "invert_x_to_pl": int(self.invert_x_to_pl),
            "invert_y_to_pl": int(self.invert_y_to_pl),
            "pan": self.cur_pan,
            "tilt": self.cur_tilt,
            "usb_ok": 0,
            "fpga_reply": "",
            "motor_ms": 0.0,
            "target_active": 0,
            "async_send": int(self.async_send),
        }

    def _request(self, cmd):
        started = time.perf_counter()
        self._sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
        data, _ = self._sock.recvfrom(512)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return data.decode("ascii", errors="replace").strip(), elapsed_ms

    def _send_async(self, cmd):
        started = time.perf_counter()
        self._tx_sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (self.host, self.port))
        return (time.perf_counter() - started) * 1000.0

    def _parse_reply(self, reply):
        parsed = {}
        for item in reply.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            parsed[key.strip()] = value.strip()
        return parsed

    def _update_from_reply(
        self,
        reply,
        elapsed_ms,
        err_x=0,
        err_y=0,
        active=1,
        tx_cmd="",
        send_cx=0,
        send_cy=0,
        aim_cx=0,
        aim_cy=0,
    ):
        parsed = self._parse_reply(reply)
        pan = int(parsed.get("pan", self.cur_pan))
        tilt = int(parsed.get("tilt", self.cur_tilt))
        self.cur_pan = pan
        self.cur_tilt = tilt
        reply_kind = reply.split(",", 1)[0] if reply else ""
        self.last_telemetry.update({
            "ready": True,
            "mode": "ultra_yubin",
            "tx_cmd": tx_cmd,
            "rx_reply": reply,
            "reply_kind": reply_kind,
            "err_x": int(err_x),
            "err_y": int(err_y),
            "cmd_x": float(err_x),
            "cmd_y": float(err_y),
            "aim_cx": int(aim_cx),
            "aim_cy": int(aim_cy),
            "send_cx": int(send_cx),
            "send_cy": int(send_cy),
            "aim_offset_x": self.aim_offset_x,
            "aim_offset_y": self.aim_offset_y,
            "invert_x_to_pl": int(self.invert_x_to_pl),
            "invert_y_to_pl": int(self.invert_y_to_pl),
            "pan": pan,
            "tilt": tilt,
            "usb_ok": int(parsed.get("usb", "0")) if parsed.get("usb", "0").isdigit() else 0,
            "compute_count": int(parsed.get("count", "0")) if parsed.get("count", "0").isdigit() else 0,
            "fpga_reply": reply,
            "motor_ms": float(elapsed_ms),
            "target_active": int(active),
            "async_send": int(self.async_send),
            "src": parsed.get("src", ""),
            "ctrl": parsed.get("ctrl", ""),
            "dry": parsed.get("dry", ""),
            "no_pl": parsed.get("no_pl", ""),
            "pan_limited": pan <= self.pan_min or pan >= self.pan_max,
            "tilt_limited": tilt <= self.tilt_min or tilt >= self.tilt_max,
        })
        return dict(self.last_telemetry)

    def start(self):
        try:
            tx_cmd = "PLPING"
            reply, elapsed_ms = self._request(tx_cmd)
            if not reply.startswith("PONG,PL,ULTRA_YUBIN"):
                print(f"[ultra_yubin] unexpected PLPING reply: {reply}")
                return self
            self.ready = True
            self._update_from_reply(reply, elapsed_ms, active=0, tx_cmd=tx_cmd)
            print(f"[ultra_yubin] connected {self.host}:{self.port} {reply}")
            if self.center_on_start:
                center_reply, center_ms = self._request("CENTER")
                self._update_from_reply(center_reply, center_ms, active=0, tx_cmd="CENTER")
                print(f"[ultra_yubin] centered {center_reply}")
        except Exception as exc:
            print(f"[ultra_yubin] connection failed host={self.host} port={self.port}: {exc}")
        self.last_telemetry["ready"] = self.ready
        return self

    def control(
        self,
        cx,
        cy,
        img_width=1280,
        img_height=720,
        bbox_width=None,
        bbox_height=None,
        aim_center_x=None,
        aim_center_y=None,
    ):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)

        img_width = max(1, int(img_width))
        img_height = max(1, int(img_height))
        cx = int(cx)
        cy = int(cy)
        bbox_width = self.default_bbox_w if bbox_width is None else max(0, int(bbox_width))
        bbox_height = self.default_bbox_h if bbox_height is None else max(0, int(bbox_height))
        aim_cx = (img_width // 2 if aim_center_x is None else int(aim_center_x)) + self.aim_offset_x
        aim_cy = (img_height // 2 if aim_center_y is None else int(aim_center_y)) + self.aim_offset_y
        err_x = cx - aim_cx
        err_y = cy - aim_cy
        if abs(err_x) < self.deadband_px and abs(err_y) < self.deadband_px:
            self.last_telemetry.update({
                "ready": True,
                "tx_cmd": "",
                "rx_reply": "SKIP,deadband",
                "reply_kind": "SKIP",
                "err_x": int(err_x),
                "err_y": int(err_y),
                "cmd_x": float(err_x),
                "cmd_y": float(err_y),
                "aim_cx": int(aim_cx),
                "aim_cy": int(aim_cy),
                "fpga_reply": "SKIP,deadband",
                "motor_ms": 0.0,
                "target_active": 1,
                "usb_ok": 0,
            })
            return dict(self.last_telemetry)

        now = time.perf_counter()
        if now - self._last_control_sent < self.control_period_sec:
            self.last_telemetry.update({
                "ready": True,
                "tx_cmd": "",
                "rx_reply": "SKIP,rate",
                "reply_kind": "SKIP",
                "err_x": int(err_x),
                "err_y": int(err_y),
                "cmd_x": float(err_x),
                "cmd_y": float(err_y),
                "aim_cx": int(aim_cx),
                "aim_cy": int(aim_cy),
                "fpga_reply": "SKIP,rate",
                "motor_ms": 0.0,
                "target_active": 1,
                "usb_ok": 0,
            })
            return dict(self.last_telemetry)

        if self._smooth_cx is None:
            self._smooth_cx = float(cx)
            self._smooth_cy = float(cy)
        else:
            alpha = max(0.0, min(1.0, self.smooth_alpha))
            self._smooth_cx = alpha * float(cx) + (1.0 - alpha) * self._smooth_cx
            self._smooth_cy = alpha * float(cy) + (1.0 - alpha) * self._smooth_cy
        send_cx = int(round(self._smooth_cx))
        send_cy = int(round(self._smooth_cy))
        control_cx = send_cx - self.aim_offset_x
        control_cy = send_cy - self.aim_offset_y
        if self.invert_x_to_pl:
            control_cx = img_width - control_cx
        if self.invert_y_to_pl:
            control_cy = img_height - control_cy
        control_cx = max(0, min(img_width, control_cx))
        control_cy = max(0, min(img_height, control_cy))
        cmd = (
            f"T {control_cx} {control_cy} {bbox_width} {bbox_height} "
            f"{img_width} {img_height} {self.default_conf} 1"
        )
        try:
            if self.async_send:
                elapsed_ms = self._send_async(cmd)
                self._last_control_sent = time.perf_counter()
                self.last_telemetry.update({
                    "ready": True,
                    "mode": "ultra_yubin",
                    "tx_cmd": cmd,
                    "rx_reply": "ASYNC,sent",
                    "reply_kind": "ASYNC",
                    "err_x": int(err_x),
                    "err_y": int(err_y),
                    "cmd_x": float(err_x),
                    "cmd_y": float(err_y),
                    "aim_cx": int(aim_cx),
                    "aim_cy": int(aim_cy),
                    "send_cx": int(control_cx),
                    "send_cy": int(control_cy),
                    "aim_offset_x": self.aim_offset_x,
                    "aim_offset_y": self.aim_offset_y,
                    "invert_x_to_pl": int(self.invert_x_to_pl),
                    "invert_y_to_pl": int(self.invert_y_to_pl),
                    "usb_ok": 1,
                    "fpga_reply": "ASYNC,sent",
                    "motor_ms": float(elapsed_ms),
                    "target_active": 1,
                    "src": "pl",
                    "async_send": 1,
                })
                return dict(self.last_telemetry)
            reply, elapsed_ms = self._request(cmd)
            self._last_control_sent = time.perf_counter()
            return self._update_from_reply(
                reply,
                elapsed_ms,
                err_x,
                err_y,
                active=1,
                tx_cmd=cmd,
                send_cx=control_cx,
                send_cy=control_cy,
                aim_cx=aim_cx,
                aim_cy=aim_cy,
            )
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
            cmd = f"A {angle} {self.default_conf} 1"
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=1, tx_cmd=cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def manual_move(self, dx, dy):
        pan = max(self.pan_min, min(self.pan_max, self.cur_pan + int(dx) * self.pan_dir))
        tilt = max(self.tilt_min, min(self.tilt_max, self.cur_tilt + int(dy) * self.tilt_dir))
        return self._send_goal(pan, tilt)

    def center(self):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        try:
            cmd = "CENTER"
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def _send_goal(self, pan, tilt):
        if not self.ready:
            self.last_telemetry["ready"] = False
            return dict(self.last_telemetry)
        try:
            cmd = f"G {int(pan)} {int(tilt)}"
            reply, elapsed_ms = self._request(cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def read_status(self):
        if not self.ready:
            return dict(self.last_telemetry)
        try:
            tx_cmd = "PLPING"
            reply, elapsed_ms = self._request(tx_cmd)
            return self._update_from_reply(reply, elapsed_ms, 0, 0, active=0, tx_cmd=tx_cmd)
        except Exception as exc:
            self.ready = False
            self.last_telemetry.update({"ready": False, "fpga_reply": f"ERR,{exc}"})
            return dict(self.last_telemetry)

    def stop(self):
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            self._tx_sock.close()
        except Exception:
            pass


MotorController = UltraYubinMotorController
