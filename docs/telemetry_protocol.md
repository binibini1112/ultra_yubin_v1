# Jetson to Raspberry Pi Telemetry Protocol

This document is the shared contract between the Jetson tracker and the Raspberry Pi Tello dashboard.

## Transport

- Direction: Jetson -> Raspberry Pi
- Protocol: UDP
- Default Raspberry Pi listen port: `5005`
- Encoding: UTF-8 JSON
- Recommended send rate: 10-30Hz
- Packet loss is acceptable; each packet must be a complete latest-state snapshot.

Default network assumption:

```text
Jetson Orin Nano  ->  Raspberry Pi eth0 / secondary network
UDP JSON          ->  <pi-ip>:5005

Raspberry Pi wlan0 -> Tello Wi-Fi
```

The Raspberry Pi dashboard does not send Tello control commands based on this telemetry. The Pi drone controller can still react to `laser.hit_detected=true` in the CLI path and execute `flip_forward` followed by `land`.

## Required Fields

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "TRACKING"
}
```

- `timestamp`: Unix time seconds from `time.time()`.
- `target_found`: boolean.
- `state`: tracking state string.

Allowed `state` values:

- `SCANNING`
- `DETECTED`
- `TRACKING`
- `LOCKED`
- `ENGAGED`
- `NEUTRALIZED`
- `LOST`

## Optional Fields

```json
{
  "timestamp": 1715600000.123,
  "frame_id": 10234,
  "fps": 24.8,
  "target_found": true,
  "confidence": 0.87,
  "bbox": {
    "x1": 420,
    "y1": 210,
    "x2": 520,
    "y2": 290,
    "cx": 470,
    "cy": 250,
    "w": 100,
    "h": 80
  },
  "frame": {
    "width": 1280,
    "height": 720
  },
  "error": {
    "x_px": -170,
    "y_px": -110,
    "x_norm": -0.266,
    "y_norm": -0.306
  },
  "ptz": {
    "pan_deg": 12.5,
    "tilt_deg": -4.2,
    "pan_cmd": 18,
    "tilt_cmd": -7
  },
  "audio": {
    "enabled": true,
    "direction_deg": 35.0,
    "confidence": 0.62
  },
  "ultra_ps": {
    "motor_deg": 92.0
  },
  "laser": {
    "armed": false,
    "hit_detected": false
  },
  "state": "TRACKING"
}
```

## Hit Response

When `laser.hit_detected` changes from `false` to `true`, the default `./run_drone.sh` path on Raspberry Pi:

1. stops RC input
2. sends `flip_forward`
3. sends `land`

Jetson may keep `hit_detected=true` in subsequent packets for the same hit event. The Pi reacts only to the rising edge.

## Coordinate Contract

### Image Bounding Box

- `bbox.x1/y1/x2/y2`: image pixel coordinates.
- `bbox.cx/cy`: target center in pixels.
- `bbox.w/h`: bbox width and height in pixels.

### Error

The dashboard treats `error` as relative target offset from the current PTZ/motor center.

```text
frame_cx = frame.width / 2
frame_cy = frame.height / 2
x_px = bbox.cx - frame_cx
y_px = bbox.cy - frame_cy
x_norm = clamp(x_px / frame_cx, -1.0, 1.0)
y_norm = clamp(y_px / frame_cy, -1.0, 1.0)
```

- `x_norm = -1`: left edge
- `x_norm = 0`: motor/PTZ center
- `x_norm = 1`: right edge
- `y_norm = -1`: top edge
- `y_norm = 0`: motor/PTZ center
- `y_norm = 1`: bottom edge

The dashboard graph history uses:

- `fps`
- `confidence`
- `error.x_px`
- Tello battery percentage, when available
- telemetry receive rate in Hz, computed on the Raspberry Pi
- `audio.confidence`

The dashboard radar uses compass-like direction angles:

- north/up is `0` degrees
- west/left is `90` degrees
- south/down is `180` degrees
- east/right is `270` degrees

Displayed radar directions:

- motor/drone direction: `ultra_ps.motor_deg` preferred, with `ptz.pan_deg` as a dashboard fallback
- audio direction: `audio.direction_deg`

The radar does not display real distance in meters.

### PTZ

- `ptz.pan_deg`: current pan angle in degrees.
- `ptz.tilt_deg`: current tilt angle in degrees.
- `ptz.pan_cmd`: latest pan command value, if available.
- `ptz.tilt_cmd`: latest tilt command value, if available.

### Audio

- `audio.direction_deg`: direction angle in degrees.
- `audio.confidence`: `0.0` to `1.0`.

The dashboard shows audio as a direction arrow only.

### Ultra PS

- `ultra_ps.motor_deg`: motor pan/drone direction angle in degrees.

The dashboard also accepts these transitional aliases inside `ultra_ps`: `motor_direction_deg`, `fan_deg`, `heading_deg`, and `direction_deg`.

## Safety Contract

Telemetry is display/logging data only, except for the explicit hit response above.

Do not implement these behaviors on either side in v1:

- automatic Tello movement from bbox/error
- automatic target-following flight
- automatic laser/fire action
- movement based only on audio direction
- automatic emergency from tracking telemetry

Allowed in v1:

- status display
- logging
- operator decision support
- target lost warning
- low battery warning
- hit-confirmed `flip_forward` + `land`

## Jetson Sender Skeleton

```python
import json
import socket
import time


class TelemetrySender:
    def __init__(self, pi_ip: str, pi_port: int = 5005):
        self.addr = (pi_ip, pi_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, payload: dict) -> None:
        try:
            payload.setdefault("timestamp", time.time())
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.addr)
        except OSError as exc:
            print(f"[telemetry] send failed: {exc}")

    def close(self) -> None:
        self.sock.close()


def clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))
```

## Minimal Packet

```python
sender.send({
    "timestamp": time.time(),
    "target_found": True,
    "state": "TRACKING",
})
```

## Full Packet Build Example

```python
def build_payload(frame, target, fps, frame_id, state, pan_deg, tilt_deg):
    frame_h, frame_w = frame.shape[:2]
    payload = {
        "timestamp": time.time(),
        "frame_id": frame_id,
        "fps": fps,
        "target_found": target is not None,
        "state": state,
        "frame": {"width": frame_w, "height": frame_h},
        "ptz": {"pan_deg": pan_deg, "tilt_deg": tilt_deg},
    }

    if target is not None:
        x1, y1, x2, y2 = map(float, target.xyxy)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        frame_cx = frame_w / 2.0
        frame_cy = frame_h / 2.0
        x_px = cx - frame_cx
        y_px = cy - frame_cy

        payload["confidence"] = float(target.confidence)
        payload["bbox"] = {
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "cx": cx,
            "cy": cy,
            "w": x2 - x1,
            "h": y2 - y1,
        }
        payload["error"] = {
            "x_px": x_px,
            "y_px": y_px,
            "x_norm": clamp(x_px / frame_cx) if frame_cx else 0.0,
            "y_norm": clamp(y_px / frame_cy) if frame_cy else 0.0,
        }

    return payload
```
