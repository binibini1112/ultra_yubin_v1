# Jetson -> Raspberry Pi Sender Contract

Jetson에서 Raspberry Pi 대시보드로 보내야 하는 UDP JSON 형식입니다.

## 목적

Jetson은 YOLO/tracking 결과와 방향 정보를 Raspberry Pi로 보냅니다. Raspberry Pi는 이 데이터를 대시보드 표시와 로그 저장에 쓰고, 기본 설정에서는 `laser.hit_detected=true` 상승 엣지를 받으면 `flip_forward` 뒤 `land`를 실행합니다.

자동 Tello 이동, 자동 발사, 자동 emergency는 사용하지 않습니다. 피격 반응만 예외적으로 허용합니다.

## 네트워크

```text
Jetson Orin Nano  ->  Raspberry Pi
UDP JSON          ->  <raspberry-pi-ip>:5005
```

기본값:

- Protocol: UDP
- Encoding: UTF-8 JSON
- Raspberry Pi listen port: `5005`
- Recommended send rate: `10-30Hz`

예시:

```text
Raspberry Pi dashboard output:
  Jetson UDP target candidates:
    <pi-ip-from-dashboard>:5005
Jetson sends to one of those candidates.
```

Jetson 자신의 IP가 따로 있어도, 패킷 목적지는 Raspberry Pi IP입니다.
캡스톤실, 체육관처럼 장소가 바뀌면 Raspberry Pi IP도 바뀔 수 있으므로,
Jetson sender 코드에 IP를 고정하지 말고 실행 시 `PI_IP`로 전달합니다.

## 최소 필수 패킷

아래 3개 필드는 반드시 포함해야 합니다.

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "TRACKING"
}
```

필드 의미:

- `timestamp`: Jetson 기준 Unix time seconds, Python `time.time()`
- `target_found`: 타겟 검출 여부, boolean
- `state`: 추적 상태 문자열

허용 상태값:

- `SCANNING`
- `DETECTED`
- `TRACKING`
- `LOCKED`
- `ENGAGED`
- `NEUTRALIZED`
- `LOST`

## 발사 횟수

대시보드의 `Shots`는 Jetson에서 fire 키/액션이 실행된 횟수입니다. `laser.armed=true`는 조준/발사 가능 상태일 뿐이라 shot으로 세지 않습니다. `laser.hit_detected=true`는 `Hits`만 올립니다.

권장 방식은 Jetson이 f 입력마다 `laser.shot_count`를 1씩 증가시켜 누적값으로 보내는 것입니다. UDP 패킷 손실이 있어도 누적값이면 대시보드가 최종 횟수를 맞출 수 있습니다.

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "LOCKED",
  "laser": {
    "armed": true,
    "fired": true,
    "shot_count": 7,
    "hit_detected": false
  }
}
```

`shot_count`를 보내기 어렵다면 f 입력 순간에 `laser.fired=true`를 보낸 뒤 다음 패킷에서 `false`로 내려도 됩니다.

## 피격 이벤트

피격 판정은 `laser.hit_detected=true`로 보냅니다. Raspberry Pi는 이 값이 `false -> true`로 바뀌는 순간을 피격으로 보고, 기본 설정에서 다음 순서로 동작합니다.

1. RC 입력 정지
2. `flip_forward`
3. `land`

같은 피격 동안 `hit_detected=true`가 계속 반복되어도 Pi는 한 번만 반응합니다.

권장 피격 패킷:

```json
{
  "timestamp": 1715600000.123,
  "target_found": true,
  "state": "LOCKED",
  "laser": {
    "armed": true,
    "fired": false,
    "shot_count": 7,
    "hit_detected": true
  }
}
```

## 권장 전체 패킷

```json
{
  "timestamp": 1715600000.123,
  "frame_id": 1234,
  "target_found": true,
  "state": "TRACKING",
  "fps": 24.5,
  "confidence": 0.82,
  "frame": {
    "width": 1280,
    "height": 720
  },
  "bbox": {
    "x1": 520,
    "y1": 260,
    "x2": 620,
    "y2": 340,
    "cx": 570,
    "cy": 300,
    "w": 100,
    "h": 80
  },
  "error": {
    "x_px": -70,
    "y_px": -60,
    "x_norm": -0.109,
    "y_norm": -0.167
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
    "fired": false,
    "shot_count": 0,
    "hit_detected": false
  }
}
```

## 좌표 규칙

`bbox`는 이미지 픽셀 좌표입니다.

```text
x1, y1: bbox left-top
x2, y2: bbox right-bottom
cx, cy: bbox center
w, h: bbox width / height
```

`error`는 프레임 중심 대비 타겟 중심 오차입니다.

```text
frame_cx = frame.width / 2
frame_cy = frame.height / 2
x_px = bbox.cx - frame_cx
y_px = bbox.cy - frame_cy
x_norm = clamp(x_px / frame_cx, -1.0, 1.0)
y_norm = clamp(y_px / frame_cy, -1.0, 1.0)
```

의미:

- `x_norm = -1`: 왼쪽 끝
- `x_norm = 0`: 화면 중심
- `x_norm = 1`: 오른쪽 끝
- `y_norm = -1`: 위쪽 끝
- `y_norm = 0`: 화면 중심
- `y_norm = 1`: 아래쪽 끝

## PS / 방향 규칙

PS 방향값은 `ultra_ps` 안에 넣습니다.

권장 필드:

```json
{
  "ultra_ps": {
    "motor_deg": 92.0
  }
}
```

대시보드는 아래 순서로 방향값을 찾습니다.

1. `ultra_ps.motor_deg`
2. `ultra_ps.motor_direction_deg`
3. `ultra_ps.fan_deg`
4. `ultra_ps.heading_deg`
5. `ultra_ps.direction_deg`
6. fallback: `ptz.pan_deg`

각도 규칙:

- `0 deg`: 위쪽 / north
- `90 deg`: 왼쪽 / west
- `180 deg`: 아래쪽 / south
- `270 deg`: 오른쪽 / east

## UltraPS -> Jetson 입력 규칙

UltraPS 원천 장치가 Serial, USB, UDP, I2C 중 무엇으로 연결되는지는 Jetson 구현 쪽에서 결정합니다. Raspberry Pi 대시보드는 원천 장치의 raw packet을 직접 받지 않습니다.

Jetson 쪽에서는 UltraPS raw 값을 읽은 뒤, 아래 내부 형태로 정규화해서 최종 telemetry JSON의 `ultra_ps`에 넣으면 됩니다.

Jetson 내부 권장 객체:

```json
{
  "motor_deg": 92.0,
  "confidence": 0.85,
  "source": "ultraps",
  "timestamp": 1715600000.123
}
```

Pi로 보낼 때 반드시 필요한 값은 `motor_deg` 하나입니다.

```json
{
  "ultra_ps": {
    "motor_deg": 92.0
  }
}
```

Jetson에서 UltraPS raw 값을 파싱할 때 허용할 만한 입력 예시는 아래처럼 잡으면 됩니다.

### JSON line 예시

```json
{"motor_deg":92.0}
```

또는:

```json
{"heading_deg":92.0,"confidence":0.85}
```

권장 alias 처리 순서:

1. `motor_deg`
2. `motor_direction_deg`
3. `heading_deg`
4. `direction_deg`
5. `fan_deg`

### CSV line 예시

```text
92.0
```

또는:

```text
92.0,0.85
```

CSV를 쓰는 경우:

- 첫 번째 값: motor direction degrees
- 두 번째 값: confidence, optional

### Jetson 정규화 함수 예시

```python
import json
import time


def normalize_ultraps(raw):
    if raw is None:
        return None

    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore").strip()

    if isinstance(raw, str):
        raw = raw.strip()
        if not raw:
            return None
        if raw.startswith("{"):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError:
                return None
        else:
            parts = [part.strip() for part in raw.split(",")]
            try:
                data = {"motor_deg": float(parts[0])}
                if len(parts) > 1:
                    data["confidence"] = float(parts[1])
                raw = data
            except (TypeError, ValueError):
                return None

    if not isinstance(raw, dict):
        return None

    for key in ("motor_deg", "motor_direction_deg", "heading_deg", "direction_deg", "fan_deg"):
        value = raw.get(key)
        if value is None:
            continue
        try:
            motor_deg = float(value) % 360.0
        except (TypeError, ValueError):
            return None

        result = {
            "motor_deg": motor_deg,
            "timestamp": float(raw.get("timestamp", time.time())),
        }

        if raw.get("confidence") is not None:
            try:
                result["confidence"] = float(raw["confidence"])
            except (TypeError, ValueError):
                pass

        if raw.get("source") is not None:
            result["source"] = str(raw["source"])

        return result

    return None
```

최종 telemetry에 붙이는 방식:

```python
ultraps = normalize_ultraps(raw_ultraps_packet)
if ultraps is not None:
    payload["ultra_ps"] = {
        "motor_deg": ultraps["motor_deg"],
    }
```

## Jetson Python 송신 예시

```python
import json
import os
import socket
import time


PI_IP = os.environ["PI_IP"]
PI_PORT = int(os.environ.get("PI_PORT", "5005"))


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

frame_id = 0

while True:
    frame_w = 1280
    frame_h = 720

    # TODO: replace these values with YOLO output.
    target_found = True
    x1, y1, x2, y2 = 520.0, 260.0, 620.0, 340.0
    confidence = 0.82

    payload = {
        "timestamp": time.time(),
        "frame_id": frame_id,
        "target_found": target_found,
        "state": "TRACKING" if target_found else "SCANNING",
        "fps": 24.5,
        "frame": {
            "width": frame_w,
            "height": frame_h,
        },
        "audio": {
            "enabled": True,
            "direction_deg": 35.0,
            "confidence": 0.62,
        },
        "ultra_ps": {
            "motor_deg": 92.0,
        },
        "laser": {
            "armed": False,
            "hit_detected": False,
        },
    }

    if target_found:
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        frame_cx = frame_w / 2.0
        frame_cy = frame_h / 2.0
        x_px = cx - frame_cx
        y_px = cy - frame_cy

        payload["confidence"] = confidence
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
            "x_norm": clamp(x_px / frame_cx),
            "y_norm": clamp(y_px / frame_cy),
        }

    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    sock.sendto(data, (PI_IP, PI_PORT))

    frame_id += 1
    time.sleep(0.05)
```

## Raspberry Pi에서 확인

Pi에서 dashboard 실행:

```bash
python -m tello_control.dashboard --host 0.0.0.0 --telemetry-host 0.0.0.0 --telemetry-port 5005
```

실행하면 현재 네트워크에서 Jetson이 보낼 목적지 후보를 출력합니다.

```text
Dashboard URLs:
  http://127.0.0.1:8000
  http://<pi-ip-from-dashboard>:8000
Jetson UDP target candidates:
  <pi-ip-from-dashboard>:5005
```

Jetson에서는 위 후보 중 Raspberry Pi와 같은 네트워크의 주소를 넣어 실행합니다.

```bash
PI_IP=<pi-ip-from-dashboard> PI_PORT=5005 python jetson_sender.py
```

브라우저:

```text
http://<raspberry-pi-ip>:8000
```

Jetson 패킷이 들어오면 UI의 Jetson status가 `CONNECTED`가 됩니다.

`./run_drone.sh`는 기본으로 이 피격 반응을 켠 상태로 실행됩니다.

수신 상태 기준:

- `0.0~1.0s`: `CONNECTED`
- `1.0~3.0s`: `STALE`
- `3.0s+`: `DISCONNECTED`
