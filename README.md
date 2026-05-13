# ultra_yubin

Jetson YOLO/Audio -> Ultra96 PS -> Ultra96 PL bbox goal compute -> Ultra96 PS USB/U2D2 -> Dynamixel 구조만 남긴 최소 프로젝트입니다.

## 핵심 구조

- Jetson: 카메라 YOLO bbox 산출, YOLO 미검출 시 ReSpeaker/Tello audio fallback 각도 산출, UDP 5016 송신
- Ultra96 PS: UDP 수신, AXI 레지스터 write/read, audio angle-to-pan 변환, USB/U2D2 Dynamixel 명령 송신
- Ultra96 PL: bbox 중심 기반 pan/tilt goal 계산

## 포함한 모델

- `models/tello_yolo.engine`: Junmo/Junyoung DJI Tello detector TensorRT 모델
- `models/tello_yolo.pt`: DJI Tello detector fallback
- `models/tello_audio_config.json`: Tello audio model config
- `models/tello_detector.keras`: Keras 원본
- `models/tello_detector.tflite`: Jetson 런타임용 TFLite 모델

## 빠른 시작

Ultra96가 USB Ethernet `192.168.3.1`로 잡힌 뒤:

```bash
cd /home/jetson/ultra_yubin
./tools/load_bitstream_only.sh
./tools/deploy_ultra96_ps_usb.sh
```

기본 배포는 SSH/USB 안정성 확인용입니다. PL AXI 경로를 실제로 켜려면 bitstream을 새로 로드한 뒤 아래처럼 lazy-open 상태로 브릿지를 띄우고, `PLPING`부터 한 단계씩 확인합니다.

```bash
ULTRA_YUBIN_NO_PL=0 ULTRA_YUBIN_SKIP_PL_LOAD=1 ULTRA_YUBIN_SKIP_CHECK=1 \
ULTRA_YUBIN_SKIP_PL_INIT=1 ULTRA_YUBIN_LAZY_PL_OPEN=1 ULTRA_YUBIN_RESTART=1 \
./tools/deploy_ultra96_ps_usb.sh

python3 tools/pl_bringup_check.py --host 192.168.3.1 --port 5016 --no-save
```

Jetson에서 YOLO bbox를 Ultra96로 보내려면:

```bash
cd /home/jetson/ultra_yubin
python3 jetson/jetson_node.py --camera 0 --model models/tello_yolo.engine
```

YOLO 미검출 시 ReSpeaker 오디오 fallback까지 켜려면:

```bash
python3 jetson/jetson_node.py --camera 0 --audio-fallback
```

## 안전 모드

기본 배포는 USB Ethernet이 끊기지 않도록 `ULTRA_YUBIN_NO_PL=1`, `ULTRA_YUBIN_SKIP_PL_LOAD=1`, `ULTRA_YUBIN_SKIP_CHECK=1`입니다. 즉 bitstream 로드는 `load_bitstream_only.sh`로 먼저 확인하고, 브릿지는 PL AXI 접근 없이 UDP/U2D2 경로부터 띄웁니다.

실제 PL/U2D2까지 강제로 사용하려면:

```bash
ULTRA_YUBIN_DRY_RUN=0 ULTRA_YUBIN_NO_PL=0 ULTRA_YUBIN_SKIP_PL_LOAD=1 \
ULTRA_YUBIN_SKIP_PL_INIT=1 ULTRA_YUBIN_LAZY_PL_OPEN=1 ULTRA_YUBIN_RESTART=1 \
./tools/deploy_ultra96_ps_usb.sh
```

PL 주소 또는 AXI 응답이 틀린 상태에서 `ULTRA_YUBIN_NO_PL=0`으로 `T/PLPING` 명령을 보내면 Ultra96 PS 버스가 멈출 수 있으므로, 먼저 no-pl 상태에서 UDP/U2D2 경로를 확인해야 합니다. `A` 오디오 fallback은 PS에서 처리합니다.
