# AI-readable branch

This branch is a lightweight code/documentation snapshot for review by AI tools.

The main demo branch contains runtime binaries and recordings that are too large
for many AI link readers. This branch intentionally omits:

- camera/audio recordings
- TensorRT/ONNX/PT/TFLite/Keras model binaries
- bitstream binaries
- benchmark logs
- presentation files

The runtime branch remains `main`. For implementation details, start with:

- `README.md`
- `run_demo_pl_drive.sh`
- `jetson/jetson_node.py`
- `jetson/src/audio_fallback.py`
- `jetson/src/ui/display.py`
- `jetson/src/control/ultra_yubin_motor.py`
- `hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v`
- `docs/system_overview_ko.md`
- `docs/telemetry_protocol.md`

