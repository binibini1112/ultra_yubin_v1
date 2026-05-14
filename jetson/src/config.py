import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODELS_DIR = os.path.join(ROOT, "models")

# Camera settings copied from /home/jetson/jh and kept local to ultra_yubin.
CAMERA_ID = os.getenv("CAMERA_ID", "auto")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "1280"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "720"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))
CAMERA_FOURCC = os.getenv("CAMERA_FOURCC", "MJPG")
CAMERA_FLIP_VERTICAL = os.getenv("CAMERA_FLIP_VERTICAL", "1") == "1"
CAMERA_FLIP_HORIZONTAL = os.getenv("CAMERA_FLIP_HORIZONTAL", "0") == "1"
CAMERA_CENTER_CROP = os.getenv("CAMERA_CENTER_CROP", "0") == "1"
CAMERA_CROP_SCALE = float(os.getenv("CAMERA_CROP_SCALE", "0.70"))
CAMERA_APPLY_GLARE_DEFAULTS = os.getenv("CAMERA_APPLY_GLARE_DEFAULTS", "0") == "1"
CAMERA_GLARE_EXPOSURE_STEP = int(os.getenv("CAMERA_GLARE_EXPOSURE_STEP", "10"))
CAMERA_GLARE_GAIN_STEP = int(os.getenv("CAMERA_GLARE_GAIN_STEP", "4"))
CAMERA_GLARE_EXPOSURE_MIN = int(os.getenv("CAMERA_GLARE_EXPOSURE_MIN", "3"))
CAMERA_GLARE_EXPOSURE_MAX = int(os.getenv("CAMERA_GLARE_EXPOSURE_MAX", "2047"))
CAMERA_GLARE_GAIN_MIN = int(os.getenv("CAMERA_GLARE_GAIN_MIN", "0"))
CAMERA_GLARE_GAIN_MAX = int(os.getenv("CAMERA_GLARE_GAIN_MAX", "255"))
CAMERA_GLARE_RESET_EXPOSURE = int(os.getenv("CAMERA_GLARE_RESET_EXPOSURE", "43"))
CAMERA_GLARE_RESET_GAIN = int(os.getenv("CAMERA_GLARE_RESET_GAIN", "16"))
CAMERA_GLARE_RESET_BACKLIGHT = int(os.getenv("CAMERA_GLARE_RESET_BACKLIGHT", "0"))

# Tello/drone YOLO model copied from /home/jetson/jh.
DRONE_ENGINE = os.path.join(MODELS_DIR, "drone_yolov8s_fromjunmo.engine")
DRONE_PT = os.path.join(MODELS_DIR, "drone_yolov8s_fromjunmo.pt")
LEGACY_ENGINE = os.path.join(MODELS_DIR, "tello_yolo.engine")
LEGACY_PT = os.path.join(MODELS_DIR, "tello_yolo.pt")
if os.path.exists(DRONE_ENGINE):
    YOLO_MODEL_PATH = DRONE_ENGINE
elif os.path.exists(DRONE_PT):
    YOLO_MODEL_PATH = DRONE_PT
elif os.path.exists(LEGACY_ENGINE):
    YOLO_MODEL_PATH = LEGACY_ENGINE
else:
    YOLO_MODEL_PATH = LEGACY_PT
YOLO_MODEL = YOLO_MODEL_PATH
YOLO_DEVICE = os.getenv("YOLO_DEVICE", "cuda:0")
YOLO_CONF = float(os.getenv("YOLO_CONF", "0.35"))
YOLO_IMGSZ = int(os.getenv("YOLO_IMGSZ", "640"))
AERIAL_TARGET_CLASSES = [0]

# jh HUD settings.
UI_RENDER_INTERVAL_FRAMES = int(os.getenv("UI_RENDER_INTERVAL_FRAMES", "2"))
UI_CAMERA_EVERY_FRAME = os.getenv("UI_CAMERA_EVERY_FRAME", "1") == "1"
UI_MINIMAL = os.getenv("UI_MINIMAL", "1") == "1"
UI_FULLSCREEN = os.getenv("UI_FULLSCREEN", "1") == "1"
UI_SCREEN_WIDTH = int(os.getenv("UI_SCREEN_WIDTH", "0"))
UI_SCREEN_HEIGHT = int(os.getenv("UI_SCREEN_HEIGHT", "0"))
UI_PANEL_WIDTH = int(os.getenv("UI_PANEL_WIDTH", "360"))
UI_CAMERA_FIT_MODE = os.getenv("UI_CAMERA_FIT_MODE", "stretch")

# Tracking / state-machine parameters used by copied jh modules.
DETECT_CONFIRM_FRAMES = int(os.getenv("DETECT_CONFIRM_FRAMES", "4"))
LOCK_HOLD_SECONDS = float(os.getenv("LOCK_HOLD_SECONDS", "2.0"))
LOCK_AIM_THRESHOLD = float(os.getenv("LOCK_AIM_THRESHOLD", "80.0"))
NEUTRALIZED_HOLD_SEC = float(os.getenv("NEUTRALIZED_HOLD_SEC", "4.0"))
DRONE_LOST_RESET_SEC = float(os.getenv("DRONE_LOST_RESET_SEC", "7.0"))
TRACK_STICKY_MAX_DIST_PX = float(os.getenv("TRACK_STICKY_MAX_DIST_PX", "10000"))
TRACK_PREFER_SAME_ID = os.getenv("TRACK_PREFER_SAME_ID", "0") == "1"
TRACK_SAME_ID_MAX_DIST_PX = float(os.getenv("TRACK_SAME_ID_MAX_DIST_PX", "320"))
TRACK_REACQUIRE_AFTER_ABSENT_FRAMES = int(os.getenv("TRACK_REACQUIRE_AFTER_ABSENT_FRAMES", "6"))
TRACK_REACQUIRE_MIN_CONF = float(os.getenv("TRACK_REACQUIRE_MIN_CONF", "0.75"))
TRACK_HOLD_LAST_FRAMES = int(os.getenv("TRACK_HOLD_LAST_FRAMES", "0"))
TRACK_TARGET_MIN_CONF = float(os.getenv("TRACK_TARGET_MIN_CONF", "0.60"))
TRACK_TARGET_MIN_AREA = int(os.getenv("TRACK_TARGET_MIN_AREA", "1500"))
TRACK_TARGET_MIN_W = int(os.getenv("TRACK_TARGET_MIN_W", "35"))
TRACK_TARGET_MIN_H = int(os.getenv("TRACK_TARGET_MIN_H", "25"))
TRACK_TARGET_MIN_ASPECT = float(os.getenv("TRACK_TARGET_MIN_ASPECT", "0.0"))
TRACK_TARGET_MAX_ASPECT = float(os.getenv("TRACK_TARGET_MAX_ASPECT", "10000.0"))
TRACK_MOTOR_MIN_CONF = float(os.getenv("TRACK_MOTOR_MIN_CONF", "0.65"))
TRACK_MOTOR_EDGE_MARGIN_X = int(os.getenv("TRACK_MOTOR_EDGE_MARGIN_X", "80"))
TRACK_MOTOR_EDGE_MARGIN_Y = int(os.getenv("TRACK_MOTOR_EDGE_MARGIN_Y", "45"))

# Ultra96 bridge.
ULTRA_YUBIN_HOST = os.getenv("ULTRA_YUBIN_HOST", "192.168.3.1")
ULTRA_YUBIN_PORT = int(os.getenv("ULTRA_YUBIN_PORT", "5016"))
ULTRA_YUBIN_TIMEOUT_SEC = float(os.getenv("ULTRA_YUBIN_TIMEOUT_SEC", "0.08"))

# Motor display limits.
PAN_MIN = int(os.getenv("PAN_MIN", "0"))
PAN_MAX = int(os.getenv("PAN_MAX", "4095"))
TILT_MIN = int(os.getenv("TILT_MIN", "2340"))
TILT_MAX = int(os.getenv("TILT_MAX", "3048"))
PAN_DIR = int(os.getenv("PAN_DIR", "1"))
TILT_DIR = int(os.getenv("TILT_DIR", "1"))

# Tello audio fallback. When enabled, this is used only while YOLO has no target.
TELLO_AUDIO_FALLBACK = os.getenv("TELLO_AUDIO_FALLBACK", "0") == "1"
TELLO_AUDIO_MODE = os.getenv("TELLO_AUDIO_MODE", "doa").lower()
TELLO_AUDIO_CONFIG = os.path.join(MODELS_DIR, "tello_audio_config.json")
TELLO_AUDIO_TFLITE = os.path.join(MODELS_DIR, "tello_detector.tflite")
TELLO_AUDIO_KERAS = os.path.join(MODELS_DIR, "tello_detector.keras")
TELLO_AUDIO_THRESHOLD = float(os.getenv("TELLO_AUDIO_THRESHOLD", "0.70"))
TELLO_AUDIO_CONSECUTIVE = int(os.getenv("TELLO_AUDIO_CONSECUTIVE", "2"))
TELLO_AUDIO_MIN_RMS = float(os.getenv("TELLO_AUDIO_MIN_RMS", "0.008"))
TELLO_AUDIO_ALSA_DEVICE = os.getenv("TELLO_AUDIO_ALSA_DEVICE", "auto")
TELLO_AUDIO_CHANNELS = int(os.getenv("TELLO_AUDIO_CHANNELS", "4"))
TELLO_AUDIO_DOA_OFFSET = int(os.getenv("TELLO_AUDIO_DOA_OFFSET", "0"))
TELLO_AUDIO_MAX_AGE_SEC = float(os.getenv("TELLO_AUDIO_MAX_AGE_SEC", "1.5"))
TELLO_AUDIO_CONTROL_PERIOD_SEC = float(os.getenv("TELLO_AUDIO_CONTROL_PERIOD_SEC", "0.35"))
TELLO_AUDIO_KEEPALIVE_SEC = float(os.getenv("TELLO_AUDIO_KEEPALIVE_SEC", "1.2"))
TELLO_AUDIO_MIN_CHANGE_DEG = float(os.getenv("TELLO_AUDIO_MIN_CHANGE_DEG", "12"))
TELLO_AUDIO_CLAMP_DEG = float(os.getenv("TELLO_AUDIO_CLAMP_DEG", "45"))
