import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODELS_DIR = os.path.join(ROOT, "models")

YOLO_ENGINE = os.path.join(MODELS_DIR, "tello_yolo.engine")
YOLO_PT = os.path.join(MODELS_DIR, "tello_yolo.pt")
YOLO_MODEL = YOLO_ENGINE if os.path.exists(YOLO_ENGINE) else YOLO_PT
YOLO_DEVICE = os.getenv("YOLO_DEVICE", "cpu")
YOLO_CONF = float(os.getenv("YOLO_CONF", "0.45"))
YOLO_IMGSZ = int(os.getenv("YOLO_IMGSZ", "320"))
AERIAL_TARGET_CLASSES = None

TELLO_AUDIO_CONFIG = os.path.join(MODELS_DIR, "tello_audio_config.json")
TELLO_AUDIO_TFLITE = os.path.join(MODELS_DIR, "tello_detector.tflite")
TELLO_AUDIO_KERAS = os.path.join(MODELS_DIR, "tello_detector.keras")
TELLO_AUDIO_THRESHOLD = float(os.getenv("TELLO_AUDIO_THRESHOLD", "0.70"))
TELLO_AUDIO_CONSECUTIVE = int(os.getenv("TELLO_AUDIO_CONSECUTIVE", "2"))
TELLO_AUDIO_MIN_RMS = float(os.getenv("TELLO_AUDIO_MIN_RMS", "0.008"))
TELLO_AUDIO_ALSA_DEVICE = os.getenv("TELLO_AUDIO_ALSA_DEVICE", "plughw:CARD=ArrayUAC10,DEV=0")
TELLO_AUDIO_CHANNELS = int(os.getenv("TELLO_AUDIO_CHANNELS", "6"))
TELLO_AUDIO_DOA_OFFSET = int(os.getenv("TELLO_AUDIO_DOA_OFFSET", "0"))

ULTRA_YUBIN_HOST = os.getenv("ULTRA_YUBIN_HOST", "192.168.3.1")
ULTRA_YUBIN_PORT = int(os.getenv("ULTRA_YUBIN_PORT", "5016"))
ULTRA_YUBIN_TIMEOUT_SEC = float(os.getenv("ULTRA_YUBIN_TIMEOUT_SEC", "0.08"))

CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))
