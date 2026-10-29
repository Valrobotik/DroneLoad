"""Paramètres du drone regroupés en un seul endroit."""
from pathlib import Path

# Racine du repo (drone/ est un sous-dossier)
ROOT_DIR = Path(__file__).resolve().parent.parent

# --- Réseau ---
PC_IP = "192.168.31.181"   # PC de pilotage, destination du flux vidéo (surcharge : --pc-ip)
VIDEO_PORT = 5000
BROKER_IP = "127.0.0.1"    # broker MQTT local sur le Pi
BROKER_PORT = 1883

# --- MAVLink ---
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 921600
SITL_URL = "udp:127.0.0.1:14550"

# --- Caméra ---
WIDTH = 640
HEIGHT = 480
FPS = 15

# --- IA ---
MODEL_PATH = ROOT_DIR / "models" / "best_ncnn_model"   # export NCNN (Pi)
# MODEL_PATH = ROOT_DIR / "models" / "best.pt"         # modèle PyTorch
