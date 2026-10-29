"""Lance le centre de contrôle du drone (macOS, Linux, Windows) : flux vidéo GStreamer + dashboard MQTT.

Usage, depuis la racine du repo :
    .venv/bin/python ground/launch.py --broker <IP du Pi>      (simulation : --broker localhost)
Les autres arguments sont transmis à dashboard.py. La fenêtre vidéo s'ouvre seule ; elle est
fermée en quittant le dashboard.
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

VIDEO_PIPELINE = [
    "udpsrc", "port={port}", "!",
    "application/x-rtp,encoding-name=JPEG,payload=26", "!",
    "rtpjpegdepay", "!", "jpegdec", "!", "autovideosink",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=5000, help="port UDP du flux vidéo")
    args, dashboard_args = parser.parse_known_args()

    gst = shutil.which("gst-launch-1.0")
    video = None
    if gst is None:
        print("gst-launch-1.0 introuvable : pas de flux vidéo (installation : voir README.md).")
    else:
        pipeline = [part.format(port=args.port) for part in VIDEO_PIPELINE]
        video = subprocess.Popen([gst, "-q", *pipeline], stdout=subprocess.DEVNULL)

    dashboard = Path(__file__).resolve().parent / "dashboard.py"
    try:
        return subprocess.call([sys.executable, str(dashboard), *dashboard_args])
    except KeyboardInterrupt:
        return 0
    finally:
        if video is not None:
            video.terminate()


if __name__ == "__main__":
    sys.exit(main())
