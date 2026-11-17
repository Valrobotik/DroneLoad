"""Lance le centre de contrôle du drone (macOS, Linux, Windows) : flux vidéo GStreamer, carte de la volière et dashboard MQTT.

Usage, depuis la racine du repo :
    .venv/bin/python ground/launch.py --broker <IP du Pi>      (simulation : --broker localhost --sim)
--sim, --taille, --depart, --rotation et --repere vont à la carte (ground/carte.py), --broker à la carte
et au dashboard, les autres arguments au dashboard. Les fenêtres vidéo et carte s'ouvrent seules ;
elles sont fermées en quittant le dashboard (--sans-carte : pas de carte).
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
GROUND = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=5000, help="port UDP du flux vidéo")
    parser.add_argument("--broker", default="192.168.2.3", help="IP du Raspberry Pi (broker MQTT)")
    parser.add_argument("--sans-carte", action="store_true", help="ne pas ouvrir la carte de la volière")
    carte = parser.add_argument_group("carte (voir ground/carte.py)")
    carte.add_argument("--sim", action="store_true")
    carte.add_argument("--taille")
    carte.add_argument("--depart")
    carte.add_argument("--rotation")
    carte.add_argument("--repere")
    args, dashboard_args = parser.parse_known_args()

    processes = []
    gst = shutil.which("gst-launch-1.0")
    if gst is None:
        print("gst-launch-1.0 introuvable : pas de flux vidéo (installation : voir README.md).")
    else:
        pipeline = [part.format(port=args.port) for part in VIDEO_PIPELINE]
        processes.append(subprocess.Popen([gst, "-q", *pipeline], stdout=subprocess.DEVNULL))

    if not args.sans_carte:
        carte_args = ["--broker", args.broker] + (["--sim"] if args.sim else [])
        for option in ("taille", "depart", "rotation", "repere"):
            if getattr(args, option) is not None:
                carte_args += [f"--{option}", getattr(args, option)]
        # Sortie de la carte masquée : elle écrirait par-dessus l'interface du dashboard
        processes.append(subprocess.Popen([sys.executable, str(GROUND / "carte.py"), *carte_args],
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

    try:
        return subprocess.call([sys.executable, str(GROUND / "dashboard.py"), "--broker", args.broker, *dashboard_args])
    except KeyboardInterrupt:
        return 0
    finally:
        for process in processes:
            process.terminate()


if __name__ == "__main__":
    sys.exit(main())
