#!/bin/bash
# Lance le centre de contrôle du drone (macOS) : flux vidéo + dashboard MQTT.
# Arguments optionnels transmis au dashboard, ex : ./dashboard.command --broker 192.168.2.3

# Racine du repo (ce script est dans ground/)
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

echo "Lancement du centre de contrôle du Drone..."

# 1. Ouvre la fenêtre pour GStreamer
osascript -e 'tell application "Terminal" to do script "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink"'

# 2. Ouvre la fenêtre pour le Dashboard avec le Python du venv .venv/
osascript -e 'tell application "Terminal" to do script "cd '"$ROOT"' && .venv/bin/python ground/dashboard.py '"$*"'"'
