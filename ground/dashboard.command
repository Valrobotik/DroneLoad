#!/bin/bash

# Récupère le dossier où se trouve ce script .command
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Lancement du centre de contrôle du Drone..."

# 1. Ouvre la fenêtre pour GStreamer
osascript -e 'tell application "Terminal" to do script "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink"'

# 2. Ouvre la fenêtre pour le Dashboard en pointant DIRECTEMENT sur le bon Python
# /!\ ATTENTION : Remplace '.venv' ci-dessous par le nom exact de ton dossier d'environnement si besoin
osascript -e 'tell application "Terminal" to do script "cd '"$DIR"' && bin/python dashboard.py"'