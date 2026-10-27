#!/bin/bash
# Raccourci macOS (double-clic dans le Finder) : lance ground/launch.py (flux vidéo + dashboard MQTT).
# Arguments optionnels transmis au dashboard, ex : ./dashboard.command --broker 192.168.2.3

# Racine du repo (ce script est dans ground/)
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$ROOT" && exec .venv/bin/python ground/launch.py "$@"
