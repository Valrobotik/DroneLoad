#!/bin/bash
# Raccourci macOS (double-clic dans le Finder) : lance ground/launch.py (flux vidéo, carte, dashboard MQTT).
# Arguments optionnels transmis à launch.py, ex : ./dashboard.command --broker 192.168.2.3 --depart -3.45,-1.25

# Racine du repo (ce script est dans ground/)
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$ROOT" && exec .venv/bin/python ground/launch.py "$@"
