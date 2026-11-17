#!/bin/bash
# Remet le drone à sa position de départ sans relancer la simu :
# replace le modèle dans Gazebo, puis redémarre le SITL (EKF et paramètres repartent de zéro).
# Drone posé et désarmé : ./sim/scripts/reset_drone.sh (depuis la racine) ou ./scripts/reset_drone.sh (depuis sim/)
# Argument facultatif : cap de départ en degrés (sens trigonométrique, 0 = vers l'est comme dans voliere.sdf),
# ex. ./sim/scripts/reset_drone.sh 90 (drone tourné vers le nord, pour tests/simu/uwb.py).
set -e
cd "$(dirname "$0")/.."   # dossier du compose.yml
# Pose de départ : même valeur que l'<include> du drone dans sim/worlds/voliere.sdf
CAP=${1:-0}
QUAT=$(python3 -c "import math; a = math.radians($CAP) / 2; print(f'z: {math.sin(a):.6f}, w: {math.cos(a):.6f}')")
POSE="name: \"drone\", position: {x: -3.75, y: -1.5, z: 0.195}, orientation: {$QUAT}"

docker compose exec -T gz-server bash -lc "gz service -s /world/voliere/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '$POSE'"
docker compose restart sitl
echo "Drone au départ ; le SITL redémarre (prêt à armer dans ~30 s)."
