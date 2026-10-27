#!/bin/bash
# Remet le drone à sa position de départ sans relancer la simu :
# replace le modèle dans Gazebo, puis redémarre le SITL (EKF et paramètres repartent de zéro).
# À lancer depuis la racine du repo, drone posé et désarmé : ./sim/scripts/reset_drone.sh
set -e
# Pose de départ : même valeur que l'<include> du drone dans sim/worlds/voliere.sdf
POSE='name: "drone", position: {x: -3.75, y: -1.5, z: 0.195}, orientation: {w: 1}'

docker compose exec -T gz-server bash -lc "gz service -s /world/voliere/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '$POSE'"
docker compose restart sitl
echo "Drone au départ ; le SITL redémarre (prêt à armer dans ~30 s)."
