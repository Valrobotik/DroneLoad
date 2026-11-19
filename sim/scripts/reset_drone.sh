#!/bin/bash
# Remet le drone à sa position de départ sans relancer la simu :
# replace le modèle dans Gazebo, puis redémarre le SITL (EKF et paramètres repartent de zéro).
# Drone posé et désarmé : ./sim/scripts/reset_drone.sh (depuis la racine) ou ./scripts/reset_drone.sh (depuis sim/)
# Argument facultatif : cap de départ en degrés (sens trigonométrique, 0 = vers l'est), à la place du cap du monde,
# ex. ./sim/scripts/reset_drone.sh 90 (drone tourné vers le nord, pour tests/simu/uwb.py).
set -e
cd "$(dirname "$0")/.."   # dossier du compose.yml
# Pose de départ lue dans le monde (<include> du drone de worlds/voliere.sdf) : suit la volière, sans copie ici.
# Cap passé en argument : remplace le lacet du monde
POSE=$("${PYTHON:-python3}" - worlds/voliere.sdf "${1:-}" <<'EOF'
import math, sys
import xml.etree.ElementTree as ET
for inclus in ET.parse(sys.argv[1]).getroot().find('world').findall('include'):
    if inclus.findtext('name') == 'drone':
        x, y, z, _, _, lacet = (float(v) for v in inclus.findtext('pose').split())
        if sys.argv[2]:
            lacet = math.radians(float(sys.argv[2]))
        print(f'name: "drone", position: {{x: {x}, y: {y}, z: {z}}}, '
              f'orientation: {{z: {math.sin(lacet / 2)}, w: {math.cos(lacet / 2)}}}')
        break
else:
    sys.exit('pas de drone dans ' + sys.argv[1])
EOF
)

docker compose exec -T gz-server bash -lc "gz service -s /world/voliere/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '$POSE'"
docker compose restart sitl
echo "Drone au départ ; le SITL redémarre (prêt à armer dans ~30 s)."
