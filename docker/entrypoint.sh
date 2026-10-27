#!/bin/bash
# Charge l'environnement ROS 2 (et Gazebo dans les images sim/gui) puis exécute la commande.
. /etc/profile.d/droneload.sh
[ -f /etc/profile.d/droneload-sim.sh ] && . /etc/profile.d/droneload-sim.sh
exec "$@"
