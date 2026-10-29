#!/bin/sh
# Applique install/ (Ansible) sur cette machine. À relancer après chaque ajout dans
# install/packages.yml ou un requirements.txt (voir install/README.md).
#
#   ./install/apply.sh          Raspberry Pi détectée automatiquement, sinon ROS 2 seul
#   ./install/apply.sh pi       force le rôle pi
#   ./install/apply.sh sim      PC Linux avec Gazebo natif
# Les arguments suivants sont passés à ansible-playbook (ex. --check --diff pour un essai à blanc).
set -e
INSTALL_DIR=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$INSTALL_DIR")

sudo=sudo
[ "$(id -u)" = 0 ] && sudo=""

cible=""
case "$1" in
    pi|sim) cible=$1; shift ;;
esac
if [ -z "$cible" ] && grep -qs "Raspberry Pi" /proc/device-tree/model; then
    cible=pi
fi

options="-e droneload_repo=$REPO"
case "$cible" in
    pi)  options="$options -e with_pi=true" ;;
    sim) options="$options -e with_sim=true" ;;
esac

if ! command -v ansible-playbook >/dev/null; then
    echo "Installation d'Ansible..."
    $sudo apt-get update && $sudo apt-get install -y ansible-core
fi

echo "Installation DroneLoad (${cible:-ros2}) depuis $INSTALL_DIR"
cd "$INSTALL_DIR"
# shellcheck disable=SC2086
$sudo ansible-playbook -c local -i localhost, site.yml $options "$@"
python3 "$INSTALL_DIR/check.py" || true
