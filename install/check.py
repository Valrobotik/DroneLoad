#!/usr/bin/env python3
"""Liste ce qui a été installé sur cette machine sans passer par Ansible (install/).

Compare l'état réel aux listes du repo :
  - paquets apt installés à la main, paquets pip du venv  -> install/packages.yml
  - pip hors venv, autres venvs (ex. .venv du repo), fichiers dans /usr/local/bin (installations à la main)

Usage : python3 install/check.py [--mqtt]
  --mqtt : publie aussi le résultat sur drone/logs (affiché par le dashboard)
Code de sortie 0 si tout est déclaré, 1 sinon. Lancé toutes les heures sur la Pi (timer systemd).
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import yaml

INSTALL_DIR = Path(__file__).resolve().parent
BASELINE = Path("/var/lib/droneload/apt-base.txt")
VENV = Path("/opt/droneload_venv")
# Installés hors des listes mais par la procédure elle-même (apply.sh, rôle common)
APT_PROCEDURE = {"ansible", "ansible-core", "python3-apt", "ros2-apt-source"}
PIP_OUTILS = {"pip", "setuptools", "wheel"}
PIP_PROCEDURE = {"torch", "torchvision"}   # version CPU, installée à part par le rôle pi


def normalise(nom):
    return re.sub(r"[-_.]+", "-", nom).lower()


def declares(type_paquet):
    """Paquets apt ou pip de packages.yml, toutes machines confondues."""
    distro = yaml.safe_load((INSTALL_DIR / "roles/common/defaults/main.yml").read_text())["ros_distro"]
    sections = yaml.safe_load((INSTALL_DIR / "packages.yml").read_text())
    return [p.replace("{{ ros_distro }}", distro)
            for section in sections.values() for p in section.get(type_paquet) or []]


def apt_hors_ansible():
    if not BASELINE.exists():
        return []
    installes = subprocess.run(["apt-mark", "showmanual"], capture_output=True, text=True).stdout.split()
    connus = set(BASELINE.read_text().split()) | set(declares("apt")) | APT_PROCEDURE
    return sorted(p for p in installes if p.split(":")[0] not in connus)


def pip_declares():
    """(nom, version figée ou "") des paquets pip de packages.yml et du rôle pi."""
    noms = {(nom, "") for nom in PIP_PROCEDURE}
    for paquet in declares("pip"):
        m = re.match(r"([A-Za-z0-9_.-]+)\s*(?:\[[^]]*\])?\s*(?:==\s*(\S+))?", paquet)
        noms.add((normalise(m.group(1)), m.group(2) or ""))
    return noms


# Exécuté par le Python du venv : dépendances (récursives) des paquets déclarés
DEPENDANCES = r"""
import json, re, sys
from importlib.metadata import distribution, PackageNotFoundError
norm = lambda n: re.sub(r"[-_.]+", "-", n).lower()
vus, pile = set(), list(json.loads(sys.argv[1]))
while pile:
    nom = pile.pop()
    if nom in vus:
        continue
    vus.add(nom)
    try:
        requis = distribution(nom).requires or []
    except PackageNotFoundError:
        continue
    for r in requis:
        if "extra ==" not in r:
            pile.append(norm(re.split(r"[<>=!~\[;( ]", r.strip())[0]))
print(json.dumps(sorted(vus)))
"""


def pip_hors_ansible():
    python = VENV / "bin" / "python"
    if not python.exists():
        return [], []
    declares = pip_declares()
    sortie = subprocess.run([python, "-m", "pip", "list", "--local", "--format=json"],
                            capture_output=True, text=True).stdout
    installes = {normalise(p["name"]): p["version"] for p in json.loads(sortie or "[]")}
    autorises = set(json.loads(subprocess.run(
        [python, "-c", DEPENDANCES, json.dumps([n for n, _ in declares])],
        capture_output=True, text=True).stdout or "[]"))
    en_trop = sorted(n for n in installes if n not in autorises | PIP_OUTILS)
    versions = sorted(f"{n} {installes[n]} (attendu {v})" for n, v in declares
                      if v and n in installes and installes[n] != v)
    return en_trop, versions


def hors_gestionnaires():
    """pip hors venv (sudo pip, pip --user) et exécutables copiés dans /usr/local/bin."""
    trouves = []
    for motif in ["/usr/local/lib/python3*/dist-packages/*.dist-info",
                  "/home/*/.local/lib/python3*/site-packages/*.dist-info",
                  "/root/.local/lib/python3*/site-packages/*.dist-info"]:
        trouves += [f"pip hors venv : {p}" for p in glob.glob(motif)]
    if os.path.isdir("/usr/local/bin"):
        trouves += [f"/usr/local/bin/{f}" for f in sorted(os.listdir("/usr/local/bin"))]
    trouves += [f"venv hors Ansible : {v} (utiliser {VENV})" for v in autres_venvs()]
    return trouves


def autres_venvs(profondeur_max=4):
    """Venvs (dossiers avec pyvenv.cfg) autres que celui d'Ansible, ex. un .venv dans le repo."""
    trouves = []
    for racine in ["/home", "/root", "/opt", "/srv"]:
        if not os.path.isdir(racine):
            continue
        base = racine.rstrip("/").count("/")
        for dossier, sous_dossiers, fichiers in os.walk(racine):
            if "pyvenv.cfg" in fichiers:
                if Path(dossier) != VENV:
                    trouves.append(dossier)
                sous_dossiers.clear()   # pas de recherche à l'intérieur d'un venv
            elif dossier.count("/") - base >= profondeur_max:
                sous_dossiers.clear()
            else:
                sous_dossiers[:] = [d for d in sous_dossiers if d not in {".git", ".cache", "node_modules"}]
    return sorted(trouves)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mqtt", action="store_true", help="publie le résultat sur drone/logs")
    args = parser.parse_args()

    apt = apt_hors_ansible()
    pip, versions = pip_hors_ansible()
    autres = hors_gestionnaires()

    lignes = []
    if not BASELINE.exists():
        lignes.append(f"{BASELINE} absent : lancer d'abord ./install/apply.sh (les paquets apt ne sont pas vérifiés)")
    if apt:
        lignes.append("apt, à ajouter dans install/packages.yml : " + " ".join(apt))
    if pip:
        lignes.append("pip (venv), à ajouter dans install/packages.yml : " + " ".join(pip))
    if versions:
        lignes.append("pip, version différente de install/packages.yml : " + ", ".join(versions))
    lignes += autres

    if not lignes:
        print("OK : tout ce qui est installé est déclaré dans install/.")
        return 0
    print("Installé hors Ansible (voir install/README.md) :")
    for ligne in lignes:
        print("  -", ligne)
    if args.mqtt:
        message = "[install] installé hors Ansible : " + " | ".join(lignes)
        subprocess.run(["mosquitto_pub", "-t", "drone/logs", "-m", message], check=False)
    return 1


if __name__ == "__main__":
    sys.exit(main())
