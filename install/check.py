#!/usr/bin/env python3
"""Liste ce qui a été installé sur cette machine sans passer par Ansible (install/).

Compare l'état réel aux listes du repo :
  - paquets apt installés à la main, paquets pip du venv  -> install/packages.yml
  - pip hors venv, autres venvs (ex. .venv du repo), fichiers dans /usr/local/bin (installations à la main)
  - sur la Pi, configuration du matériel (Pixhawk, caméra, GPIO), du bouton d'alimentation et des mises à jour
  - ce qui se lance tout seul hors d'install/ (services, crontabs, rc.local, .bashrc...) : un vieux script
    relancé à chaque démarrage survivrait aux redémarrages et prendrait le port de la Pixhawk

Usage : python3 install/check.py [--mqtt]
  --mqtt : publie aussi le résultat sur drone/logs (affiché par le dashboard)
Code de sortie 0 si tout est déclaré, 1 sinon. Lancé toutes les heures sur la Pi (timer systemd).
"""
import argparse
import glob
import grp
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
PIP_PROCEDURE = {"torch", "torchvision"}   # version CPU, installée à part par le rôle common
BOOT = Path("/boot/firmware")              # carte de démarrage de la Pi (absente ailleurs)
PORT_PIXHAWK = Path("/dev/ttyAMA0")
REGLES_UDEV = Path("/etc/udev/rules.d/99-droneload.rules")
GROUPES = {"dialout": "port de la Pixhawk", "video": "caméra", "gpio": "GPIO du crochet"}
LOGIND = Path("/etc/systemd/logind.conf.d/droneload.conf")
SERVICES_INSTALL = {"droneload.service", "droneload-check.service", "droneload-check.timer"}   # rôle pi
# Commande qui lance un programme : python script.py, ros2 launch/run, mavproxy, nohup, tmux/screen détachés
LANCEMENT = re.compile(r"\bpython[0-9.]*\s+\S+\.py|\bros2\s+(launch|run)\b|\bmavproxy|\bnohup\b|\b(tmux|screen)\b.*\s-d")


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


def config_pi():
    """Écarts avec la configuration du rôle pi (Pixhawk, caméra, GPIO, mises à jour automatiques)."""
    if not (BOOT / "config.txt").exists():
        return []
    ecarts = []
    config = (BOOT / "config.txt").read_text()
    if not re.search(r"^dtparam=uart0=on", config, re.M):
        ecarts.append("config.txt : dtparam=uart0=on absent (UART de la Pixhawk désactivé)")
    if not re.search(r"^camera_auto_detect=1", config, re.M):
        ecarts.append("config.txt : camera_auto_detect=1 absent (caméra CSI non détectée)")
    if re.search(r"console=(serial0|ttyAMA0)", (BOOT / "cmdline.txt").read_text()):
        ecarts.append("cmdline.txt : console Linux sur le port de la Pixhawk")
    if not PORT_PIXHAWK.exists():
        ecarts.append(f"{PORT_PIXHAWK} absent (redémarrer la Pi après ./install/apply.sh)")
    if not LOGIND.exists():
        ecarts.append(f"{LOGIND} absent (bouton d'alimentation : arrêt propre non garanti)")
    if not REGLES_UDEV.exists():
        ecarts.append(f"{REGLES_UDEV} absent (GPIO et caméra réservés à root)")
    utilisateur = INSTALL_DIR.parent.owner()   # celui qui lance le code du drone (rôle pi)
    for groupe, usage in GROUPES.items():
        try:
            membres = grp.getgrnam(groupe).gr_mem
        except KeyError:
            membres = []
        if utilisateur not in membres:
            ecarts.append(f"{utilisateur} hors du groupe {groupe} (pas d'accès : {usage})")
    apt = subprocess.run(["apt-config", "dump"], capture_output=True, text=True).stdout
    if re.search(r'^APT::Periodic::Unattended-Upgrade "[1-9]', apt, re.M):
        ecarts.append("mises à jour automatiques actives (unattended-upgrades)")
    return ecarts


def lignes_qui_lancent(fichier):
    """Lignes non commentées d'un fichier de démarrage qui lancent un programme."""
    try:
        texte = Path(fichier).read_text(errors="replace")
    except OSError:   # crontabs illisibles hors root : vérifiées par le timer (root)
        return []
    return [f"{fichier} : {ligne.strip()[:100]}" for ligne in texte.splitlines()
            if not ligne.strip().startswith("#") and LANCEMENT.search(ligne)]


def demarrage_auto():
    """Ce qui se lance tout seul (démarrage, connexion SSH, cron) sans venir d'install/."""
    trouves = []
    # Services écrits à la main : fichiers dans /etc/systemd/system (ceux des paquets sont dans /usr/lib)
    for unite in sorted(glob.glob("/etc/systemd/system/*.service") + glob.glob("/etc/systemd/system/*.timer")
                        + glob.glob("/home/*/.config/systemd/user/*.service")):
        nom = Path(unite).name
        # snap.* : services des snaps d'Ubuntu (lxd...), écrits là par snapd
        if os.path.isfile(unite) and not os.path.islink(unite) and nom not in SERVICES_INSTALL \
                and not nom.startswith("snap."):
            trouves.append(f"service hors install/ : {unite}")
    if os.access("/etc/rc.local", os.X_OK):
        trouves.append("/etc/rc.local exécutable (lancé au démarrage)")
    fichiers = ["/etc/crontab", "/etc/bash.bashrc", "/etc/profile"]
    fichiers += glob.glob("/etc/cron.d/*") + glob.glob("/var/spool/cron/crontabs/*")
    fichiers += [f for f in glob.glob("/etc/profile.d/*") if not f.endswith("/droneload.sh")]
    for maison in glob.glob("/home/*") + ["/root"]:
        fichiers += [f"{maison}/{nom}" for nom in (".bashrc", ".profile", ".bash_profile", ".bash_login")]
    for fichier in fichiers:
        trouves += [f"lancement automatique : {ligne}" for ligne in lignes_qui_lancent(fichier)]
    return trouves


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mqtt", action="store_true", help="publie le résultat sur drone/logs")
    args = parser.parse_args()

    apt = apt_hors_ansible()
    pip, versions = pip_hors_ansible()
    autres = hors_gestionnaires()
    pi = config_pi()
    auto = demarrage_auto()

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
    lignes += [f"configuration de la Pi modifiée, relancer ./install/apply.sh : {e}" for e in pi]
    lignes += auto

    if not lignes:
        print("OK : tout ce qui est installé est déclaré dans install/.")
        return 0
    print("Écarts avec install/ (voir install/README.md) :")
    for ligne in lignes:
        print("  -", ligne)
    if args.mqtt:
        message = "[install] écarts avec install/ : " + " | ".join(lignes)
        subprocess.run(["mosquitto_pub", "-t", "drone/logs", "-m", message], check=False)
    return 1


if __name__ == "__main__":
    sys.exit(main())
