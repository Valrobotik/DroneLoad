# install/ : installation des ordinateurs (Ansible)

Tout ce qui est installé sur la Raspberry Pi, dans les images Docker de la simulation et sur un PC Linux passe par ce dossier. Une carte SD neuve doit pouvoir redonner une Pi fonctionnelle avec une seule commande.

| Fichier | Contenu |
|---|---|
| `packages.yml` | **le seul fichier à modifier** : paquets apt et pip, par machine (`commun`, `pi`, `sim`) |
| `site.yml`, `roles/` | le playbook : `common` (ROS 2, MAVROS, venv), `sim` (Gazebo), `pi` (broker MQTT, caméra, servos) |
| `apply.sh` | lance le playbook sur la machine courante |
| `check.py` | liste ce qui a été installé sans passer par ici |

## Règle : on n'installe jamais rien à la main sur la Pi

Pas de `sudo apt install`, de `pip install` ni de logiciel compilé à la main. Ce qui n'est pas dans `install/` sera perdu à la prochaine réinstallation, et le drone ne marchera plus le jour où il faudra changer la carte SD.

Pour ajouter un paquet :

1. Dans `install/packages.yml`, ajouter une ligne sous la machine concernée (`pi`, `commun` pour toutes, ou `sim`), dans `apt` (paquet système) ou `pip` (paquet Python, version figée `nom==x.y.z`) :
   ```yaml
   pi:
     apt:
       - i2c-tools
     pip:
       - smbus2==0.5.1
   ```
2. Sur la Pi, depuis le repo : `./install/apply.sh`. On peut relancer cette commande autant de fois qu'on veut : elle ne refait que ce qui manque. Pour la simu : `cd sim && docker compose build`.
3. Committer `packages.yml` avec le code qui en a besoin, dans la même PR.

Une installation qui n'est pas un simple paquet (dépôt apt, compilation, fichier de config) se fait par une tâche dans `roles/<rôle>/tasks/main.yml`.

Pour un essai rapide, on peut installer à la main, mais il faut ensuite faire les étapes 1 à 3, ou désinstaller.

## Vérification : `check.py`

```bash
python3 install/check.py
```

Le script affiche ce qui a été installé sans passer par `install/` :
- paquets apt installés à la main ;
- paquets du venv absents de `packages.yml`, ou dont la version diffère de celle figée ;
- `pip` hors du venv (`sudo pip`, `pip --user`) ;
- tout autre venv que `/opt/droneload_venv` (par exemple un `.venv` créé dans le repo), dont les paquets échapperaient au suivi ;
- fichiers dans `/usr/local/bin`.

Sur la Pi, il tourne toutes les heures (timer systemd `droneload-check`) et publie les écarts sur `drone/logs`, ce qui les affiche dans le dashboard. Pour voir son dernier résultat : `journalctl -u droneload-check`.

Les paquets apt présents avant la première exécution d'`apply.sh` (le système de base) sont mémorisés dans `/var/lib/droneload/apt-base.txt` et ne sont pas signalés. Il faut donc lancer `apply.sh` juste après avoir flashé la carte, avant toute installation à la main.

## Réinstaller de zéro

C'est le seul vrai test que `install/` est complet. À faire avant chaque compétition, et à garder comme carte de secours :

1. Flasher Ubuntu Server 24.04 (arm64) pour Raspberry Pi sur une carte SD neuve.
2. `git clone` du repo, puis `./install/apply.sh`.
3. Vérifier que le drone fonctionne.

## Usages

```bash
./install/apply.sh            # détecte la Pi (rôle pi), sinon ROS 2 seul
./install/apply.sh pi         # force le rôle pi
./install/apply.sh sim        # PC Linux avec la GUI Gazebo native
./install/apply.sh --check --diff   # essai à blanc : montre ce qui changerait
```

Les images Docker de `sim/docker/` utilisent le même playbook. Après une modification, `cd sim && docker compose build`.
