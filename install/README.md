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
- fichiers dans `/usr/local/bin` ;
- sur la Pi, une configuration du rôle `pi` défaite (UART de la Pixhawk, console série, caméra, règles udev, groupes `dialout` / `video` / `gpio`, bouton d'alimentation, mises à jour automatiques) ;
- ce qui se lance tout seul sans venir d'`install/` : services écrits dans `/etc/systemd/system` (hors snaps), `/etc/rc.local`, crontabs, lignes de `.bashrc` / `.profile` / `/etc/profile.d` qui lancent un programme (`python x.py`, `ros2 launch`, `mavproxy`, `nohup`...). Un vieux script relancé à chaque démarrage survivrait aux redémarrages et prendrait le port de la Pixhawk. Les crontabs des utilisateurs ne sont lisibles que par root : vérifiées par le timer, pas par un `check.py` lancé sans `sudo`.

Sur la Pi, le rôle `pi` installe aussi le service systemd `droneload`, qui lance `drone.launch.py` au démarrage et le relance s'il s'arrête (`restart_script` du dashboard, plantage). Il faut avoir construit `drone/` (`colcon build --symlink-install`) ; `sudo systemctl start droneload` pour le lancer sans redémarrer, `journalctl -u droneload -f` pour ses logs. Le rôle autorise aussi l'utilisateur du repo à lancer `reboot` et `poweroff` sans mot de passe (commandes du dashboard).

Il configure enfin le matériel de la Pi :
- liaison série vers la Pixhawk (`/dev/ttyAMA0`, GPIO 14/15) : `dtparam=uart0=on` dans `/boot/firmware/config.txt`, pas de console Linux ni de login sur ce port (`cmdline.txt`, `serial-getty@ttyAMA0` masqué) ;
- caméra CSI : `camera_auto_detect=1` ;
- bouton d'alimentation de la Pi 5 : appui court = arrêt propre (`/etc/systemd/logind.conf.d/droneload.conf`, même si une session bloque l'arrêt), appui long = coupure forcée ;
- droits : règles udev `/etc/udev/rules.d/99-droneload.rules` (puces GPIO au groupe `gpio`, `dma_heap` de libcamera au groupe `video`, réservés à root sinon sous Ubuntu), utilisateur du repo dans les groupes `dialout`, `video` et `gpio`.
 Il coupe aussi les mises à jour automatiques d'Ubuntu (`/etc/apt/apt.conf.d/99droneload-no-auto-upgrades`) : au concours 2026, la Pi ne voyait plus la Pixhawk, même après redémarrage, et rien ne doit changer le système en dehors d'`apply.sh`. Après une modification de `config.txt` ou `cmdline.txt`, `apply.sh` demande de redémarrer la Pi.

Sur la Pi, `check.py` tourne toutes les heures (timer systemd `droneload-check`) et publie les écarts sur `drone/logs`, ce qui les affiche dans le dashboard. Pour voir son dernier résultat : `journalctl -u droneload-check`.

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
