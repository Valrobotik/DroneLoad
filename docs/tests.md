# Tests

Tous les tests sont dans `tests/`, jamais dans le code du drone :

```
tests/
  run.py           lanceur (sur le PC, Python standard) : prépare la simu, lance les tests, fait le bilan
  simu/            tests de bout en bout, dans un conteneur de la simulation Docker
    commun.py      code partagé des tests de simu
  resultats/       journaux du lanceur (ignoré par git)
```

La simulation elle-même (installation, services, dépannage) est décrite dans [simulation.md](simulation.md).

## Lancer les tests

Depuis la racine du repo, sur le PC (macOS, Linux ; Windows avec Git Bash) :

```bash
python3 tests/run.py --liste              # tests, suites et ce qu'il leur faut
python3 tests/run.py mission              # un test
python3 tests/run.py mission uwb          # plusieurs
python3 tests/run.py avec_uwb             # une suite : base, avec_uwb, tout
```

Pour chaque test de simu, le lanceur :
1. démarre la simu si elle ne tourne pas, dans la config du test : avec ou sans l'UWB simulé (`PARAM_UWB`), avec ou sans le code du drone (service `drone`, arrêté pour `smoke`) ;
2. remet le drone au départ (`sim/scripts/reset_drone.sh`) puis attend 40 s que le SITL accepte d'armer ;
3. lance le test dans le bon conteneur et affiche les lignes `[OK]` / `[ÉCHEC]` (et `OBSERVATION`) ;
4. garde la sortie complète dans `tests/resultats/<date>/<test>.log`.

À la fin : bilan par test et code de sortie (0 si tout est OK). La simu est arrêtée si le lanceur l'a démarrée.

**Config UWB.** Pour changer de config, le lanceur redémarre toute la simu (un SITL recréé seul laisserait MAVROS et le code du drone sur les paramètres de l'ancien). Il le fait seulement pour une simu qu'il a démarrée : une simu lancée à la main dans l'autre config n'est jamais touchée, le test est sauté (compté en échec) avec la commande pour la relancer. Les tests sont regroupés (sans UWB, puis avec) pour ne redémarrer qu'une fois.

| Option | Effet |
|---|---|
| `--garder` | laisser tourner la simu démarrée par le lanceur |
| `--attente S` | secondes après la remise au départ (40 par défaut) |
| `-v` | afficher toute la sortie des tests |

Pour regarder un test dans la vue 3D : lancer d'abord la simu à la main avec `--profile gui` dans la config du test (voir [simulation.md](simulation.md)), puis le lanceur ; il ne touche pas à une simu qu'il n'a pas démarrée.

## Les tests

| Test | Vérifie | Simu | Durée |
|---|---|---|---|
| `smoke` | autopilote seul : MAVROS, caméras, lidar, GUIDED refusé par MAVLink (`FLTMODE_GCSBLOCK`), GUIDED à la radio simulée et armement, décollage à 1 m, 5 s de stationnaire, atterrissage. Ne teste pas le code de `drone/` | sans le code du drone | 1 min |
| `mission` | 2 cycles décollage / attente stable 5 s / atterrissage commandés par MQTT comme le dashboard, GUIDED à la radio simulée ; `takeoff` attend GUIDED sans armer ; `land` refusé quand le pilote a repris la main (FlowHold) ; pose 3D de l'ArUco 0 cohérente avec le lidar | code du drone | 1,5 min |
| `position` | perte de position en mission `deplacement` : pause en moins de 2 s, reprise après le recalage, abandon après 30 s, reprise en main par le pilote (radio simulée en FlowHold) sans reprise seule | avec UWB | 2 min |
| `uwb` | position UWB dans l'EKF : stationnaire, 30 % de sauts de 2 m rejetés, carrés de 1,6 m en zone B à 0,5 et 1 m/s, pannes de 5 et 30 s, jeu de sources 2 | avec UWB | 4 min |
| `uwb_cap` | même EKF que `uwb`, drone posé tourné de 90° au départ (`reset_drone.sh 90`) : l'UWB ne donne pas de cap, la position de l'EKF ne doit pas tourner avec le drone (décollage, stationnaire 20 s, décalage EKF / vérité, atterrissage) | avec UWB | 1,5 min |
| `uwb_pilote` | pilote du vrai tag UWB (`uwb.py`) contre un faux tag : décodage TLV, ancres, tag muet, port déjà ouvert | code du drone construit | 0,5 min |

Suites : `base` (smoke, mission), `avec_uwb` (uwb, uwb_cap, position, uwb_pilote), `tout` (les deux, ~12 min).

Le détail de chaque test (étapes, seuils) est en tête de son fichier dans `tests/simu/`.

**Quel test après une modification ?** Tout changement de `drone/` : au moins `mission` (le code du drone doit avoir tourné, `smoke` ne suffit pas). Missions, position, UWB : `position`, `uwb`. Paramètres ArduPilot ou monde Gazebo : `smoke`, puis `mission`.

**Ce que les tests ne couvrent pas** : le vrai drone et la vraie Pi (caméra, servo, MAVROS en série, vrai tag UWB) ; le flux optique simulé est presque parfait, la dérive réelle sera plus forte.

## Lancer un test à la main

Pour la mise au point d'un test. Simu dans la bonne config (voir [simulation.md](simulation.md)), drone au départ :

```bash
cd sim
./scripts/reset_drone.sh
docker compose exec companion bash -lc 'python3 tests/simu/mission.py'
```

`exec` ne passe pas par l'entrypoint du conteneur : `bash -lc '...'` charge l'environnement ROS 2. `uwb_pilote` a besoin de `. drone/install/setup.bash &&` avant. Lancés à la main, `uwb` et `position` s'adaptent à la config de la simu (avec ou sans UWB) ; par le lanceur, ils échouent si elle n'est pas celle attendue.

## Écrire un test

1. Un fichier dans `tests/simu/`, sans suffixe `_test`, avec en tête ce qu'il vérifie et comment le lancer.
2. Utiliser `tests/simu/commun.py` :
   - `Etapes('TEST DE ...')` : `etape(nom, ok)` affiche `[OK]` / `[ÉCHEC]`, `etape.fin()` affiche le verdict et renvoie le code de sortie ;
   - `ClientMqtt(nom)` : le drone vu du dashboard (`telemetry`, `logs`, `set_mode`, `set_point`, `commande`, `attendre`, `heure_log`) ;
   - `Radio()` : radio du pilote simulée (voies RC envoyées à l'entrée RC du SITL, UDP 5501) : `guided()` met l'interrupteur de modes sur GUIDED (seule façon d'y passer, `FLTMODE_GCSBLOCK`), `envoyer('flowhold', gaz=1500)` simule une reprise en main ; à appeler avant tout décollage, après chaque remise au départ (le SITL redémarré oublie les voies) ;
   - `depart_sdf()` (départ du drone lu dans le monde Gazebo), `position_sdf('aruco_0')` (position d'un visuel du monde) : lire les cotes dans `sim/worlds/voliere.sdf` plutôt que les recopier dans le test ;
   - `param_drone(noeud, nom)` (réglage lu dans `drone.yaml`, plutôt que d'en garder une copie dans le test), `xy(pose)`, `entier(reponse)` (paramètre entier lu par `GetParameters`), `ros2(...)`, `set_uwb()`, `set_vitesse()` (en ligne de commande ; un test qui a déjà son nœud ROS passe plutôt par ses services, plus rapides), `verifier_uwb()` ;
   - toujours `atterrir_si_arme()` dans un `finally` : un test interrompu ne laisse jamais le drone en vol.
3. Piloter le drone comme le dashboard (MQTT : `set_mode`, `set_point`…) plutôt qu'en envoyant des consignes à MAVROS : on teste le vrai code du drone.
4. Une ligne dans la table `TESTS` de `tests/run.py` : commande, conteneur, config UWB (`True`, `False`, `None` : indifférente), code du drone requis ou non, remise au départ (et cap de départ, `cap=90` pour `uwb_cap`) ; et l'ajouter à une suite.
5. Un résultat qui dépend du hasard de la simu et qui n'est pas une erreur : l'afficher en `OBSERVATION`, sans `[ÉCHEC]`.
