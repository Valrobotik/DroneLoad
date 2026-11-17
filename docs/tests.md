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
python3 tests/run.py smoke mission        # plusieurs
python3 tests/run.py base                 # une suite : base, tout
```

Pour chaque test de simu, le lanceur :
1. démarre la simu si elle ne tourne pas, dans la config du test : avec ou sans le code du drone (service `drone`, arrêté pour `smoke`) ;
2. remet le drone au départ (`sim/scripts/reset_drone.sh`) puis attend 40 s que le SITL accepte d'armer ;
3. lance le test dans le bon conteneur et affiche les lignes `[OK]` / `[ÉCHEC]` (et `OBSERVATION`) ;
4. garde la sortie complète dans `tests/resultats/<date>/<test>.log`.

À la fin : bilan par test et code de sortie (0 si tout est OK). La simu est arrêtée si le lanceur l'a démarrée.

| Option | Effet |
|---|---|
| `--garder` | laisser tourner la simu démarrée par le lanceur |
| `--attente S` | secondes après la remise au départ (40 par défaut) |
| `-v` | afficher toute la sortie des tests |

Pour regarder un test dans la vue 3D : lancer d'abord la simu à la main avec `--profile gui` (voir [simulation.md](simulation.md)), puis le lanceur ; il ne touche pas à une simu qu'il n'a pas démarrée.

## Les tests

| Test | Vérifie | Simu | Durée |
|---|---|---|---|
| `smoke` | autopilote seul : MAVROS, caméras, lidar, GUIDED et armement, décollage à 1 m, 5 s de stationnaire, atterrissage. Ne teste pas le code de `drone/` | sans le code du drone | 1 min |
| `mission` | 2 cycles décollage / attente stable 5 s / atterrissage commandés par MQTT comme le dashboard ; pose 3D de l'ArUco 0 cohérente avec le lidar | code du drone | 1,5 min |

Suites : `base` (smoke, mission), `tout` (tous les tests).

Le détail de chaque test (étapes, seuils) est en tête de son fichier dans `tests/simu/`.

**Quel test après une modification ?** Tout changement de `drone/` : au moins `mission` (le code du drone doit avoir tourné, `smoke` ne suffit pas). Paramètres ArduPilot ou monde Gazebo : `smoke`, puis `mission`.

**Ce que les tests ne couvrent pas** : le vrai drone et la vraie Pi (caméra, servo, MAVROS en série) ; le flux optique simulé est presque parfait, la dérive réelle sera plus forte.

## Lancer un test à la main

Pour la mise au point d'un test. Simu dans la bonne config (voir [simulation.md](simulation.md)), drone au départ :

```bash
cd sim
./scripts/reset_drone.sh
docker compose exec companion bash -lc 'python3 tests/simu/mission.py'
```

`exec` ne passe pas par l'entrypoint du conteneur : `bash -lc '...'` charge l'environnement ROS 2.

## Écrire un test

1. Un fichier dans `tests/simu/`, sans suffixe `_test`, avec en tête ce qu'il vérifie et comment le lancer.
2. Utiliser `tests/simu/commun.py` :
   - `Etapes('TEST DE ...')` : `etape(nom, ok)` affiche `[OK]` / `[ÉCHEC]`, `etape.fin()` affiche le verdict et renvoie le code de sortie ;
   - `ClientMqtt(nom)` : le drone vu du dashboard (`telemetry`, `logs`, `set_mode`, `commande`, `attendre`, `heure_log`) ;
   - `depart_sdf()` (départ du drone lu dans le monde Gazebo), `ros2(...)` ;
   - toujours `atterrir_si_arme()` dans un `finally` : un test interrompu ne laisse jamais le drone en vol.
3. Piloter le drone comme le dashboard (MQTT : `set_mode`…) plutôt qu'en envoyant des consignes à MAVROS : on teste le vrai code du drone.
4. Une ligne dans la table `TESTS` de `tests/run.py` : commande, conteneur, code du drone requis ou non, remise au départ ; et l'ajouter à une suite.
5. Un résultat qui dépend du hasard de la simu et qui n'est pas une erreur : l'afficher en `OBSERVATION`, sans `[ÉCHEC]`.
