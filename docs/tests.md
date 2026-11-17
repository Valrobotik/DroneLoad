# Tests

Tous les tests sont dans `tests/`, jamais dans le code du drone :

```
tests/
  run.py           lanceur (sur le PC, Python standard) : prépare la simu, lance les tests, fait le bilan
  unitaires/       tests sans ROS ni simulation (pytest)
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
python3 tests/run.py mission carte        # plusieurs
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
| `unitaires` | changements de repère de la carte (`reperes.py`) : caméra sol vers la carte, fenêtre par ses ArUco (recto, verso, tournée) | aucune | < 1 s |
| `smoke` | autopilote seul : MAVROS, caméras, lidar, GUIDED et armement, décollage à 1 m, 5 s de stationnaire, atterrissage. Ne teste pas le code de `drone/` | sans le code du drone | 1 min |
| `mission` | 2 cycles décollage / attente stable 5 s / atterrissage commandés par MQTT comme le dashboard ; pose 3D de l'ArUco 0 cohérente avec le lidar | code du drone | 1,5 min |
| `carte` | carte en stationnaire (mission `releve`) : ArUco 0 placé à moins de 8 cm (erreur de l'EKF retirée), EKF à moins de 30 cm de la vérité, `reset_carte` | sans UWB | 1 min |
| `carte_uwb` | idem, avec l'UWB | avec UWB | 1 min |
| `carte_mouvement` | carte en mouvement : passages au-dessus de l'ArUco 0 à 0,5 puis 1 m/s (point à moins de 15 cm ; à 1 m/s, pas toujours placé : observation), panne d'UWB et recalage de 3 s (rien enregistré), changement de repère refusé, nœud `carte` redémarré puis `charger_carte` | avec UWB, conteneur `drone` | 2,5 min |
| `position` | perte de position en mission `deplacement` : pause en moins de 2 s, reprise après le recalage, abandon après 30 s, reprise en main par le pilote (ALT_HOLD) sans reprise seule | avec UWB | 2 min |
| `uwb` | position UWB dans l'EKF : stationnaire, 30 % de sauts de 2 m rejetés, carrés de 2 m à 0,5 et 1 m/s, pannes de 5 et 30 s, jeu de sources 2 | avec UWB | 4 min |
| `uwb_pilote` | pilote du vrai tag UWB (`uwb.py`) contre un faux tag : décodage TLV, ancres, tag muet, port déjà ouvert | code du drone construit | 0,5 min |

Suites : `base` (unitaires, smoke, mission, carte), `avec_uwb` (uwb, position, carte_uwb, carte_mouvement, uwb_pilote), `tout` (les deux, ~15 min).

Le détail de chaque test (étapes, seuils) est en tête de son fichier dans `tests/simu/`.

**Quel test après une modification ?** Tout changement de `drone/` : au moins `mission` (le code du drone doit avoir tourné, `smoke` ne suffit pas). Missions, position, UWB : `position`, `uwb`. Carte : `carte`, `carte_mouvement`, `unitaires`. Paramètres ArduPilot ou monde Gazebo : `smoke`, puis `mission`.

**Ce que les tests ne couvrent pas** : le vrai drone et la vraie Pi (caméra, servo, MAVROS en série, vrai tag UWB) ; le flux optique simulé est presque parfait, la dérive réelle sera plus forte.

## Lancer un test à la main

Pour la mise au point d'un test. Simu dans la bonne config (voir [simulation.md](simulation.md)), drone au départ :

```bash
cd sim
./scripts/reset_drone.sh
docker compose exec companion bash -lc 'python3 tests/simu/mission.py'
```

`exec` ne passe pas par l'entrypoint du conteneur : `bash -lc '...'` charge l'environnement ROS 2. `carte_mouvement` se lance dans le conteneur `drone` (il redémarre le nœud `carte`) ; `uwb_pilote` a besoin de `. drone/install/setup.bash &&` avant. Lancés à la main, `uwb` et `position` s'adaptent à la config de la simu (avec ou sans UWB) ; par le lanceur, ils échouent si elle n'est pas celle attendue.

## Écrire un test

1. Un fichier dans `tests/simu/`, sans suffixe `_test` (pytest ne doit ramasser que `tests/unitaires/`), avec en tête ce qu'il vérifie et comment le lancer.
2. Utiliser `tests/simu/commun.py` :
   - `Etapes('TEST DE ...')` : `etape(nom, ok)` affiche `[OK]` / `[ÉCHEC]`, `etape.fin()` affiche le verdict et renvoie le code de sortie ;
   - `ClientMqtt(nom)` : le drone vu du dashboard (`telemetry`, `logs`, `position`, `carte`, `set_mode`, `set_point`, `commande`, `attendre`, `point`, `heure_log`) ;
   - `depart_sdf()` (départ du drone lu dans le monde Gazebo), `ros2(...)`, `set_uwb()`, `set_vitesse()`, `verifier_uwb()` ;
   - toujours `atterrir_si_arme()` dans un `finally` : un test interrompu ne laisse jamais le drone en vol.
3. Piloter le drone comme le dashboard (MQTT : `set_mode`, `set_point`…) plutôt qu'en envoyant des consignes à MAVROS : on teste le vrai code du drone.
4. Une ligne dans la table `TESTS` de `tests/run.py` : commande, conteneur, config UWB (`True`, `False`, `None` : indifférente), code du drone requis ou non, remise au départ ; et l'ajouter à une suite.
5. Un résultat qui dépend du hasard de la simu et qui n'est pas une erreur (ex. ArUco pas vu à 1 m/s) : l'afficher en `OBSERVATION`, sans `[ÉCHEC]`.

Un test unitaire (sans ROS) va dans `tests/unitaires/test_*.py`, exécutable aussi directement (`python3 <fichier>`).

## CI

`.github/workflows/tests.yml` lance `python3 tests/run.py unitaires` sur chaque PR vers `main` (sans ROS ni simulation). Les tests de simu se lancent en local : une simu dans la CI reste à faire.
