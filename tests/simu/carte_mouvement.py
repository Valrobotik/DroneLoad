"""Test de la carte en mouvement et de sa reprise après redémarrage, avec l'UWB (repère de l'EKF = Gazebo).

Par MQTT comme le dashboard (mission releve : enregistre ce que voit la caméra, en stationnaire ou vers un
point envoyé par set_point pendant le relevé). Complète carte.py (stationnaire seulement) :
1. passages au-dessus de l'ArUco 0 en ligne droite sans s'arrêter, à 0,5 puis 1 m/s (à 1 m/s, l'ArUco
   peut être vu moins de vues_min fois et ne pas être placé : observation ; vitesse_max de
   main_node réglée en vol ; drone penché,
   marqueur au bord de l'image, pose prise à la réception de l'image alors que le drone a avancé), carte
   vidée avant chaque passage, hors de vue de l'ArUco : le point n'est fait que d'images prises en mouvement.
   Vérifie : ArUco 0 repéré, à moins d'ARUCO_MOUVEMENT_MAX de sa position dans voliere.sdf, EKF à moins de
   SUIVI_MAX de la vérité pendant le passage, aucune vue rejetée par le nœud carte ;
2. retour au-dessus de l'ArUco par set_point, stationnaire : point à moins d'ARUCO_STATIONNAIRE_MAX ;
3. panne d'UWB de PANNE s en relevé stationnaire : position non fiable (télémétrie position_fiable), rien
   n'est enregistré (ni par la mission releve, en pause, ni un point envoyé d'un autre nœud sur
   /carte/ajouter, comme le fera la caméra avant) ; au retour de l'UWB, toujours rien pendant
   delai_recalage, puis l'enregistrement reprend ;
4. repère : /carte/repere passé à flux (comme un vol sans UWB) alors que la carte a des points en uwb :
   ajouts refusés et signalés ; revenu à uwb : acceptés ;
5. redémarrage du nœud carte (tué, relancé par le launch) : carte vide au démarrage, puis charger_carte
   reprend la carte d'avant (même position, repère uwb) ;
6. atterrissage.

Passages selon y, à x constant (celui du départ) : fenetre_1 est en x = -3, le mur bas en x = -4,
le mur droit en y = -3.

Lancement : python3 tests/run.py carte_mouvement, qui prépare la simu (avec l'UWB) et le lance dans le
conteneur drone (pour pouvoir redémarrer le nœud carte). À la main : PARAM_UWB=uwb docker compose --profile
drone up -d, drone au départ (sim/scripts/reset_drone.sh), puis
docker compose exec drone bash -lc 'python3 tests/simu/carte_mouvement.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import math
import subprocess
import sys
import time

from commun import REPO, TAKEOFF_ALT, ClientMqtt, Etapes, Radio, depart_sdf, param_drone, ros2, set_uwb, set_vitesse

ARUCO_0 = depart_sdf()         # le drone part posé sur l'ArUco 0

ELAN = 0.8                     # m : départ du passage avant l'ArUco, côté mur droit (à 1,5 m de l'ArUco)
AU_DELA = 1.2                  # m après l'ArUco, vers le centre : le drone freine (vitesse = distance x 1/s) au-delà de 0,5 m
ARUCO_MOUVEMENT_MAX = 0.15     # m : point fait d'images prises en mouvement
ARUCO_STATIONNAIRE_MAX = 0.08  # m
SUIVI_MAX = 0.3                # m : écart EKF - vérité pendant le passage
RECHARGE_MAX = 0.01            # m : point rechargé à la même place
CARTE_NODE = 'lib/droneload_vol/carte'   # motif pkill du processus du nœud carte
PANNE = 5.0                    # s de panne d'UWB
DELAI_RECALAGE = param_drone('main_node', 'delai_recalage')   # s
VITESSES = (0.5, 1.0)          # m/s des passages
VITESSE_DEFAUT = param_drone('main_node', 'vitesse_max')     # m/s, remise à la fin
VITESSE_SURE = 0.5             # m/s : au-delà, l'ArUco peut ne pas être placé (observation seulement)


def set_repere(repere):
    """Repère de l'EKF annoncé au nœud carte (normalement par main_node, à l'armement)."""
    # transient local comme les abonnements (sinon ros2 topic pub n'en trouve aucun compatible) ; attend les
    # deux abonnés (carte, mqtt_server) et reste 1 s : publié dès le premier, le nœud carte pouvait le rater
    return ros2('topic', 'pub', '--once', '-w', '2', '--keep-alive', '1', '--qos-durability', 'transient_local',
                '--qos-depth', '1',
                '/carte/repere', 'std_msgs/msg/String', f'{{data: {repere}}}')


def point_externe(action=0):
    """Point 'cible externe' envoyé sur /carte/ajouter comme un autre nœud (caméra avant) ; action 2 : le retirer."""
    return ros2('topic', 'pub', '--once', '-w', '1', '/carte/ajouter', 'visualization_msgs/msg/Marker',
                f"{{ns: cible, text: externe, action: {action}, pose: {{orientation: {{w: 0.0}}}}}}")


class Test(ClientMqtt):
    def __init__(self):
        super().__init__('carte_mouvement_test')

    def aruco_0(self):
        return self.point('aruco', '0')

    def aller(self, x, y, timeout=30):
        """set_point pendant le relevé ; vrai une fois arrivé. Mesure l'écart EKF - vérité et la vitesse réelle."""
        self.set_point(x, y)
        self.attendre(lambda: self.mission == 'releve:EN_ROUTE', 3)
        ecart_max = vitesse_max = 0.0
        precedent = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self.mission != 'releve:ARRIVE':
            ecart = self.ecart_ekf_verite()
            if ecart is not None:
                ecart_max = max(ecart_max, ecart)
            v = (self.position or {}).get('verite')
            now = time.monotonic()
            if v and (precedent is None or now - precedent[0] >= 0.2):
                if precedent is not None:
                    vitesse_max = max(vitesse_max, math.dist((v['x'], v['y']), precedent[1:]) / (now - precedent[0]))
                precedent = (now, v['x'], v['y'])
            time.sleep(0.05)
        return self.mission == 'releve:ARRIVE', ecart_max, vitesse_max


def main():
    test = Test()
    radio = Radio()
    step = Etapes('TEST DE LA CARTE EN MOUVEMENT')

    def verifier_point(nom, tolerance):
        a = test.aruco_0()
        if not step(f'{nom} : ArUco 0 sur drone/carte', a is not None):
            return None
        ecart = math.dist((a['x'], a['y']), ARUCO_0)
        step(f"{nom} : ArUco 0 en ({a['x']:+.2f}, {a['y']:+.2f}), {a['vues']} vues, "
             f'à {ecart:.2f} m de sa position (< {tolerance} m)', ecart < tolerance)
        return a

    x0, y0 = ARUCO_0
    try:
        if not step('télémétrie MQTT du drone reçue', test.attendre(lambda: test.telemetry is not None, 30)):
            return 1
        if not step('MAVROS connecté', test.attendre(lambda: test.telemetry.get('fcu_connected'), 60)):
            return 1
        if not step('mesures UWB reçues (simu lancée avec PARAM_UWB=uwb)',
                    test.attendre(lambda: test.telemetry.get('uwb_hz', 0) > 0, 10)):
            return 1
        if not step('drone/position : EKF dans le repère de Gazebo (UWB) au sol',
                    test.attendre(lambda: (test.ecart_ekf_verite() or 1.0) < 0.3, 30)):
            return 1
        test.commande('reset_carte')

        test.set_mode('takeoff')
        radio.guided()   # pilote simulé : seule la radio passe en GUIDED (FLTMODE_GCSBLOCK)
        if not step(f'décollage à {TAKEOFF_ALT} m', test.attendre(test.en_vol, 150)):
            return 1
        time.sleep(3.0)
        test.set_mode('releve')
        step('mode releve : ArUco 0 enregistré en stationnaire', test.attendre(lambda: test.aruco_0() is not None, 10))

        debut_logs = len(test.logs)
        try:
            for v in VITESSES:
                step(f'vitesse des déplacements : {v} m/s', set_vitesse(v))
                if not step('placé au début du passage, hors de vue de l\'ArUco', test.aller(x0, y0 - ELAN)[0]):
                    break
                test.commande('reset_carte')
                if not test.attendre(lambda: test.carte == [], 5):
                    continue
                time.sleep(1.5)   # vérifie que rien n'est vu d'ici
                step('carte vide avant le passage (ArUco hors de vue)', test.carte == [])
                arrive, ecart, vitesse = test.aller(x0, y0 + AU_DELA)
                if step(f'passage à {v} m/s au-dessus de l\'ArUco 0 (vitesse réelle max {vitesse:.2f} m/s)', arrive):
                    time.sleep(1.5)   # publication de la carte (1 Hz)
                    step(f'EKF suit la vérité pendant le passage (écart max {ecart:.2f} m < {SUIVI_MAX} m)',
                         ecart < SUIVI_MAX)
                    if v > VITESSE_SURE and test.aruco_0() is None:
                        # Au-delà de ~0,5 m/s à 1 m de haut, l'ArUco peut être vu moins de vues_min fois (champ
                        # de ±0,3 m au sol, ~12 images/s) : pas placé. Limite connue, observée sans critère
                        print(f'    OBSERVATION : à {v} m/s, ArUco 0 vu moins de vues_min fois : pas placé', flush=True)
                    else:
                        verifier_point(f'passage à {v} m/s', ARUCO_MOUVEMENT_MAX)
        finally:
            set_vitesse(VITESSE_DEFAUT)
        step('aucune vue rejetée par la carte (drone/logs)',
             not any('de la moyenne, ignorée' in log for log in test.logs[debut_logs:]))

        if step('retour au-dessus de l\'ArUco 0', test.aller(x0, y0)[0]):
            test.set_mode('releve')   # oublie le point : stationnaire
            test.commande('reset_carte')
            time.sleep(5.0)
            avant = verifier_point('stationnaire', ARUCO_STATIONNAIRE_MAX)

            def vues():
                a = test.aruco_0()
                return a['vues'] if a else 0

            def externe():
                return any(p.get('type') == 'cible' and p.get('nom') == 'externe' for p in test.carte or [])

            debut_logs = len(test.logs)
            if step(f'panne d\'UWB ({PANNE:.0f} s) en relevé', set_uwb(False)):
                try:
                    step('télémétrie : position non fiable',
                         test.attendre(lambda: test.telemetry.get('position_fiable') is False, 5))
                    time.sleep(1.5)   # dernières vues acceptées, carte publiée
                    v0 = vues()
                    point_externe()
                    time.sleep(max(0.0, PANNE - 3.0))
                    step(f'pendant la panne : rien enregistré (ArUco 0 : {v0} -> {vues()} vues, point externe absent)',
                         vues() == v0 and not externe())
                finally:
                    set_uwb(True)
                step('UWB rétabli : position fiable', test.attendre(lambda: test.telemetry.get('position_fiable'), 5))
                step('enregistrement repris après le recalage',
                     test.attendre(lambda: vues() > v0, DELAI_RECALAGE + 5))
                # Délai entre le retour de la position (main_node) et la reprise (carte), mesuré sur drone/logs :
                # les deux logs suivent le même chemin (rosout -> mqtt_server -> MQTT)
                retour = test.heure_log('revenue : ', debut_logs)   # retour de l'UWB (MavrosDrone)
                reprise = test.heure_log('enregistrement repris', debut_logs)
                if step('retour de la position et reprise de la carte sur drone/logs', None not in (retour, reprise)):
                    step(f'recalage : rien enregistré pendant {reprise - retour:.1f} s après le retour '
                         f'(>= {DELAI_RECALAGE - 0.3:.1f} s)', reprise - retour >= DELAI_RECALAGE - 0.3)
                point_externe()
                step('point externe accepté', test.attendre(externe, 5))
                point_externe(action=2)
                step('point externe retiré', test.attendre(lambda: not externe(), 5))
                step('refus et reprise signalés sur drone/logs',
                     any('non fiable' in log for log in test.logs[debut_logs:])
                     and any('enregistrement repris' in log for log in test.logs[debut_logs:]))

            test.set_mode('attente')   # attente n'enregistre rien
            time.sleep(1.0)
            debut_logs = len(test.logs)
            if step('repère annoncé : flux (vol sans UWB)', set_repere('flux')):
                try:
                    point_externe()
                    time.sleep(1.5)
                    step('carte en uwb, EKF en flux : point refusé et signalé',
                         not externe() and any('ne mélange pas' in log for log in test.logs[debut_logs:]))
                finally:
                    set_repere('uwb')
                point_externe()
                step('repère revenu à uwb : point accepté', test.attendre(externe, 5))
                point_externe(action=2)
                test.attendre(lambda: not externe(), 5)
            time.sleep(2.0)   # dernière sauvegarde (1 Hz)
            avant = test.aruco_0()
            debut_logs = len(test.logs)
            tue_a = time.time()
            tue = subprocess.run(['pkill', '-f', CARTE_NODE]).returncode == 0
            if step('nœud carte arrêté (test lancé dans le conteneur drone)', tue) and avant is not None:
                # Carte d'avant mise de côté par le nœud relancé : message de démarrage sur drone/logs, ou à défaut
                # le fichier renommé (ctime : un renommage ne change pas la date de modification) ; carte publiée vide
                precedente = REPO / 'donnees' / 'carte_sim_precedente.json'

                def mise_de_cote():
                    return (test.log_recu('carte précédente gardée de côté', debut_logs)
                            or (precedente.exists() and precedente.stat().st_ctime > tue_a))
                step('nœud carte relancé : carte vide au démarrage, ancienne mise de côté',
                     test.attendre(mise_de_cote, 20) and test.attendre(lambda: test.carte == [], 5))
                test.commande('charger_carte')
                if step('charger_carte : ArUco 0 repris', test.attendre(lambda: test.aruco_0() is not None, 5)):
                    apres = test.aruco_0()
                    d = math.dist((apres['x'], apres['y']), (avant['x'], avant['y']))
                    step(f"même position ({d * 1000:.0f} mm) et mêmes vues ({apres['vues']})",
                         d < RECHARGE_MAX and apres['vues'] == avant['vues'])
                    step('chargée en repère uwb, sans avertissement',
                         any('repère uwb' in log for log in test.logs[debut_logs:])
                         and not any('sans UWB' in log for log in test.logs[debut_logs:]))

        test.set_mode('land')
        step('atterrissage et désarmement', test.attendre(lambda: not test.armed, 60))
        test.set_mode('attente')
    finally:
        test.atterrir_si_arme()   # test interrompu en vol : ne jamais laisser le drone voler
        radio.fermer()
        test.fermer()
    return step.fin()


if __name__ == '__main__':
    sys.exit(main())
