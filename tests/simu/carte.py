"""Test de la carte (mission releve, nœud carte, relais MQTT de mqtt_server), par MQTT seulement comme ground/carte.py.

Décollage, relevé (mode releve : enregistre ce que voit la caméra) en stationnaire au-dessus de l'ArUco 0,
puis vérifie :
- drone/position : position EKF reçue, à moins de POSITION_TOLERANCE de la vérité Gazebo ;
- drone/carte : ArUco 0 repéré à moins d'ARUCO_TOLERANCE de sa position dans voliere.sdf, erreur de l'EKF
  retirée (sinon le test mesurerait surtout la dérive du flux optique) ;
- reset_carte : la carte se vide ;
puis atterrissage.

Repère : sans UWB (PARAM_UWB vide, défaut), l'origine de l'EKF est le départ du drone (DEPART) ;
avec --uwb (PARAM_UWB=uwb), le repère de l'EKF est celui de Gazebo.

Lancement : python3 tests/run.py carte (ou carte_uwb : simu avec PARAM_UWB=uwb), qui prépare la simu.
À la main : simu lancée avec le code du drone (docker compose --profile drone up -d), drone au départ
(sim/scripts/reset_drone.sh), puis docker compose exec companion bash -lc 'python3 tests/simu/carte.py [--uwb]'.
Code de sortie 0 si tout est OK, 1 sinon.
"""

import argparse
import math
import sys
import time

from commun import TAKEOFF_ALT, ClientMqtt, Etapes, Radio, depart_sdf

DEPART = depart_sdf()
ARUCO_0 = DEPART           # le drone part posé sur l'ArUco 0
POSITION_TOLERANCE = 0.3   # m, écart EKF / vérité Gazebo
ARUCO_TOLERANCE = 0.08     # m, erreur de la carte seule (sans celle de l EKF)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--uwb', action='store_true', help='simu lancée avec PARAM_UWB=uwb')
    decalage = (0.0, 0.0) if parser.parse_args().uwb else DEPART   # sans UWB : origine de l'EKF au départ
    drone = ClientMqtt('carte_test')
    radio = Radio()   # pilote simulé : seule la radio passe en GUIDED (FLTMODE_GCSBLOCK)
    etape = Etapes('TEST DE LA CARTE')

    def terrain(x, y):
        return x + decalage[0], y + decalage[1]

    def aruco_0():
        return drone.point('aruco', '0')

    try:
        if not etape('télémétrie MQTT du drone reçue', drone.attendre(lambda: drone.telemetry is not None, 30)):
            return 1
        if not etape('MAVROS connecté', drone.attendre(lambda: drone.telemetry.get('fcu_connected'), 60)):
            return 1
        if not etape('drone/position : position EKF et vérité Gazebo reçues',
                     drone.attendre(lambda: drone.ecart_ekf_verite(decalage) is not None, 30)):
            return 1
        drone.commande('reset_carte')
        drone.attendre(lambda: drone.carte == [], 5)
        print(f'    au sol : écart EKF / vérité {drone.ecart_ekf_verite(decalage):.2f} m', flush=True)

        drone.set_mode('takeoff')
        radio.guided()
        if not etape(f'décollage à {TAKEOFF_ALT} m', drone.attendre(drone.en_vol, 150)):
            return 1
        drone.set_mode('releve')
        time.sleep(3.0)

        ecart = drone.ecart_ekf_verite(decalage)
        etape(f'en vol : écart EKF / vérité Gazebo {ecart:.2f} m (< {POSITION_TOLERANCE} m)',
              ecart is not None and ecart < POSITION_TOLERANCE)
        if etape('ArUco 0 sur drone/carte', drone.attendre(lambda: aruco_0() is not None, 10)):
            a = aruco_0()
            x, y = terrain(a['x'], a['y'])
            # Erreur propre à la carte (caméra, pose ArUco, changement de repère) : écart à la position
            # attendue, une fois retirée l'erreur de l'EKF (dérive du flux optique, mesurée sur la vérité)
            p = drone.position
            ex, ey = terrain(p['ekf']['x'], p['ekf']['y'])
            dx, dy = ex - p['verite']['x'], ey - p['verite']['y']
            ecart = math.dist((x - dx, y - dy), ARUCO_0)
            etape(f"ArUco 0 placé en ({x:+.2f}, {y:+.2f}), {a['vues']} vues : écart brut {math.dist((x, y), ARUCO_0):.2f} m, "
                  f'{ecart:.2f} m sans l\'erreur de l\'EKF (< {ARUCO_TOLERANCE} m)', ecart < ARUCO_TOLERANCE)

        # Le drone voit encore l'ArUco : la carte se vide puis le point revient après vues_min images
        drone.set_mode('attente')   # attente n'enregistre rien
        time.sleep(0.5)
        drone.commande('reset_carte')
        etape('reset_carte : carte vidée', drone.attendre(lambda: drone.carte == [], 5))

        drone.set_mode('land')
        etape('atterrissage et désarmement', drone.attendre(lambda: not drone.armed, 60))
        drone.set_mode('attente')
    finally:
        drone.atterrir_si_arme()
        radio.fermer()
        drone.fermer()
    return etape.fin()


if __name__ == '__main__':
    sys.exit(main())
