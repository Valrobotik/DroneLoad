TARGET_CLASS = 'cow'      # la classe COCO standard pour une vache
DISTANCE_MIN = 1.0        # m : s'arrêter à 1 mètre de l'obstacle
DISTANCE_AVANT = 2.5      # m : TODO(#34) aucun capteur frontal pour l'instant (profondeur à brancher)
K_YAW = 0.002             # gain de rotation, à ajuster en essai réel
CENTRE_PX = 80            # la cible est « centrée » à +/- 80 pixels


def run_epreuve2(drone, detections, frame_width):
    """Un tick : tourne vers la vache la plus sûre et avance tant qu'elle est centrée et loin.

    detections : liste de dictionnaires {"class": "cow", "box": (x1, y1, x2, y2), "conf": 0.85}
    frame_width : largeur de l'image (ex. 640) pour le centrage.
    """
    cibles = [d for d in detections if d['class'] == TARGET_CLASS]
    if not cibles:
        # Vache perdue (ou pas encore trouvée) : le drone garde sa position
        drone.send_velocity_body(0, 0, 0, 0)
        return

    cible = max(cibles, key=lambda d: d['conf'])
    x1, _, x2, _ = cible['box']
    erreur_x = (x1 + x2) / 2 - frame_width / 2

    # Contrôleur proportionnel : on tourne vers la vache, on avance si elle est à peu près centrée
    yaw_rate = erreur_x * K_YAW
    vx = 0.3 if abs(erreur_x) < CENTRE_PX and DISTANCE_AVANT > DISTANCE_MIN else 0.0
    drone.send_velocity_body(vx, 0, 0, yaw_rate)
