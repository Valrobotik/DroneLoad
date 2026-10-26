def run_epreuve2(drone, hw, yolo_data, frame_width):
    """
    S'exécute à chaque tour de boucle (tick) lorsque le mode 'epreuve2' est actif.

    yolo_data : liste de dictionnaires sous la forme
                [{"class": "cow", "box": (x1, y1, x2, y2), "conf": 0.85}, ...]
    frame_width : largeur de l'image (ex: 640) pour le centrage.
    """
    TARGET_CLASS = "cow"  # La classe COCO standard pour une vache
    DISTANCE_MIN = 1.0  # S'arrêter à 1 mètre de l'obstacle

    distance_avant = hw.get_forward_distance()

    # 1. Chercher la cible dans les détections YOLO
    cible = None
    meilleure_confiance = 0.0

    for detection in yolo_data:
        if detection["class"] == TARGET_CLASS:
            # S'il y a plusieurs vaches, on prend celle avec le meilleur score de confiance
            if detection["conf"] > meilleure_confiance:
                meilleure_confiance = detection["conf"]
                cible = detection

    # 2. Logique de navigation
    if cible is not None:
        # La cible est vue !
        x1, y1, x2, y2 = cible["box"]

        # Calcul du centre de la bounding box
        center_x = int((x1 + x2) / 2)

        # Calcul de l'erreur par rapport au centre de la caméra
        erreur_x = center_x - (frame_width / 2)

        # --- Contrôleur Proportionnel ---

        # Rotation (Yaw) : On tourne vers la vache
        # Le k_yaw permet d'adoucir la rotation. Ajuste-le selon les tests réels.
        k_yaw = 0.002
        yaw_rate = erreur_x * k_yaw

        # Avancement (Vx) : On avance si la vache est à peu près centrée
        vx = 0.0

        # Si la vache est au centre de l'image (+/- 80 pixels)
        if abs(erreur_x) < 80:
            if distance_avant > DISTANCE_MIN:
                vx = 0.3  # Avancer à 0.3 m/s vers l'avant
            else:
                vx = 0.0  # On est arrivé près de la vache, on stoppe

        # Envoi de la commande au Pixhawk
        # (vitesse X, vitesse Y, vitesse Z, vitesse rotation Yaw)
        drone.send_velocity_body(vx, 0, 0, yaw_rate)

    else:
        # La vache est perdue (ou pas encore trouvée)
        # On stoppe le mouvement, le drone maintient son altitude et sa position
        drone.send_velocity_body(0, 0, 0, 0)