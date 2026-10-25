import time

class Epreuve1Task:
    def __init__(self, modell):
        # États possibles : "TAKEOFF", "SEARCH", "CENTER", "LANDED"
        self.state = "TAKEOFF"
        self.takeoff_done = False

        self.modell = modell
        self.last_time = time.time()
        self.sum_err_x = 0.0
        self.sum_err_y = 0.0

        # --- Paramètres Caméra (À AJUSTER selon ton matériel) ---
        self.FOV_X_DEG = 70.0 # Angle de vue horizontal de ta caméra (ex: 70°)
        self.FOV_Y_DEG = 55.0 # Angle de vue vertical de ta caméra (ex: 55°)

    def compensate_camera_angles(self, raw_x, raw_y, width, height, roll_rad, pitch_rad, cam_is_down=True):
        """
        Corrige la position (x, y) de la cible en fonction de l'inclinaison du drone.
        Retourne (corrected_x, corrected_y).
        """
        # 1. Calcul du ratio : Combien de pixels représente 1 radian d'inclinaison ?
        px_per_rad_x = width / math.radians(self.FOV_X_DEG)
        px_per_rad_y = height / math.radians(self.FOV_Y_DEG)
        
        if cam_is_down:
            # Caméra regarde le sol
            
            offset_x = roll_rad * px_per_rad_x
            offset_y = pitch_rad * px_per_rad_y
            
            corrected_x = raw_x + offset_x
            corrected_y = raw_y - offset_y # Le signe dépend du repère MAVLink vs OpenCV
            
            return corrected_x, corrected_y
        
        else:
            # Caméra vers l'avant (Plus complexe, gère le lacet)
            # ... (À implémenter compléter !!!!!!!!!!!!!!!!!!!!!!!!!)
            pass

    def is_only_class_detected(self , frame, target_class: str,confidence_threshold: float = 0.5) -> bool:

        results = self.modell(frame, verbose=False)
        result = results[0]

        if len(result.boxes) == 0:
            return False

        confidences = result.boxes.conf.tolist()
        class_ids = [int(c) for c in result.boxes.cls.tolist()]

        detected_classes = set()

        for conf, cls_id in zip(confidences, class_ids):
            if conf >= confidence_threshold:
                class_name = result.names[cls_id]
                detected_classes.add(class_name)

        # Vérifie que target_class est présente ET que c'est la SEULE classe
        return detected_classes == {target_class}

    def run(self, drone, hw, arucos_data, frame_width, frame_height, frame,classe, t):

        if self.is_only_class_detected(frame, classe, 0.5):
            t = t +1
        if t >= 25:
            self.controle_servo(0)
            return 0
        return t

