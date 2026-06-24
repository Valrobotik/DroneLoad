import time
import cv2

class TakeOff:
    def __init__(self):
        # États possibles : "TAKEOFF", "SEARCH", "CENTER", "LANDED"
        self.state = "TAKEOFF"
        self.takeoff_done = False

        self.last_time = time.time()
        
    def run(self, drone, hw, frame_width, frame_height):
        ALTITUDE_CIBLE = 1 # 1=100+25 cm

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # S'assurer qu'on a les derniers angles du drone
        drone.update_attitude()
        
        # ---------------------------------------------------------
        # ETAT 1 : DÉCOLLAGE
        # ---------------------------------------------------------
        
        if self.state == "TAKEOFF":
            print(f"Décollage demandé à {ALTITUDE_CIBLE}m (Absolu Lidar)...")
            # On lance l'action une seule fois
            if drone.arm_and_takeoff_guided2(ALTITUDE_CIBLE):
                self.state = "TAKEOFF_MONITORING"
                
        elif self.state == "TAKEOFF_MONITORING":
            # Cette zone est lue en boucle (à chaque frame caméra) sans bloquer le script
            current_alt = drone.get_current_alt_brute() # Via RANGEFINDER
            print(f"\r[TAKEOFF] Lidar: {current_alt:.2f}m | Cible: {ALTITUDE_CIBLE:.2f}m", end="", flush=True)

            # Vérification avec tolérance
            if current_alt >= (ALTITUDE_CIBLE - 0.05):
                print("Altitude atteinte ! Passage à la recherche.")
                self.state = "SEARCH"

                
        elif self.state == "SEARCH":
            drone.send_velocity_body(0, 0, 0, 0)


class Land:
    def __init__(self):
        # États possibles : "TAKEOFF", "SEARCH", "CENTER", "LANDED"
        self.state = "LAND"
        self.takeoff_done = False

        self.last_time = time.time()
        
    def run(self, drone, hw, frame_width, frame_height):
        if self.state == "LAND":
            drone.send_velocity_body(0, 0, 0, 0)
            drone.land()
            self.state = "LANDED"
        
        elif self.state == "SEARCH":
            pass



class Epreuve1Task:
    def __init__(self, modell):
        # États possibles : "TAKEOFF", "SEARCH", "CENTER", "LANDED"
        self.state = "TAKEOFF"
        self.takeoff_done = False

        self.modell = modell

        self.tps = time.time()
        self.fps=0
        self.t=0

        self.frame = None

    def is_only_class_detected(self, target_class: str,confidence_threshold: float = 0.5) -> bool:
        if self.frame is None:
            return False
        
        results = self.modell(self.frame, verbose=False)
        result = results[0]

        if len(result.boxes) == 0:
            return False

        confidences = result.boxes.conf.tolist()
        class_ids = [int(c) for c in result.boxes.cls.tolist()]

        boxes = result.boxes.xyxy.tolist()

        detected_classes = set()

        for conf, cls_id, box in zip(confidences, class_ids, boxes):
            if conf >= confidence_threshold:
                class_name = result.names[cls_id]
                detected_classes.add(class_name)

                color = (0, 255, 0) if class_name == target_class else (0, 0, 255)
                x1, y1, x2, y2 = map(int, box)
                
                # Dessine le rectangle
                cv2.rectangle(self.frame, (x1, y1), (x2, y2), color, 2)
                
                # Ajoute le texte (Nom + Confiance)
                label = f"{class_name} {conf:.2f}"
                cv2.putText(self.frame, label, (x1, max(y1 - 10, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Vérifie que target_class est présente ET que c'est la SEULE classe
        return detected_classes == {target_class}

    def run(self, drone, hw, frame_width, frame_height, frame, classe):
        self.frame = frame
        if self.is_only_class_detected(classe, 0.5):
            self.t+=1
        self.fps+=1
        if time.time()-self.tps>=2:
            if self.t >= self.fps*0.8:
                return True
            self.tps=time.time()
            self.t=0
            self.fps=0
        return False