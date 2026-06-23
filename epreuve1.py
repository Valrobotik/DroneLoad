import time
import math

class Epreuve1Task:
    def __init__(self):
        self.state = "TEST_CAMERA"
        self.stream_requested = False

        self.sum_err_x = 0.0
        self.sum_err_y = 0.0
        
        # --- Paramètres Caméra ---
        self.FOV_X_DEG = 70.0 
        self.FOV_Y_DEG = 55.0 

    def compensate_camera_angles(self, raw_x, raw_y, width, height, roll_rad, pitch_rad, cam_is_down=True):
        """ Code conservé pour plus tard """
        px_per_rad_x = width / math.radians(self.FOV_X_DEG)
        px_per_rad_y = height / math.radians(self.FOV_Y_DEG)
        
        if cam_is_down:
            offset_x = roll_rad * px_per_rad_x
            offset_y = pitch_rad * px_per_rad_y
            corrected_x = raw_x + offset_x
            corrected_y = raw_y - offset_y 
            return corrected_x, corrected_y
        else:
            pass

    def run(self, drone, hw, arucos_data, frame_width, frame_height):        
        
        if not self.stream_requested:
            if hasattr(drone, 'request_message_interval'):
                print("\n[TEST] Demande du flux ATTITUDE (ID 30) à 20Hz...")
                drone.request_message_interval(30, 20) 
            self.stream_requested = True

        # 2. Mise à jour des angles (lit le buffer MAVLink via ton controlleur)
        drone.update_attitude()
        
        # Récupération sécurisée des attributs créés par ton drone.update_attitude()
        roll_rad = getattr(drone, 'roll', 0.0) 
        pitch_rad = getattr(drone, 'pitch', 0.0)

        # 2. Vérification de la détection d'ArUco
        if arucos_data: # Vérifie si le dictionnaire n'est pas vide
            
            # On extrait le premier élément du dictionnaire 
            # iter() crée un itérateur sur les paires clé/valeur, next() prend la première
            first_id, (raw_x, raw_y) = next(iter(arucos_data.items()))

            # 3. Application de la correction
            corr_x, corr_y = self.compensate_camera_angles(
                raw_x, raw_y, 
                frame_width, frame_height, 
                roll_rad, pitch_rad, 
                cam_is_down=True
            )
            
            # 4. Affichage dynamique sur une seule ligne (le \r écrase la ligne précédente)
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            
            print(f"\r[TEST] Roll: {roll_deg:+06.1f}° | Pitch: {pitch_deg:+06.1f}° || "
                  f"ID:{first_id:2d} | RAW: X={raw_x:4.0f}, Y={raw_y:4.0f} -> CORR: X={corr_x:4.0f}, Y={corr_y:4.0f}      ", 
                  end="", flush=True)
                  
        else:
            # Aucun ArUco en vue
            print("\r[TEST] En attente d'un ArUco face à la caméra...                               ", 
                  end="", flush=True)