from pymavlink import mavutil
import time


class DroneController:
    def __init__(self, connection_string="/dev/ttyAMA0", baudrate=921600):
        # Pour SITL, connection_string sera par ex "udp:127.0.0.1:14550"
        print(f"Connexion au drone sur {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.wait_heartbeat()
        print("Cible connectée !")
        self._last_lidar_alt = 0.0
    
    def request_message_interval(self, message_id, frequency_hz):
        interval_us = int(1000000 / frequency_hz)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0
        )

    def update_attitude(self):
        """ Appeler cette fonction souvent (ex: dans un thread ou une boucle) 
            pour garder les angles à jour """
        msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.roll = msg.roll   # en radians
            self.pitch = msg.pitch # en radians
            self.yaw = msg.yaw     # en radians
    
    def set_mode_safe(self, mode_id, mode_name, timeout=3.0):
        """
        Demande un changement de mode et écoute les Heartbeats 
        pour vérifier que le Pixhawk a bien accepté.
        """
        print(f"Demande de passage en mode {mode_name} (ID: {mode_id})...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
            
        start_time = time.time()
        while time.time() - start_time < timeout:
            # On attrape le prochain Heartbeat (envoyé à 1Hz par défaut)
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg:
                if msg.custom_mode == mode_id:
                    print(f"[OK] Mode {mode_name} validé et actif !")
                    return True
                    
        print(f"[ERREUR] Timeout : Le Pixhawk a refusé ou ignoré le mode {mode_name}.")
        return False

    def set_ekf_origin(self):
        """ Injecte un repère spatial pour autoriser le mode GUIDED en intérieur """
        print("Initialisation de l'origine EKF (Faux GPS)...")
        # On injecte une coordonnée fictive et 1 mètre d'altitude (1000 mm)
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(48.8566 * 1e7), int(2.3522 * 1e7), 1000
        )
        time.sleep(1.0)

    def arm_and_takeoff_guided2(self, target_altitude=0.5):
        """ Envoie juste l'ordre de décollage sans attendre """
        self.request_message_interval(173, 20) # RANGEFINDER (Lidar brut)
        self.request_message_interval(132, 20) # DISTANCE_SENSOR (Lidar brut)
        time.sleep(0.2) # Petit temps mort pour que le Pixhawk ouvre le canal
        
        #self.set_ekf_origin()      #pas besoin en ext

        if not self.set_mode_safe(4, "GUIDED"):
            return False
            
        print("Armement des moteurs...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Moteurs Armés !")
        
        # Envoi de l'ordre
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude)
        return True
    

    def get_current_alt_brute(self):
        """ 
        Écoute le flux MAVLink et retourne l'altitude brute du Lidar en mètres.
        Si aucun message n'est reçu, retourne la dernière valeur connue.
        """
        # On écoute les messages RANGEFINDER ou DISTANCE_SENSOR sans bloquer (blocking=False)
        # pour ne pas figer la boucle principale OpenCV si le Pixhawk est lent
        msg = self.master.recv_match(type=['RANGEFINDER', 'DISTANCE_SENSOR'], blocking=False)
        
        # Par défaut, on garde la dernière valeur connue (0.0 au démarrage)
        if msg:
            if msg.get_type() == 'RANGEFINDER':
                self._last_lidar_alt = msg.distance
            elif msg.get_type() == 'DISTANCE_SENSOR':
                self._last_lidar_alt = msg.current_distance / 100.0 # cm vers mètres
                
        return self._last_lidar_alt


    def arm(self):
        self.master.arducopter_arm()

    def disarm(self):
        self.master.arducopter_disarm()

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        """
        Envoie des vitesses au drone relatives à lui-même (avant, droite, bas).
        vx: vitesse vers l'avant (m/s)
        vy: vitesse vers la droite (m/s)
        vz: vitesse vers le bas (m/s) - Négatif pour monter !
        yaw_rate: rotation sur lui-même (rad/s)
        """
        # MAV_FRAME_BODY_NED : l'axe X est toujours l'avant du drone
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b01111111000111,  # Masque pour ignorer la position, n'écouter que la vitesse et yaw_rate
            0, 0, 0,  # Positions x, y, z ignorées
            vx, vy, vz,  # Vitesses
            0, 0, 0,  # Accélérations ignorées
            0, yaw_rate)
    
    def land(self):
        if not self.set_mode_safe(4, "GUIDED"):
            return False
        
        print("Atterrissage commandé !")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)
        


# Mode sans pixhawk
class DummyMaster:
    """ Imite l'objet master de pymavlink (lecture de messages) """
    def recv_match(self, type=None, blocking=False, timeout=None):
        # On simule qu'on ne reçoit aucun message
        return None


class DummyDroneController:
    """ Faux drone pour tester la vision sans matériel (même interface que DroneController) """
    def __init__(self):
        print("[MOCK] Faux Pixhawk initialisé (Mode Vision-Only)")
        self.master = DummyMaster()

    def update_attitude(self):
        pass

    def set_mode_safe(self, mode_id, mode_name, timeout=3.0):
        print(f"[MOCK] Mode {mode_name} (ID: {mode_id})")
        return True

    def arm(self):
        print("[MOCK] Commande d'armement reçue")

    def disarm(self):
        print("[MOCK] Commande de désarmement reçue")

    def arm_and_takeoff_guided2(self, target_altitude=0.5):
        print(f"[MOCK] Décollage virtuel à {target_altitude}m")
        return True

    def get_current_alt_brute(self):
        return 0.0

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        # print(f"[MOCK] Vol -> vx:{vx:.2f}, vy:{vy:.2f}, vz:{vz:.2f}, yaw:{yaw_rate:.3f}")
        pass

    def land(self):
        print("[MOCK] Atterrissage commandé")
