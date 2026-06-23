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

    def arm_and_takeoff_hybrid(self, target_altitude=0.5):
        """ 
        Décollage forcé au Thrust (GUIDED_NOGPS) puis bascule en Vitesse (GUIDED)
        """
        # 1. Forcer le flux du Lidar brut (173 = RANGEFINDER, 132 = DISTANCE_SENSOR)
        print("Demande du flux Lidar brut...")
        self.request_message_interval(173, 20)
        self.request_message_interval(132, 20)
        time.sleep(0.5)

        # 2. Passage en GUIDED_NOGPS pour ignorer les sécurités EKF au sol
        print("Passage en mode GUIDED_NOGPS (Mode 20)...")
        if not self.set_mode_safe(20, "GUIDED_NOGPS"):
            print("Décollage annulé.")
            return

        # 3. Armement
        print("Armement des moteurs...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Moteurs Armés !")

        print(f"Décollage au Thrust vers {target_altitude}m...")
        
        while True:
            # Envoi d'une poussée constante (ajuste entre 0.55 et 0.65 selon le poids)
            thrust_value = 0.53 
            self.master.mav.set_attitude_target_send(
                0, self.master.target_system, self.master.target_component,
                0b00000111, [1, 0, 0, 0], 0, 0, 0, thrust_value
            )

            # Lecture exclusive du Lidar Brut
            msg = self.master.recv_match(type=['RANGEFINDER', 'DISTANCE_SENSOR'], blocking=True, timeout=0.1)
            current_alt = 0.0
            
            if msg:
                if msg.get_type() == 'RANGEFINDER':
                    current_alt = msg.distance
                elif msg.get_type() == 'DISTANCE_SENSOR':
                    current_alt = msg.current_distance / 100.0
            
            # Affichage de validation
            print(f"Altitude Lidar Brut : {current_alt:.2f} m")
            
            # Dès que la cible est atteinte, on coupe la boucle
            if current_alt >= target_altitude:
                print(f"Altitude atteinte ({current_alt:.2f}m) !")
                break 
                
            time.sleep(0.05)

        # 4. BASCULE EN MODE GUIDED POUR L'IA
        print("Repassage en mode GUIDED (Mode 4) pour le pilotage en VITESSE...")
        if not self.set_mode_safe(4, "GUIDED"):
            print("Passage Guided failed")
            return
        time.sleep(0.5)
        
        # 5. Envoi d'une commande de vitesse nulle (Hover) pour stabiliser
        self.send_velocity_body(0, 0, 0, 0)
        print("Décollage terminé ! Le drone est en stationnaire et attend les vitesses de l'IA.")
        
    

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
    

    def arm_and_takeoff_guided(self, target_altitude=0.5):
        """ 
        Décollage avec la commande officielle NAV_TAKEOFF en mode GUIDED.
        Le drone utilise son propre algorithme de décollage avec le flux optique.
        """
        # 1. On donne un repère à l'EKF
        #self.set_ekf_origin()          #pas besoin en ext
        
        # 2. Passage en mode GUIDED (Mode 4)
        if not self.set_mode_safe(4, "GUIDED"):
            print("Erreur : Impossible de passer en GUIDED. Annulation.")
            return

        # 3. Armement des moteurs
        print("Armement des moteurs...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Moteurs Armés !")

        # 4. COMMANDE OFFICIELLE DE DÉCOLLAGE
        print(f"Envoi de l'ordre de décollage autonome vers {target_altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude)
        
        # 5. Boucle de surveillance de l'altitude
        while True:
            # On écoute la vraie hauteur du Lidar
            msg = self.master.recv_match(type=['RANGEFINDER', 'DISTANCE_SENSOR'], blocking=True, timeout=0.1)
            current_alt = 0.0
            
            if msg:
                if msg.get_type() == 'RANGEFINDER':
                    current_alt = msg.distance
                elif msg.get_type() == 'DISTANCE_SENSOR':
                    current_alt = msg.current_distance / 100.0
            
            print(f"Altitude : {current_alt:.2f} m")
            
            # Si on atteint la cible (avec 5% de marge pour l'inertie)
            if current_alt >= target_altitude * 0.95:
                print(f"Altitude de {target_altitude}m atteinte !")
                break 
                
            time.sleep(0.1)

        print("Décollage terminé ! Le drone est verrouillé sur place et prêt pour l'IA.")

    def get_current_alt_brute(self):
        """ 
        Écoute le flux MAVLink et retourne l'altitude brute du Lidar en mètres.
        Si aucun message n'est reçu, retourne une valeur par défaut (-1.0).
        """
        # On écoute les messages RANGEFINDER ou DISTANCE_SENSOR sans bloquer (blocking=False)
        # pour ne pas figer la boucle principale OpenCV si le Pixhawk est lent
        msg = self.master.recv_match(type=['RANGEFINDER', 'DISTANCE_SENSOR'], blocking=False)
        
        # Par défaut, on garde la dernière valeur connue ou une valeur négative si inconnu
        # Pour faire propre, on peut stocker ça dans une variable d'instance initialisée à 0.0 dans __init__
        if not hasattr(self, '_last_lidar_alt'):
            self._last_lidar_alt = 0.0

        if msg:
            if msg.get_type() == 'RANGEFINDER':
                self._last_lidar_alt = msg.distance
            elif msg.get_type() == 'DISTANCE_SENSOR':
                self._last_lidar_alt = msg.current_distance / 100.0 # cm vers mètres
                
        return self._last_lidar_alt


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
        print("Atterrissage commandé !")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)
        


# Mode sans pixhawk
class DummyMaster:
    """ Imite l'objet master de pymavlink pour que MQTT ne plante pas """
    def arducopter_arm(self):
        print("[MOCK] Commande d'armement reçue")
        
    def arducopter_disarm(self):
        print("[MOCK] Commande de désarmement reçue")
        
    def recv_match(self, type, blocking=False):
        # On simule qu'on ne reçoit aucune info batterie
        return None

class DummyDroneController:
    """ Faux drone pour tester la vision sans matériel """
    def __init__(self):
        print("[MOCK] Faux Pixhawk initialisé (Mode Vision-Only)")
        self.master = DummyMaster()

    def arm_and_takeoff_hybrid(self, target_altitude):
        print(f"[MOCK] Décollage virtuel à {target_altitude}m")

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        # Optionnel : Tu peux décommenter le print ci-dessous pour voir 
        # dans le terminal quelles décisions de vol l'IA prend.
        # Mais ça risque de spammer ton terminal à 15 FPS !
        # print(f"[MOCK] Vol -> vx:{vx:.2f}, vy:{vy:.2f}, vz:{vz:.2f}, yaw:{yaw_rate:.3f}")
        pass