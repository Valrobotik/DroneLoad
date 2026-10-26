import time
import numpy as np
import argparse

from mavlink import DroneController, DummyDroneController
from gstream import VideoManager
from sensors import HardwareManager
# from epreuve1 import run_epreuve1
from epreuve1 import Epreuve1Task, TakeOff, Land
from epreuve2 import run_epreuve2
from mqtt_server import MqttManager
from opencv import ArucoProcessor
from yolo import YoloProcessor

from ultralytics import YOLO

IP = "192.168.31.181"
#IP = "192.168.2.9"

def parse_arguments():
    parser = argparse.ArgumentParser(description='Raspberry Pi')
    parser.add_argument('--sim', action='store_true', help='Mode Simulation (SITL/Docker)')
    parser.add_argument('--no-pixhawk', action='store_true', help='Test de la vision sans connexion MAVLink')
    return parser.parse_args()


def main():
    args = parse_arguments()

    # Choix automatique de la connexion selon l'argument du terminal
    if args.no_pixhawk:
        print("--- DÉMARRAGE EN MODE VISION UNIQUEMENT ---")
        drone = DummyDroneController()
    elif args.sim:
        print("--- DÉMARRAGE EN MODE SIMULATION (SITL) ---")
        drone = DroneController(connection_string="udp:127.0.0.1:14550")
    else:
        print("--- DÉMARRAGE EN MODE RASPBERRY PI RÉEL ---")
        drone = DroneController(connection_string="/dev/ttyAMA0")

    # 1. Initialisation Hardware & Réseau
    hw = HardwareManager()
    #drone = DroneController(connection_string=mavlink_port, baudrate=921600)
    # On passe 'drone' à MqttManager pour qu'il puisse armer/désarmer
    mqtt = MqttManager(hw, drone)
    mqtt.start(broker_ip="127.0.0.1")

   # drone = DroneController(connection_string=mavlink_port)
    video = VideoManager(ip_dest=IP, width=640, height=480)

    #modell1 = YOLO("best.pt")               #thiaw
    modell1 = YOLO("best_ncnn_model") 
    # 2. Initialisation Vision
    #aruco_vision = ArucoProcessor()
    #yolo_vision = YoloProcessor()
    epreuve1_task = Epreuve1Task(modell1)            #thiaw
    takeoff_task = TakeOff()
    land_task = Land()

    mqtt.log_to_pc("Système prêt. En attente de commandes...")

    init_detection = True

    hw.servo_2_down() 

    # 3. Boucle Principale
    while True:
        # A. Acquisition de l'image réelle
        #frame = video.get_frame()
        #if frame is None:

        canvas = video.get_frame()
        if canvas is None:
            time.sleep(0.01)
            continue

        # Création du calque de dessin (Normal = copie de l'image, Noir = image vide)
        #if mqtt.black_bg:
        #    canvas = np.zeros_like(frame)
        #else:
        #    canvas = frame.copy()

        # B. Traitement visuel (analyse sur 'frame', dessine sur 'canvas')
        #canvas, arucos_data = aruco_vision.process(frame, canvas)
        #canvas, yolo_data = yolo_vision.process(frame, canvas)

        # C. Logique de vol
        if mqtt.current_mode == "epreuve1" and mqtt.target_class is not None: #is not None ????????
            #run_epreuve1(drone, hw, arucos_data, video.width)
            if init_detection:
                epreuve1_task.tps=time.time()
                init_detection = False
            if epreuve1_task.run(drone, hw, video.width, video.height, canvas, mqtt.target_class):
                print("Cible trouvée")
                hw.state=True
                hw.servo_2_up()   
            canvas = epreuve1_task.frame              

        elif mqtt.current_mode == "epreuve2":
            # On passe les données de YOLO
            run_epreuve2(drone, hw, video.width)

        elif mqtt.current_mode == "takeoff":
            print("[MAIN] Appel de la fonction de décollage...")
            takeoff_task.run(drone, hw, video.width, video.height)

        elif mqtt.current_mode == "land":
            print("[MAIN] Appel de la fonction d'atterrissage...")
            land_task.run(drone, hw, video.width, video.height)

        elif mqtt.current_mode == "attente":
            epreuve1_task.state = "TAKEOFF"
            epreuve1_task.takeoff_done = False
            drone.send_velocity_body(0, 0, 0, 0)


        if mqtt.hook_action == "hook_open":
            print("[MAIN] Appel de la fonction d'ouverture du crochet...")
            hw.state=True
            hw.servo_2_up()             
            mqtt.hook_action = None      

        elif mqtt.hook_action == "hook_close":
            print("[MAIN] Appel de la fonction de fermeture du crochet...")
            hw.state=True
            hw.servo_2_down()
            mqtt.hook_action = None
                                



        # D. Envoi de l'image dessinée au PC via GStreamer
        video.send_frame(canvas)
        mqtt.update_telemetry()  # Calcule les FPS et envoie les données
        hw.update()


if __name__ == "__main__":
    main()
