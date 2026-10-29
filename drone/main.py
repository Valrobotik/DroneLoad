import time
import argparse

from ultralytics import YOLO

from . import config
from .mavlink import DroneController, DummyDroneController
from .video import VideoManager
from .hardware import HardwareManager
from .tasks.epreuve1 import Epreuve1Task, TakeOff, Land
from .tasks.epreuve2 import run_epreuve2
from .mqtt_server import MqttManager


def parse_arguments():
    parser = argparse.ArgumentParser(description='Raspberry Pi')
    parser.add_argument('--sim', action='store_true', help='Mode Simulation (SITL/Docker)')
    parser.add_argument('--no-pixhawk', action='store_true', help='Test de la vision sans connexion MAVLink')
    parser.add_argument('--pc-ip', default=config.PC_IP, help='IP du PC qui reçoit le flux vidéo')
    return parser.parse_args()


def main():
    args = parse_arguments()

    # Choix automatique de la connexion selon l'argument du terminal
    if args.no_pixhawk:
        print("--- DÉMARRAGE EN MODE VISION UNIQUEMENT ---")
        drone = DummyDroneController()
    elif args.sim:
        print("--- DÉMARRAGE EN MODE SIMULATION (SITL) ---")
        drone = DroneController(connection_string=config.SITL_URL)
    else:
        print("--- DÉMARRAGE EN MODE RASPBERRY PI RÉEL ---")
        drone = DroneController(connection_string=config.SERIAL_PORT, baudrate=config.BAUDRATE)

    # 1. Initialisation Hardware & Réseau
    hw = HardwareManager()
    # On passe 'drone' à MqttManager pour qu'il puisse armer/désarmer
    mqtt = MqttManager(hw, drone)
    mqtt.start(broker_ip=config.BROKER_IP, port=config.BROKER_PORT)

    video = VideoManager(ip_dest=args.pc_ip, port=config.VIDEO_PORT,
                         width=config.WIDTH, height=config.HEIGHT, fps=config.FPS)

    # 2. Initialisation Vision
    model = YOLO(str(config.MODEL_PATH))
    epreuve1_task = Epreuve1Task(model)
    takeoff_task = TakeOff()
    land_task = Land()

    mqtt.log_to_pc("Système prêt. En attente de commandes...")

    init_detection = True

    hw.servo_2_down()

    # 3. Boucle Principale
    while True:
        # A. Acquisition de l'image réelle
        canvas = video.get_frame()
        if canvas is None:
            time.sleep(0.01)
            continue

        # B. Logique de vol
        if mqtt.current_mode == "epreuve1" and mqtt.target_class is not None:
            if init_detection:
                epreuve1_task.tps = time.time()
                init_detection = False
            if epreuve1_task.run(drone, hw, video.width, video.height, canvas, mqtt.target_class):
                print("Cible trouvée")
                hw.servo_2_up()
            canvas = epreuve1_task.frame

        elif mqtt.current_mode == "epreuve2":
            # TODO: aucun détecteur ne fournit encore yolo_data -> le drone reste en stationnaire
            run_epreuve2(drone, hw, [], video.width)

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

        # C. Crochet
        if mqtt.hook_action == "hook_open":
            print("[MAIN] Appel de la fonction d'ouverture du crochet...")
            hw.servo_2_up()
            mqtt.hook_action = None

        elif mqtt.hook_action == "hook_close":
            print("[MAIN] Appel de la fonction de fermeture du crochet...")
            hw.servo_2_down()
            mqtt.hook_action = None

        # D. Envoi de l'image dessinée au PC via GStreamer
        video.send_frame(canvas)
        mqtt.update_telemetry()  # Calcule les FPS et envoie les données
        hw.update()


if __name__ == "__main__":
    main()
