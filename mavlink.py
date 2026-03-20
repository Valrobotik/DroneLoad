from pymavlink import mavutil
import time

def pixhawk_port():
    pass


def init_mavlink():
    print("Connexion au Pixhawk...")

    master = mavutil.mavlink_connection('COM4', baud=57600)

    # attendre le heartbeat
    master.wait_heartbeat()

    print("Heartbeat reçu")
    print("System:", master.target_system)
    print("Component:", master.target_component)

    target_system = master.target_system
    target_component = master.target_component

    return master, target_system, target_component


# -------------------------------
# 2. Fonction pour envoyer position
# -------------------------------

def move(master, target_system, target_component, xyz: list, speed: tuple, accel: tuple):
    """
    Envoie une commande de déplacement relative.

    x : avant/arrière (m)
    y : gauche/droite (m)
    z : haut/bas (m)

    Frame BODY_OFFSET_NED :
    x + = avance
    y + = droite
    y - = gauche
    z + = descend
    z - = monte
    """

    #1 c'est désactivé 0 c'est activé
    type_mask = 0b110111111000  # ignore vitesse, accel, yaw

    master.mav.set_position_target_local_ned_send(
        int(time.time()*1000),
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask,
        *xyz,
        *speed,   # vitesses
        *accel,   # accélérations
        0, 0       # yaw
    )


def drone_position(master, blocking=True, timeout=2):
    """
    Retourne la position locale du drone : (x, y, z)

    x : avant/nord selon le repère utilisé
    y : droite/est
    z : bas (en NED, z négatif = altitude au-dessus du point d'origine)

    master   : connexion pymavlink
    blocking : attend un message si True
    timeout  : temps max d'attente en secondes
    """
    msg = master.recv_match(
        type='LOCAL_POSITION_NED',
        blocking=blocking,
        timeout=timeout
    )

    if msg is None:
        return None

    return (msg.x, msg.y, msg.z)

