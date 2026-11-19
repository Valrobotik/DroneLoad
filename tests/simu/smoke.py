"""Test de fumée de la simulation : capteurs, MAVROS, décollage à 1 m (lidar), vol stationnaire, atterrissage.

Lancement : python3 tests/run.py smoke (arrête le service drone le temps du test). À la main, simu sans le
code du drone (docker compose up -d) : docker compose exec companion bash -lc 'python3 tests/simu/smoke.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import sys
import time

import rclpy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode, StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range

from commun import Etapes, Radio, attendre

TAKEOFF_ALT = 1.0      # m au-dessus du sol (lidar) : hauteur de ce test, envoyée directement à ArduPilot (pas le code du drone ni drone.yaml)
ALT_TOLERANCE = 0.2    # m
HOVER_TIME = 5.0       # s
SETTLE_TIME = 3.0      # s : le drone dépasse un peu la cible au décollage avant de se stabiliser
MAVLINK_RANGEFINDER = 173  # id du message RANGEFINDER, publié sur /mavros/rangefinder/rangefinder
# Origine EKF fictive (INSA Lyon) : sans GPS, ArduPilot en a besoin pour le mode GUIDED
ORIGIN = (45.7838, 4.8722, 170.0)


class SmokeTest(Node):
    def __init__(self):
        super().__init__("smoke_test")
        self.state = State()
        self.height = None
        self.images = {"/camera/image_raw": 0, "/stereo/depth/image_raw": 0}

        self.statustexts = []
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(StatusText, "/mavros/statustext/recv", lambda m: self.statustexts.append(m.text),
                                 qos_profile_sensor_data)   # publié en best effort par MAVROS
        self.create_subscription(Range, "/mavros/rangefinder/rangefinder", self._on_range,
                                 qos_profile_sensor_data)
        for topic in self.images:
            self.create_subscription(Image, topic, lambda _msg, t=topic: self._on_image(t),
                                     qos_profile_sensor_data)
        self.origin_pub = self.create_publisher(GeoPointStamped, "/mavros/global_position/set_gp_origin", 10)

        self.stream_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
        self.msg_interval = self.create_client(MessageInterval, "/mavros/set_message_interval")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

    def _on_state(self, msg):
        self.state = msg

    def _on_range(self, msg):
        self.height = msg.range

    def _on_image(self, topic):
        self.images[topic] += 1

    def wait_for(self, condition, timeout):
        """Fait tourner le nœud jusqu'à ce que condition() soit vraie ; False si timeout."""
        return attendre(condition, timeout, spin=lambda: rclpy.spin_once(self, timeout_sec=0.1))

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def send_origin(self):
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude, msg.position.longitude, msg.position.altitude = ORIGIN
        self.origin_pub.publish(msg)

    def gcs_guided_refused(self):
        """GUIDED demandé par MAVLink (comme la Pi) : refusé par FLTMODE_GCSBLOCK, seule la radio y passe."""
        # Réessayé : juste après le démarrage du SITL, la demande peut être perdue ou refusée pour une autre
        # raison. Le message d'ArduPilot arrive en plusieurs STATUSTEXT : textes recollés avant la recherche.
        for _ in range(10):
            self.call(self.set_mode, SetMode.Request(custom_mode="GUIDED"))
            if self.wait_for(lambda: "GCS entry disabled" in "".join(self.statustexts), 2.0):
                return self.state.mode != "GUIDED"
        return False

    def mode_and_arm(self, radio, timeout):
        """GUIDED à la radio (simulée), puis armement réessayé tant que les pré-armements d'ArduPilot
        (EKF prêt...) refusent."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.send_origin()
            if self.state.mode != "GUIDED":
                radio.guided()
            elif not self.state.armed:
                self.call(self.arming, CommandBool.Request(value=True))
            if self.wait_for(lambda: self.state.mode == "GUIDED" and self.state.armed, 2.0):
                return True
        return False

    def at_takeoff_alt(self):
        return self.height is not None and abs(self.height - TAKEOFF_ALT) < ALT_TOLERANCE

    def hover_ok(self):
        """Vrai si l'altitude reste dans la tolérance pendant HOVER_TIME."""
        return not self.wait_for(lambda: not self.at_takeoff_alt(), HOVER_TIME)


def main():
    rclpy.init()
    node = SmokeTest()
    radio = Radio()
    step = Etapes("TEST DE FUMÉE")

    try:
        if not step("MAVROS connecté au SITL", node.wait_for(lambda: node.state.connected, 60)):
            return 1
        node.call(node.stream_rate, StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
        node.call(node.msg_interval, MessageInterval.Request(message_id=MAVLINK_RANGEFINDER, message_rate=10.0))
        step("images caméra et profondeur reçues", node.wait_for(lambda: all(node.images.values()), 30))
        if not step("distance lidar reçue", node.wait_for(lambda: node.height is not None, 30)):
            return 1
        step("GUIDED refusé par MAVLink (FLTMODE_GCSBLOCK)", node.gcs_guided_refused())
        if not step("mode GUIDED (radio) et armement", node.mode_and_arm(radio, 120)):
            return 1
        result = node.call(node.takeoff, CommandTOL.Request(altitude=TAKEOFF_ALT))
        if not step("commande de décollage acceptée", result is not None and result.success):
            return 1
        step(f"altitude {TAKEOFF_ALT} m atteinte",
             node.wait_for(node.at_takeoff_alt, 30))
        node.wait_for(lambda: False, SETTLE_TIME)
        step(f"vol stationnaire {HOVER_TIME:.0f} s", node.hover_ok())
        node.call(node.set_mode, SetMode.Request(custom_mode="LAND"))
        step("atterrissage et désarmement", node.wait_for(lambda: not node.state.armed, 60))
    finally:
        radio.fermer()
        node.destroy_node()
        rclpy.shutdown()
    return step.fin()


if __name__ == "__main__":
    sys.exit(main())
