"""Test de mission de bout en bout : commandes MQTT comme le dashboard, contrôle par ROS 2.

Deux cycles : takeoff (1 m au lidar) -> attente stable 5 s -> land (désarmé). Le 2e cycle vérifie
que les missions repartent du début à chaque demande. Pendant l'attente du 1er cycle, le drone est
au-dessus de l'ArUco 0 (au sol) : sa pose (nœud aruco) doit donner la hauteur du lidar.

Prérequis : simu lancée avec le code du drone (docker compose --profile drone up -d), drone au
départ sur l'ArUco 0 (sim/scripts/reset_drone.sh : il atterrit là où il est et dérive d'un test à l'autre).
Usage : docker compose exec companion bash -lc 'python3 sim/scripts/mission_test.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import json
import sys
import time

import numpy as np
import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

BROKER = ('127.0.0.1', 1883)
TAKEOFF_ALT = 1.0      # = takeoff_altitude de drone.yaml
ALT_TOLERANCE = 0.2    # m
HOLD_TIME = 5.0        # s en attente
CYCLES = 2
ARUCO_ID = '0'           # ArUco au sol sous le drone au départ (sim/worlds/voliere.sdf)
ARUCO_Z = 0.012          # m, hauteur du marqueur au-dessus du sol
ARUCO_TOLERANCE = 0.05   # m, écart toléré entre la distance au marqueur et la hauteur lidar
ARUCO_MAX_OFFSET = 0.25  # m, décalage horizontal maximal (le marqueur sort du champ au-delà)


class MissionTest(Node):
    def __init__(self):
        super().__init__('mission_test')
        self.state = State()
        self.height = None
        self.mission_state = ''
        self.telemetry = None
        self.aruco = None
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self._on_range,
                                 qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, '/aruco/detections', self._on_aruco,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/state', self._on_mission, latched)

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'mission_test')
        self.mqtt.on_message = self._on_mqtt
        self.mqtt.connect(*BROKER)
        self.mqtt.subscribe([('drone/telemetry', 0), ('drone/logs', 0)])
        self.mqtt.loop_start()

    def _on_state(self, msg):
        self.state = msg

    def _on_range(self, msg):
        self.height = msg.range

    def _on_aruco(self, msg):
        for det in msg.detections:
            if det.results[0].hypothesis.class_id == ARUCO_ID:
                self.aruco = det.results[0].pose.pose.position

    def aruco_matches_lidar(self):
        """ArUco 0 vu, à la distance attendue d'après le lidar, à peu près sous le drone."""
        self.aruco = None
        self.wait_for(lambda: self.aruco is not None, 3.0)
        if self.aruco is None or self.height is None:
            print('    ArUco 0 non vu', flush=True)
            return False
        p = self.aruco
        expected = self.height - ARUCO_Z
        print(f'    ArUco 0 : x {p.x:+.3f} y {p.y:+.3f} z {p.z:.3f} m '
              f'(attendu z ≈ {expected:.3f} m d\'après le lidar)', flush=True)
        return abs(p.z - expected) < ARUCO_TOLERANCE and np.hypot(p.x, p.y) < ARUCO_MAX_OFFSET

    def _on_mission(self, msg):
        if msg.data != self.mission_state:
            print(f'    état mission : {msg.data}', flush=True)
        self.mission_state = msg.data

    def _on_mqtt(self, client, userdata, msg):
        if msg.topic == 'drone/telemetry':
            self.telemetry = json.loads(msg.payload)
        else:
            print(f'    log : {msg.payload.decode()}', flush=True)

    def set_mode(self, mode):
        self.mqtt.publish('drone/cmd', json.dumps({'action': 'set_mode', 'mode': mode}))

    def wait_for(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return True
        return False

    def at_alt(self):
        return self.height is not None and abs(self.height - TAKEOFF_ALT) < ALT_TOLERANCE

    def holds_alt(self, duration):
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not (self.state.armed and self.at_alt()):
                print(f'    hauteur {self.height:.2f} m après {time.monotonic() - start:.1f} s, '
                      f'armé : {self.state.armed}, mode : {self.state.mode}', flush=True)
                return False
        return True


def main():
    rclpy.init()
    node = MissionTest()
    steps = []

    def step(name, ok):
        print(f"[{'OK' if ok else 'ÉCHEC'}] {name}", flush=True)
        steps.append(ok)
        return ok

    try:
        if not step('télémétrie MQTT du drone reçue', node.wait_for(lambda: node.telemetry is not None, 30)):
            return 1
        if not step('MAVROS connecté', node.wait_for(lambda: node.state.connected, 60)):
            return 1
        step('vision active (fps > 0 dans la télémétrie)',
             node.wait_for(lambda: node.telemetry['fps'] > 0, 60))
        for cycle in range(1, CYCLES + 1):
            node.set_mode('takeoff')
            if not step(f'cycle {cycle} : décollage à {TAKEOFF_ALT} m',
                        node.wait_for(lambda: node.state.armed and node.at_alt(), 150)):
                return 1
            node.wait_for(lambda: False, 3.0)   # le drone dépasse un peu la cible avant de se stabiliser
            node.set_mode('attente')
            step(f'cycle {cycle} : attente stable {HOLD_TIME:.0f} s', node.holds_alt(HOLD_TIME))
            if cycle == 1:
                step('ArUco 0 : pose 3D cohérente avec le lidar', node.aruco_matches_lidar())
            node.set_mode('land')
            if not step(f'cycle {cycle} : atterrissage et désarmement',
                        node.wait_for(lambda: not node.state.armed, 60)):
                return 1
            node.set_mode('attente')
            node.wait_for(lambda: False, 5.0)   # ArduPilot refuse de réarmer juste après l'atterrissage
    finally:
        node.mqtt.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

    print('TEST DE MISSION :', 'OK' if all(steps) else 'ÉCHEC')
    return 0 if all(steps) else 1


if __name__ == '__main__':
    sys.exit(main())
