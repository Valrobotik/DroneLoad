"""Test de mission de bout en bout : commandes MQTT comme le dashboard, contrôle par ROS 2.

Deux cycles : takeoff (1 m au lidar) -> attente stable 5 s -> land (désarmé). Le 2e cycle vérifie
que les missions repartent du début à chaque demande.

Prérequis : simu lancée avec le code du drone (docker compose --profile drone up -d).
Usage : docker compose exec companion bash -lc 'python3 sim/scripts/mission_test.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import json
import sys
import time

import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

BROKER = ('127.0.0.1', 1883)
TAKEOFF_ALT = 1.0      # = takeoff_altitude de drone.yaml
ALT_TOLERANCE = 0.2    # m
HOLD_TIME = 5.0        # s en attente
CYCLES = 2


class MissionTest(Node):
    def __init__(self):
        super().__init__('mission_test')
        self.state = State()
        self.height = None
        self.mission_state = ''
        self.telemetry = None
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self._on_range,
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
