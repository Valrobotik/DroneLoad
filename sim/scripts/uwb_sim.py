"""Capteur UWB simulé (tag DWM1001 sous firmware PANS) : publie /uwb/pose comme le fera le vrai pilote série.

Lancé par le service uwb de sim/compose.yml (par chemin, sans colcon build). Part de la vérité terrain
de Gazebo (/sim/drone/pose, sim/bridge.yaml) et la dégrade comme le vrai système :
- 10 Hz (cadence maximale de PANS) ;
- bruit gaussien : 0,10 m en x et y, 0,30 m en z (le z de l'UWB est mauvais, le drone vole au lidar) ;
- retard d'environ 50 ms (VISO_DELAY_MS de config/ardupilot/uwb.param) ;
- environ 2 % de mesures perdues ;
- orientation à l'identité : PANS ne donne pas de cap.

Repère : monde Gazebo (ENU, origine au centre de la volière). En réel, ce sera le repère des ancres ;
le nœud position_ext du drone le ramène dans le repère map de MAVROS.

Panne simulée : ros2 param set /uwb_sim actif false (puis true pour la fin de la panne).
Sauts simulés (ancre masquée, reflet) : ros2 param set /uwb_sim taux_sauts 0.3 (proportion de mesures
décalées de SAUT m en x ; 0 pour arrêter).
"""
import collections
import random

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

PERIODE = 0.1        # s (10 Hz)
BRUIT_XY = 0.10      # m, écart type
BRUIT_Z = 0.30       # m
RETARD = 0.05        # s
PERTES = 0.02        # proportion de mesures perdues
SAUT = 2.0           # m, décalage d'une mesure fausse (paramètre taux_sauts)


class UwbSim(Node):
    def __init__(self):
        super().__init__('uwb_sim', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('actif', True)
        self.declare_parameter('taux_sauts', 0.0)
        self.actif = True
        self.taux_sauts = 0.0
        self.add_on_set_parameters_callback(self._on_params)
        self.verite = None
        self.file = collections.deque()   # mesures en attente de leur retard : (heure de publication, message)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/pose', 10)
        self.create_subscription(PoseStamped, '/sim/drone/pose', self._on_verite, qos_profile_sensor_data)
        self.create_timer(PERIODE, self._mesure)
        self.create_timer(0.01, self._publier)
        self.get_logger().info('UWB simulé : /sim/drone/pose -> /uwb/pose (10 Hz, bruit 0,10 m)')

    def _on_params(self, params):
        for p in params:
            if p.name == 'actif':
                self.actif = bool(p.value)
                self.get_logger().warn('UWB simulé ' + ('rétabli' if self.actif else 'EN PANNE'))
            elif p.name == 'taux_sauts':
                self.taux_sauts = float(p.value)
                self.get_logger().warn(f'UWB simulé : {100 * self.taux_sauts:.0f} % de mesures décalées de {SAUT} m')
        return SetParametersResult(successful=True)

    def _on_verite(self, msg):
        self.verite = msg

    def _mesure(self):
        if self.verite is None or not self.actif or random.random() < PERTES:
            return
        maintenant = self.get_clock().now()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = maintenant.to_msg()
        msg.header.frame_id = 'uwb'
        p = self.verite.pose.position
        saut = SAUT if random.random() < self.taux_sauts else 0.0
        msg.pose.pose.position.x = p.x + saut + random.gauss(0.0, BRUIT_XY)
        msg.pose.pose.position.y = p.y + random.gauss(0.0, BRUIT_XY)
        msg.pose.pose.position.z = p.z + random.gauss(0.0, BRUIT_Z)
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = BRUIT_XY ** 2
        cov[14] = BRUIT_Z ** 2
        cov[21] = cov[28] = cov[35] = 1e6   # orientation inconnue
        msg.pose.covariance = cov
        self.file.append((maintenant.nanoseconds + int(RETARD * 1e9), msg))

    def _publier(self):
        maintenant = self.get_clock().now().nanoseconds
        while self.file and self.file[0][0] <= maintenant:
            self.pub.publish(self.file.popleft()[1])


def main():
    rclpy.init()
    node = UwbSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
