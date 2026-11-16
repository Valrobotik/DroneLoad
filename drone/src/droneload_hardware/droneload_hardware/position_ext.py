"""Position externe : /uwb/pose (repère des ancres) -> /mavros/vision_pose/pose_cov (repère map, ENU).

La Pixhawk la reçoit en VISION_POSITION_ESTIMATE et la fusionne dans l'EKF (EK3_SRC1_POSXY 6,
config/ardupilot/uwb.param), avec la vitesse du flux optique et la hauteur du lidar.

/uwb/pose vient du pilote série du tag DWM1001 (à venir) ou, en simu, du service uwb (sim/scripts/uwb_sim.py).

- Repère : position_map = rotation(cap_ancres) x position_ancres + origine. cap_ancres = angle (degrés,
  sens trigonométrique) de l'axe x des ancres par rapport à l'est. Le cap du drone vient du compas :
  un repère des ancres mal orienté contredirait la vitesse du flux optique (spirales en stationnaire).
  En simu, le monde Gazebo est déjà en ENU : 0 et [0, 0, 0].
- Seules les mesures fraîches (moins de age_max) sont transmises : ArduPilot ne lit pas l'horodatage
  et suppose un retard fixe (VISO_DELAY_MS), une mesure en retard tirerait le drone vers une position passée.
- Perte de l'UWB (aucune mesure depuis delai_perte) et retour : journalisés (donc sur drone/logs).
- Orientation transmise telle quelle (identité : PANS ne donne pas de cap ; le cap de l'EKF vient du compas).
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class PositionExt(Node):
    def __init__(self):
        super().__init__('position_ext')
        self.declare_parameter('cap_ancres', 0.0)            # degrés
        self.declare_parameter('origine', [0.0, 0.0, 0.0])   # m, dans le repère map
        self.declare_parameter('age_max', 0.3)               # s
        self.declare_parameter('delai_perte', 1.0)           # s
        cap = math.radians(self.get_parameter('cap_ancres').value)
        self.cos, self.sin = math.cos(cap), math.sin(cap)
        self.origine = list(self.get_parameter('origine').value)
        self.age_max = self.get_parameter('age_max').value
        self.delai_perte = self.get_parameter('delai_perte').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/uwb/pose', self._on_uwb, qos_profile_sensor_data)
        self.derniere = None       # heure (horloge du nœud) de la dernière mesure transmise
        self.perdu = False
        self.perimees = 0          # mesures jetées car trop vieilles, depuis le dernier rappel
        self.debut = None          # pris au 1er tour de surveillance (en simu, l'horloge démarre au 1er /clock)
        self.create_timer(0.2, self._surveiller)

    def _on_uwb(self, msg):
        maintenant = self.get_clock().now()
        age = (maintenant - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if not -self.age_max < age < self.age_max:
            self.perimees += 1
            return
        p = msg.pose.pose.position
        x = self.cos * p.x - self.sin * p.y + self.origine[0]
        y = self.sin * p.x + self.cos * p.y + self.origine[1]
        z = p.z + self.origine[2]

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = x, y, z
        out.pose.pose.orientation = msg.pose.pose.orientation
        out.pose.covariance = self._tourner_covariance(list(msg.pose.covariance))
        self.pub.publish(out)

        if self.perdu:
            self.get_logger().info('UWB revenu')
            self.perdu = False
        self.derniere = maintenant

    def _tourner_covariance(self, cov):
        """Tourne le bloc x, y de la covariance (6x6 ligne à ligne) : C' = R C Rt."""
        c, s = self.cos, self.sin
        xx, xy, yy = cov[0], cov[1], cov[7]
        cov[0] = c * c * xx - 2 * c * s * xy + s * s * yy
        cov[7] = s * s * xx + 2 * c * s * xy + c * c * yy
        cov[1] = cov[6] = c * s * (xx - yy) + (c * c - s * s) * xy
        return cov

    def _surveiller(self):
        maintenant = self.get_clock().now()
        if self.debut is None:
            if maintenant.nanoseconds == 0:   # horloge simulée pas encore reçue
                return
            self.debut = maintenant
        reference = self.derniere or self.debut
        if not self.perdu and (maintenant - reference).nanoseconds / 1e9 > self.delai_perte:
            self.perdu = True
            raison = f' ({self.perimees} mesures trop vieilles)' if self.perimees else ''
            self.get_logger().warn(('UWB PERDU' if self.derniere else 'UWB : aucune mesure') + raison +
                                   ' : la Pixhawk vole au flux optique')
            self.perimees = 0


def main():
    rclpy.init()
    node = PositionExt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
