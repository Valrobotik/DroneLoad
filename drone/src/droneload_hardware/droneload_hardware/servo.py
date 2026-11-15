"""Servo du crochet (GPIO 13) : /hook/cmd (true = ouvert, false = fermé).

Publie /hook/ready chaque seconde (true si le servo est utilisable), affiché sur le dashboard.
GPIO indisponible (puce introuvable, broche prise par un autre programme) : le nœud reste lancé, le
signale sur /rosout et réessaie ; une commande reçue entre-temps est refusée (pas exécutée plus tard :
le drone aurait bougé).
Avec mock:=true (simulation), le mouvement est seulement journalisé.
"""
import glob
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

PIN_CROCHET = 13
ANGLE_OUVERT = 0.0
ANGLE_FERME = 90.0
PWM_DURATION = 0.5    # s : le signal est coupé après le mouvement (évite les tremblements)
RP1_LABEL = 'pinctrl-rp1'   # puce des GPIO du connecteur sur Pi 5 (gpiochip0 ou gpiochip4 selon le noyau)
RETRY_PERIOD = 5.0    # s entre deux tentatives d'ouverture du GPIO
ERROR_THROTTLE = 30.0  # s entre deux rappels de la même erreur


def rp1_chip(lgpio):
    """Numéro de la puce GPIO du connecteur, cherchée par son nom : son numéro change selon le noyau
    et une autre puce accepterait la broche 13 sans erreur (crochet immobile, rien de signalé)."""
    labels = {}
    for path in sorted(glob.glob('/dev/gpiochip[0-9]*')):
        chip = int(path.removeprefix('/dev/gpiochip'))
        handle = lgpio.gpiochip_open(chip)
        try:
            labels[chip] = lgpio.gpio_get_chip_info(handle)[3]
        finally:
            lgpio.gpiochip_close(handle)
        if labels[chip] == RP1_LABEL:
            return chip
    raise RuntimeError(f'puce GPIO {RP1_LABEL} introuvable (puces : {labels or "aucune"})')


def make_servo(pin):
    """Servo gpiozero sur Pi 5 (lgpio)."""
    import gpiozero.pins.lgpio
    import lgpio
    from gpiozero import AngularServo

    chip = rp1_chip(lgpio)

    # Correctif nécessaire sur Pi 5 : gpiozero ne trouve pas seul la puce du connecteur
    def patched_init(self, chip=chip):
        gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
        self._handle = lgpio.gpiochip_open(chip)
        self._chip = chip
        self.pin_class = gpiozero.pins.lgpio.LGPIOPin

    gpiozero.pins.lgpio.LGPIOFactory.__init__ = patched_init

    correction = 0.45
    servo = AngularServo(pin, min_angle=0.0, max_angle=180.0,
                         min_pulse_width=(1.0 - correction) / 1000,
                         max_pulse_width=(2.0 + correction) / 1000)
    servo.angle = None
    return servo


class Servo(Node):
    def __init__(self):
        super().__init__('servo')
        self.declare_parameter('mock', False)
        self.mock = self.get_parameter('mock').value
        self.servo = None
        # Position inconnue au lancement (crochet resté ouvert après un redémarrage du code) : pas de
        # mouvement au démarrage, mais la 1re commande, ouverte ou fermée, est toujours exécutée
        self.open = None
        self.moved_at = None

        self.ready_pub = self.create_publisher(Bool, '/hook/ready', 10)
        self.create_subscription(Bool, '/hook/cmd', self._on_cmd, 10)
        self.create_timer(0.1, self._cut_pwm)
        self.create_timer(1.0, self._publish_ready)
        if self.mock:
            self.get_logger().info('Crochet prêt (mock)')
        else:
            self._open_gpio()
            self.create_timer(RETRY_PERIOD, self._open_gpio)

    @property
    def ready(self):
        return self.mock or self.servo is not None

    def _open_gpio(self):
        if self.servo is not None:
            return
        try:
            self.servo = make_servo(PIN_CROCHET)
        except Exception as e:   # lgpio.error, GPIOZeroError, RuntimeError : tous signalés pareil
            self.get_logger().error(f'Crochet INUTILISABLE (GPIO {PIN_CROCHET}) : {e} ; '
                                    f'nouvel essai toutes les {RETRY_PERIOD:.0f} s',
                                    throttle_duration_sec=ERROR_THROTTLE)
            return
        self.get_logger().info(f'Crochet prêt (GPIO {PIN_CROCHET})')

    def _publish_ready(self):
        self.ready_pub.publish(Bool(data=self.ready))

    def _on_cmd(self, msg):
        if not self.ready:
            self.get_logger().error('Commande du crochet REFUSÉE : GPIO indisponible', throttle_duration_sec=5.0)
            return
        if msg.data == self.open:
            return
        self.open = msg.data
        self.get_logger().info('Crochet ouvert' if self.open else 'Crochet fermé')
        if self.servo is not None:
            self.servo.angle = ANGLE_OUVERT if self.open else ANGLE_FERME
            self.moved_at = time.monotonic()

    def _cut_pwm(self):
        if self.moved_at is not None and time.monotonic() - self.moved_at > PWM_DURATION:
            self.servo.angle = None
            self.moved_at = None


def main():
    rclpy.init()
    node = Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
