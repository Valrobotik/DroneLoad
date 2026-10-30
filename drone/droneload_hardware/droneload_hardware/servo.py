"""Servo du crochet (GPIO 13) : /hook/cmd (true = ouvert, false = fermé).

Avec mock:=true (simulation), le mouvement est seulement journalisé.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

PIN_CROCHET = 13
ANGLE_OUVERT = 0.0
ANGLE_FERME = 90.0
PWM_DURATION = 0.5    # s : le signal est coupé après le mouvement (évite les tremblements)


def make_servo(pin):
    """Servo gpiozero sur Pi 5 (lgpio)."""
    import gpiozero.pins.lgpio
    import lgpio
    from gpiozero import AngularServo

    # Correctif nécessaire sur Pi 5 : force l'ouverture de gpiochip0
    def patched_init(self, chip=None):
        gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
        chip = 0
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
        self.servo = None if self.mock else make_servo(PIN_CROCHET)
        self.open = False         # le crochet démarre fermé, sans mouvement au lancement
        self.moved_at = None

        self.create_subscription(Bool, '/hook/cmd', self._on_cmd, 10)
        self.create_timer(0.1, self._cut_pwm)
        self.get_logger().info('Crochet prêt' + (' (mock)' if self.mock else ''))

    def _on_cmd(self, msg):
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
