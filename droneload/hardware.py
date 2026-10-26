import time
from gpiozero import AngularServo
import gpiozero.pins.lgpio
import lgpio


class HardwareManager:
    def __init__(self):
        print("Initialisation des servos...")

        # Correctif nécessaire sur Pi 5 : force l'ouverture de gpiochip0
        def __patched_init(self, chip=None):
            gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
            chip = 0
            self._handle = lgpio.gpiochip_open(chip)
            self._chip = chip
            self.pin_class = gpiozero.pins.lgpio.LGPIOPin

        gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init
        pin_servo1 = 17  # caméra
        pin_servo2 = 13  # crochet

        myCorrection = 0.45
        max_ang = 180.0
        min_ang = 0.0
        maxPW = (2.0 + myCorrection) / 1000
        minPW = (1.0 - myCorrection) / 1000

        self.servo1 = AngularServo(pin_servo1, min_angle=min_ang, max_angle=max_ang, min_pulse_width=minPW, max_pulse_width=maxPW)
        self.servo2 = AngularServo(pin_servo2, min_angle=min_ang, max_angle=max_ang, min_pulse_width=minPW, max_pulse_width=maxPW)

        self.servo1.angle = None
        self.servo2.angle = None

        self.t_servo1 = 0
        self.t_servo2 = 0
        self.etat_servo1 = "up"
        self.etat_servo2 = "down"

    def update(self):
        """ Gère la coupure du signal PWM pour éviter les tremblements """
        # Coupe le signal du servo 1 après 0.5s
        if self.t_servo1 != 0 and time.time() > self.t_servo1 + 0.5:
            self.servo1.angle = None
            self.t_servo1 = 0

        # Coupe le signal du servo 2 après 0.5s
        if self.t_servo2 != 0 and time.time() > self.t_servo2 + 0.5:
            self.servo2.angle = None
            self.t_servo2 = 0

    # --- Servo 2 : crochet ---
    def servo_2_up(self):
        if self.etat_servo2 != "up":
            self.servo2.angle = 0.0
            self.t_servo2 = time.time()
            self.etat_servo2 = "up"

    def servo_2_down(self):
        if self.etat_servo2 != "down":
            self.servo2.angle = 90.0
            self.t_servo2 = time.time()
            self.etat_servo2 = "down"

    # --- Servo 1 : caméra (à appeler à chaque tick jusqu'à la fin du mouvement) ---
    def servo_1_up(self):
        now = time.time()
        if self.t_servo1 == 0:
            self.t_servo1 = now
        if now <= self.t_servo1 + 0.2:
            if self.etat_servo1 == "down":
                self.servo1.angle = 10.0
        else:
            self.servo1.angle = None
            self.etat_servo1 = "up"
            self.t_servo1 = 0

    def servo_1_down(self):
        now = time.time()
        if self.t_servo1 == 0:
            self.t_servo1 = now
        if now <= self.t_servo1 + 0.2:
            if self.etat_servo1 == "up":
                self.servo1.angle = 99.5
        else:
            self.servo1.angle = None
            self.etat_servo1 = "down"
            self.t_servo1 = 0

    def set_camera_pitch(self, cam_state):
        """ cam_state : 1 -> servo_1_down (99.5°), sinon servo_1_up (10°) """
        if cam_state == 1:
            self.servo_1_down()
        else:
            self.servo_1_up()

    def get_forward_distance(self):
        """ Retourne la distance vers l'avant en mètres """
        # TODO: aucun capteur frontal (VL53L5CX non fonctionnel), valeur fictive
        return 2.5
