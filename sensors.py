# Bibliothèques fictives ou génériques pour l'exemple
# Utilise gpiozero ou RPi.GPIO pour les servos, et la lib pimoroni pour le ToF
import time
import numpy as np
#import vl53l5cx_ctypes as vl53l5cx
#from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE, RANGING_MODE_CONTINUOUS
from gpiozero import AngularServo
import gpiozero.pins.lgpio
import lgpio

class VL53L5CXReader:
    """
    Thread dédié à la lecture continue du VL53L5CX en I2C sur Raspberry Pi.
    Expose la dernière distance mesurée sans jamais bloquer l'appelant.
    """

    def __init__(self):
        self._distance_mm: float | None = None

        # Init capteur
        print("Uploading firmware, please wait...")
        self._sensor = vl53l5cx.VL53L5CX()
        print("Done!")
        self._sensor.set_resolution(8 * 8)
   
   ####
        self._sensor.set_ranging_frequency_hz(15) # 15 Hz
        self._sensor.set_ranging_mode(RANGING_MODE_CONTINUOUS)
####
        self._sensor.start_ranging()
        
        
    

    def get_distance(self) -> float | None:
        """
        Retourne la dernière distance estimée en mm.
        Non bloquant, instantané.
        """
        with self._lock:
            return self._distance_mm

    def _run(self):
        data = self._sensor.get_data()

        distances = np.array(data.distance_mm, dtype=float)
        distances = distances[distances > 0]  # éliminer les mesures nulles/invalides

        # Médiane des 64 zones → robuste au bruit et aux fausses détections
        median_mm = float(np.median(distances))


        self._distance_mm = median_mm

class HardwareManager:
    def __init__(self):
        print("Initialisation du capteur VL53L5CX et des servos...")
        # TODO: init I2C pour VL53L5CX
        def __patched_init(self, chip=None):
            gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
            chip = 0
            self._handle = lgpio.gpiochip_open(chip)
            self._chip = chip
            self.pin_class = gpiozero.pins.lgpio.LGPIOPin

        gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init
        pin_servo1=17
        pin_servo2=27

        myCorrection=0.45
        max_ang = 180.0
        min_ang = 0.0
        maxPW=(2.0+myCorrection)/1000
        minPW=(1.0-myCorrection)/1000
        
        self.servo1 = AngularServo(pin_servo1,min_angle=min_ang, max_angle=max_ang, min_pulse_width=minPW,max_pulse_width=maxPW)
        self.servo2 = AngularServo(pin_servo2,min_angle=min_ang, max_angle=max_ang, min_pulse_width=minPW,max_pulse_width=maxPW)
        
        self.t_servo1 = 0
        self.t_servo2 = 0
        self.etat_servo1="up"
        self.etat_servo2="down"

        
    def servo_1_up(self):
        global t_servo1 
        global etat_servo1
        global now_servo
        now_servo = time.time()
    
        if t_servo1 == 0:
            t_servo1 = time.time()
        if now_servo <= t_servo1 + 0.2:
            if etat_servo1 == "down":
                self.servo1.angle=10.0  
        else : 
            self.servo1.angle = None
            etat_servo1 = "up"
            t_servo1 = 0

    def servo_1_down(self):
        global t_servo1 
        global etat_servo1
        global now_servo1
        now_servo1 = time.time()
    
        if t_servo1 == 0:
            t_servo1 = time.time()
        if now_servo1 <= t_servo1 + 0.2:
            if etat_servo1 == "up":
                print("going2")
                self.servo1.angle=99.5  
        else : 
            print("check2")
            self.servo1.angle = None
            etat_servo1 = "down"
            t_servo1 = 0

    def servo_2_up(self):
        global t_servo2 
        global etat_servo2
        global now_servo2
        now_servo2 = time.time()
    
        if t_servo2 == 0:
            t_servo2 = time.time()
        if now_servo2 <= t_servo2 + 0.2:
            if etat_servo2 == "down":
                self.servo2.angle=0.0 
        else : 
            self.servo2.angle = None
            etat_servo2 = "up"
            t_servo2 = 0
    
    def servo_2_down(self):
        global t_servo2 
        global etat_servo2
        global now_servo2
        now_servo2 = time.time()
    
        if t_servo2 == 0:
            t_servo2 = time.time()
        if now_servo2 <= t_servo2 + 0.2:
            if etat_servo2 == "down":
                self.servo2.angle=90.0 
        else : 
            self.servo2.angle = None
            etat_servo2 = "up"
            t_servo2 = 0

    def get_forward_distance(self):
        """ Retourne la distance vers l'avant en mètres """
        # TODO: Lire le capteur Pimoroni
        return 2.5  # Mock: retourne 2.5 mètres par défaut

    def set_camera_pitch(self, cam_state):
        """ cam_state : -1 : caméra position haute
                        -0 : caméra position basse
        """
        #print(f"Caméra pivotée à {cam_state}°")
        if cam_state == 1:
            self.servo_1_down()
        else:
            self.servo_1_up()
    
    def set_hook(self, open_state):
        #print(f"Crochet ouvert : {open_state}")
        if open_state == 1:
            self.servo_2_up()
        if open_state == 0:
            self.servo_2_down()


manager = HardwareManager()

while True:
    manager.set_camera_pitch(1)
    manager.set_hook(0)