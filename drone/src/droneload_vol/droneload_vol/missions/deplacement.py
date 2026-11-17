import math

GAIN = 1.0   # 1/s : vitesse vers le point = GAIN x distance, plafonnée à vitesse_max (freinage à l'approche)


class Deplacement:
    """Va au point visé (x, y en m, repère map de l'EKF), à hauteur constante, puis reste en stationnaire.

    Mission qui a besoin de la position (besoin_position) : main_node ne l'appelle que si
    drone.get_position() est fiable, et la met en pause sinon. Ne bloque jamais.
    """
    besoin_position = True

    def __init__(self, log, vitesse_max=0.5, precision=0.1):
        self.log = log
        self.vitesse_max = vitesse_max
        self.precision = precision
        self.point = None
        self.reset()

    def reset(self):
        # États : SANS_POINT -> EN_ROUTE -> ARRIVE, ou ECHEC
        self.state = 'SANS_POINT' if self.point is None else 'EN_ROUTE'

    def set_point(self, x, y):
        self.point = (float(x), float(y))

    def _set_state(self, state):
        if state != self.state:
            self.state = state
            self.log.info(f'[DÉPLACEMENT] {state}')

    def run(self, drone, now):
        if self.point is None:
            self._set_state('SANS_POINT')
            drone.hover()
            return
        if self.state == 'ECHEC':
            drone.hover()
            return
        if drone.get_current_alt_brute() is None:
            # Hauteur inconnue : le drone s'arrête, il ne vole pas sans lidar
            self.log.error('[DÉPLACEMENT] ÉCHEC : hauteur lidar inconnue, stationnaire')
            self._set_state('ECHEC')
            drone.hover()
            return
        x, y = drone.get_position()   # fiable : vérifié par main_node avant l'appel
        dx, dy = self.point[0] - x, self.point[1] - y
        distance = math.hypot(dx, dy)
        if distance < self.precision:
            if self.state != 'ARRIVE':
                self.log.info(f'[DÉPLACEMENT] arrivé en ({x:.2f}, {y:.2f})')
            self._set_state('ARRIVE')
            drone.hover()
            return
        if self.state == 'ARRIVE' and distance < 2 * self.precision:
            # Petite dérive après l'arrivée : on reste ARRIVE et on corrige sans repartir
            pass
        else:
            self._set_state('EN_ROUTE')
        v = min(self.vitesse_max, GAIN * distance)
        drone.send_velocity_local(v * dx / distance, v * dy / distance, 0)
