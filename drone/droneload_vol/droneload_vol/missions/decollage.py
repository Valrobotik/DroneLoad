RETRY_PERIOD = 1.0     # s entre deux demandes à l'autopilote (mode, armement, décollage)
TIMEOUT = 60.0         # s pour passer en GUIDED et armer (pré-armements d'ArduPilot, EKF prêt...)
TOLERANCE = 0.05       # m


class Decollage:
    """GUIDED, armement, décollage à la hauteur cible (mesurée au lidar), puis vol stationnaire."""

    def __init__(self, log, altitude):
        self.log = log
        self.altitude = altitude
        self.reset()

    def reset(self):
        # États : GUIDED -> ARMEMENT -> MONTEE -> STATIONNAIRE, ou ECHEC
        self.state = 'GUIDED'
        self.start = None
        self.last_request = None

    def _retry_due(self, now):
        if self.last_request is None or now - self.last_request >= RETRY_PERIOD:
            self.last_request = now
            return True
        return False

    def _set_state(self, state):
        self.state = state
        self.last_request = None
        self.log.info(f'[DÉCOLLAGE] {state}')

    def run(self, drone, now):
        if self.start is None:
            self.start = now
            self.log.info(f'[DÉCOLLAGE] demandé à {self.altitude} m (lidar)')

        if self.state in ('GUIDED', 'ARMEMENT') and now - self.start > TIMEOUT:
            self.log.error('[DÉCOLLAGE] ÉCHEC : GUIDED ou armement refusé (voir les pré-armements)')
            self._set_state('ECHEC')

        if self.state == 'GUIDED':
            if drone.mode == 'GUIDED':
                self._set_state('ARMEMENT')
            elif self._retry_due(now):
                drone.set_mode('GUIDED')

        elif self.state == 'ARMEMENT':
            if drone.armed:
                drone.takeoff(self.altitude)
                self._set_state('MONTEE')
            elif self._retry_due(now):
                drone.arm()

        elif self.state == 'MONTEE':
            if not drone.armed:
                # ArduPilot désarme seul si le décollage n'arrive pas assez vite après l'armement
                self.log.warning('[DÉCOLLAGE] désarmé pendant la montée, nouvel essai')
                self._set_state('GUIDED')
                self.start = now
            elif drone.get_current_alt_brute() >= self.altitude - TOLERANCE:
                self._set_state('STATIONNAIRE')

        elif self.state == 'STATIONNAIRE':
            drone.send_velocity_body(0, 0, 0, 0)
