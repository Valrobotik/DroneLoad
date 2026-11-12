RETRY_PERIOD = 1.0     # s entre deux demandes à l'autopilote (mode, armement, décollage)
TIMEOUT = 60.0         # s pour passer en GUIDED et armer (pré-armements d'ArduPilot, EKF prêt...)
CLIMB_TIMEOUT = 30.0   # s pour atteindre la hauteur cible une fois armé
TOLERANCE = 0.05       # m
LIFTOFF_HEIGHT = 0.3   # m : en dessous, le drone n'a pas encore quitté le sol
TAKEOFF_RETRY = 5.0    # s sans quitter le sol avant de renvoyer l'ordre (montée normale : ~2,5 s en SITL ; avant DISARM_DELAY, 10 s)


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
        self.climb_start = None
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
                # Ordre parti seulement si MAVROS est prêt, sinon nouvel essai au tick suivant
                if drone.takeoff(self.altitude):
                    self._set_state('MONTEE')
                    self.climb_start = self.last_request = now
            elif self._retry_due(now):
                drone.arm()

        elif self.state == 'MONTEE':
            if not drone.armed:
                # ArduPilot désarme seul si le décollage n'arrive pas assez vite après l'armement
                self.log.warning('[DÉCOLLAGE] désarmé pendant la montée, nouvel essai')
                self._set_state('GUIDED')
                self.start = now
            elif now - self.climb_start > CLIMB_TIMEOUT:
                # Le drone reste en stationnaire (tenu par l'autopilote) : l'opérateur décide (land)
                self.log.error(f'[DÉCOLLAGE] ÉCHEC : {self.altitude} m non atteints en {CLIMB_TIMEOUT:.0f} s '
                               f'(hauteur lidar : {drone.get_current_alt_brute()})')
                self._set_state('ECHEC')
            else:
                height = drone.get_current_alt_brute()   # None : hauteur inconnue, on attend
                if height is not None and height >= self.altitude - TOLERANCE:
                    self._set_state('STATIONNAIRE')
                elif height is not None and height < LIFTOFF_HEIGHT and now - self.last_request >= TAKEOFF_RETRY:
                    # Ordre perdu ou refusé : sans renvoi, ArduPilot désarmerait seul au sol (DISARM_DELAY).
                    # Un renvoi superflu (drone déjà en montée) est simplement refusé par l'autopilote.
                    self.log.warning(f'[DÉCOLLAGE] toujours au sol ({height:.2f} m), ordre renvoyé')
                    self.last_request = now
                    drone.takeoff(self.altitude)

        elif self.state == 'STATIONNAIRE':
            drone.send_velocity_body(0, 0, 0, 0)
