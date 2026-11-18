RETRY_PERIOD = 1.0     # s entre deux demandes à l'autopilote (armement, décollage)
TIMEOUT = 60.0         # s pour armer une fois en GUIDED (pré-armements d'ArduPilot, EKF prêt, kill relâché...)
CLIMB_TIMEOUT = 30.0   # s pour atteindre la hauteur cible une fois armé
TOLERANCE = 0.05       # m
LIFTOFF_HEIGHT = 0.3   # m : en dessous, le drone n'a pas encore quitté le sol
TAKEOFF_RETRY = 5.0    # s sans quitter le sol avant de renvoyer l'ordre (montée normale : ~2,5 s en SITL ; avant DISARM_DELAY, 10 s)


class Decollage:
    """Attente de GUIDED (radio), armement, décollage à la hauteur cible (mesurée au lidar), stationnaire.

    Seule la radio passe en GUIDED (FLTMODE_GCSBLOCK de pixhawk.param refuse GUIDED par MAVLink) : la
    mission attend le pilote sans limite de temps. Le kill motor de la radio (RC8) relâché vaut autorisation
    d'armer : tant qu'il est actif, ArduPilot refuse l'armement. Autopilote sorti de GUIDED avant d'être
    armé : retour en attente, la mission n'arme jamais hors GUIDED.
    """

    def __init__(self, log, altitude):
        self.log = log
        self.altitude = altitude
        self.reset()

    def reset(self):
        # États : ATTENTE_GUIDED -> ARMEMENT -> MONTEE -> STATIONNAIRE, ou ECHEC
        self.state = None
        self.arm_start = None
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

    def _attendre_guided(self, raison):
        self._set_state('ATTENTE_GUIDED')
        self.log.warning(f'[DÉCOLLAGE] {raison} : passer en GUIDED à la radio (et relâcher le kill)')

    def run(self, drone, now):
        if self.state is None:
            self.log.info(f'[DÉCOLLAGE] demandé à {self.altitude} m (lidar)')
            self._attendre_guided('en attente du pilote')

        if self.state == 'ATTENTE_GUIDED':
            if drone.mode == 'GUIDED':
                self._set_state('ARMEMENT')
                self.arm_start = now

        elif self.state == 'ARMEMENT':
            if drone.mode != 'GUIDED':
                # Pilote sorti de GUIDED avant l'armement : ne jamais armer dans son mode
                self._attendre_guided(f'autopilote en {drone.mode or "?"}, armement annulé')
            elif drone.armed:
                # Ordre parti seulement si MAVROS est prêt, sinon nouvel essai au tick suivant
                if drone.takeoff(self.altitude):
                    self._set_state('MONTEE')
                    self.climb_start = self.last_request = now
            elif now - self.arm_start > TIMEOUT:
                self.log.error('[DÉCOLLAGE] ÉCHEC : armement refusé (kill actif ? voir les pré-armements)')
                self._set_state('ECHEC')
            elif self._retry_due(now):
                drone.arm()

        elif self.state == 'MONTEE':
            if not drone.armed:
                # ArduPilot désarme seul si le décollage n'arrive pas assez vite après l'armement
                self.log.warning('[DÉCOLLAGE] désarmé pendant la montée, nouvel essai')
                self._attendre_guided('désarmé')
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
