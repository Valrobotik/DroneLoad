RETRY_PERIOD = 1.0     # s entre deux demandes du mode LAND


class Atterrissage:
    """Mode LAND d'ArduPilot jusqu'au désarmement automatique au sol."""

    def __init__(self, log):
        self.log = log
        self.reset()

    def reset(self):
        # États : LAND -> DESCENTE -> POSE
        self.state = 'LAND'
        self.last_request = None

    def run(self, drone, now):
        if self.state == 'LAND':
            if drone.mode == 'LAND':
                self.state = 'DESCENTE'
                self.log.info('[ATTERRISSAGE] descente')
            elif self.last_request is None or now - self.last_request >= RETRY_PERIOD:
                self.last_request = now
                drone.land()

        elif self.state == 'DESCENTE':
            if not drone.armed:
                self.state = 'POSE'
                self.log.info('[ATTERRISSAGE] posé et désarmé')
