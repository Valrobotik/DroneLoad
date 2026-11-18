RETRY_PERIOD = 1.0     # s entre deux demandes du mode LAND


class Atterrissage:
    """Mode LAND d'ArduPilot jusqu'au désarmement automatique au sol.

    La Pi ne passe en LAND que depuis GUIDED (MavrosDrone.land()) : dans un autre mode, le pilote a la
    main à la radio, la mission s'arrête en REFUSE sans rien envoyer (jusqu'à une nouvelle demande de mode).
    Drone déjà désarmé : POSE, sans commande.
    """

    def __init__(self, log):
        self.log = log
        self.reset()

    def reset(self):
        # États : LAND -> DESCENTE -> POSE, ou REFUSE
        self.state = 'LAND'
        self.last_request = None

    def run(self, drone, now):
        if self.state == 'LAND':
            if not drone.armed:
                self.state = 'POSE'
                self.log.info('[ATTERRISSAGE] drone déjà posé et désarmé')
            elif drone.mode == 'LAND':
                self.state = 'DESCENTE'
                self.log.info('[ATTERRISSAGE] descente')
            elif drone.mode != 'GUIDED':
                self.state = 'REFUSE'
                self.log.warning(f'[ATTERRISSAGE] REFUSÉ : autopilote en {drone.mode or "?"}, pas en GUIDED '
                                 '(pilote aux commandes, à lui d\'atterrir)')
            elif self.last_request is None or now - self.last_request >= RETRY_PERIOD:
                self.last_request = now
                drone.land()

        elif self.state == 'DESCENTE':
            if not drone.armed:
                self.state = 'POSE'
                self.log.info('[ATTERRISSAGE] posé et désarmé')
