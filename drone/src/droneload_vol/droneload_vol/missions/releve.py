from .deplacement import Deplacement


class Releve:
    """Relevé du terrain : enregistre sur la carte tout ce que voit la caméra sol (carte.enregistrer_vus :
    ArUco, fenêtre, robot, images YOLO), en stationnaire, ou en allant à un point envoyé (set_point)
    pendant le relevé. Sert aux tests de la carte et de modèle aux missions qui enregistrent des points.

    besoin_position : sans position fiable, rien n'est enregistré (la carte serait fausse) et main_node
    met la mission en pause. Le point est oublié à chaque entrée dans le mode : un ancien point visé
    (mission deplacement) n'est jamais rejoint par surprise. Ne bloque jamais.
    """
    besoin_position = True

    def __init__(self, log, carte, vitesse_max=0.5, precision=0.1):
        self.carte = carte
        self.deplacement = Deplacement(log, vitesse_max, precision)
        self.reset()

    def reset(self):
        # États : STATIONNAIRE, ou ceux de Deplacement (EN_ROUTE, ARRIVE, ECHEC) une fois un point envoyé
        self.deplacement.point = None
        self.deplacement.reset()
        self.state = 'STATIONNAIRE'

    def set_point(self, x, y):
        self.deplacement.set_point(x, y)
        self.deplacement.reset()

    def run(self, drone, now):
        self.carte.enregistrer_vus()
        if self.deplacement.point is None:
            drone.hover()
            self.state = 'STATIONNAIRE'
            return
        self.deplacement.run(drone, now)
        self.state = self.deplacement.state
