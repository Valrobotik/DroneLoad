"""Missions du drone : machines à états avancées d'un pas à chaque tick du nœud principal.

Chaque mission a un état (self.state), une méthode reset() appelée à chaque entrée dans le mode
correspondant, et une méthode run(drone, now) qui ne bloque jamais. Elle s'enregistre en une ligne dans
la table self.missions de main_node (mode -> mission).

Mission qui a besoin de la position (aller à un point, à la fenêtre, revenir) : besoin_position = True.
main_node ne l'appelle alors que si drone.get_position() est fiable (jamais None dans run) ; sinon le drone
reste en stationnaire (POSITION_PERDUE), la mission reprend si la position revient avant delai_position,
et est abandonnée au-delà (ABANDON). Pour que la reprise marche : viser des points absolus du repère
de l'EKF (calculés une fois, gardés dans la mission), pas des déplacements comptés en temps ; et
prévoir que les délais propres à la mission continuent de courir pendant une pause.
"""
from .atterrissage import Atterrissage
from .decollage import Decollage
from .deplacement import Deplacement
from .epreuve1 import Epreuve1
from .epreuve2 import run_epreuve2
from .releve import Releve

__all__ = ['Atterrissage', 'Decollage', 'Deplacement', 'Epreuve1', 'Releve', 'run_epreuve2']
