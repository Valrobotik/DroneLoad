WINDOW = 2.0           # s : fenêtre d'observation
RATIO = 0.8            # part des images où seule la classe cible doit être détectée


class Epreuve1:
    """Validation d'une cible : sur une fenêtre de 2 s, seule la classe demandée est détectée sur
    au moins 80 % des images analysées par le nœud vision.

    Fenêtre sans aucune image analysée (nœud vision arrêté, caméra muette) : état SANS_VISION, signalé,
    au lieu de chercher sans fin une cible qu'on ne peut pas voir ; retour en RECHERCHE dès qu'une
    fenêtre contient de nouveau des images.
    """

    def __init__(self, log):
        self.log = log
        self.reset()

    def reset(self):
        # États : RECHERCHE <-> SANS_VISION, RECHERCHE -> TROUVEE
        self.state = 'RECHERCHE'
        self.window_start = None
        self.frames = 0
        self.matches = 0

    def add_frame(self, classes, target_class):
        """À appeler pour chaque message de détections : classes = noms détectés sur l'image."""
        self.frames += 1
        if classes == {target_class}:
            self.matches += 1

    def run(self, now):
        """Vrai au moment où la cible est validée (une seule fois jusqu'au prochain reset)."""
        if self.state == 'TROUVEE':
            return False
        if self.window_start is None:
            self.window_start = now
        if now - self.window_start < WINDOW:
            return False
        frames, matches = self.frames, self.matches
        self.window_start, self.frames, self.matches = now, 0, 0

        if frames == 0:
            if self.state != 'SANS_VISION':
                self.state = 'SANS_VISION'
                self.log.error(f'[ÉPREUVE 1] aucune image analysée depuis {WINDOW:.0f} s : vision arrêtée ?')
            return False
        if self.state == 'SANS_VISION':
            self.state = 'RECHERCHE'
            self.log.info('[ÉPREUVE 1] images de nouveau analysées, recherche')

        if matches >= frames * RATIO:
            self.state = 'TROUVEE'
            return True
        return False
