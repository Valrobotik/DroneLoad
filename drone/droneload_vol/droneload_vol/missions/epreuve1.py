WINDOW = 2.0           # s : fenêtre d'observation
RATIO = 0.8            # part des images où seule la classe cible doit être détectée


class Epreuve1:
    """Validation d'une cible : sur une fenêtre de 2 s, seule la classe demandée est détectée sur
    au moins 80 % des images analysées par le nœud vision."""

    def __init__(self, log):
        self.log = log
        self.reset()

    def reset(self):
        # États : RECHERCHE -> TROUVEE
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
        if self.state != 'RECHERCHE':
            return False
        if self.window_start is None:
            self.window_start = now
        if now - self.window_start < WINDOW:
            return False
        found = self.frames > 0 and self.matches >= self.frames * RATIO
        self.window_start, self.frames, self.matches = now, 0, 0
        if found:
            self.state = 'TROUVEE'
        return found
