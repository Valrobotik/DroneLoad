"""Changements de repère de la carte : ce que voit une caméra du drone -> repère map (ENU) de MAVROS.

Sans ROS (testé par test/test_reperes.py). Conventions :
- pose du drone : ((x, y, z), (qx, qy, qz, qw)), /mavros/local_position/pose (repère map ENU, drone FLU) ;
- repère optique d'une caméra : x vers la droite de l'image, y vers le bas, z dans l'axe de visée ;
- Camera : position dans le repère du drone (FLU, m) et rotation optique -> drone (matrice 3x3).
  Caméra sol, haut de l'image vers l'avant : x_drone = -y_opt, y_drone = -x_opt, z_drone = -z_opt.
- repère d'un ArUco (cv2.solvePnP, coins de aruco.py) : z sort de la face imprimée, vers la caméra.
"""
import math

CAMERA_SOL_ROTATION = (0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0)   # optique -> drone, par lignes


def rotate(q, v):
    """Applique le quaternion q (x, y, z, w) au vecteur v."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    return (vx + 2 * (w * cx + y * cz - z * cy),
            vy + 2 * (w * cy + z * cx - x * cz),
            vz + 2 * (w * cz + x * cy - y * cx))


def quaternion_cap(cap):
    """Quaternion (x, y, z, w) d'une rotation de cap (rad, autour de z)."""
    return 0.0, 0.0, math.sin(cap / 2), math.cos(cap / 2)


def cap_quaternion(q):
    """Cap (rad) de l'axe x tourné par q, projeté à l'horizontale."""
    x, y, _ = rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(y, x)


class Camera:
    def __init__(self, position, rotation):
        """position : (avant, gauche, haut) en m dans le repère du drone ; rotation : 9 valeurs par lignes,
        optique -> drone."""
        if len(position) != 3 or len(rotation) != 9:
            raise ValueError('position : 3 valeurs, rotation : 9 valeurs')
        self.position = tuple(float(p) for p in position)
        self.rotation = tuple(tuple(float(r) for r in rotation[3 * i:3 * i + 3]) for i in range(3))

    def direction_drone(self, v):
        return tuple(sum(r * c for r, c in zip(ligne, v)) for ligne in self.rotation)

    def point_drone(self, v):
        return tuple(d + p for d, p in zip(self.direction_drone(v), self.position))


def point_map(pose, camera, v):
    """Point v du repère optique de la caméra -> repère map."""
    position, q = pose
    return tuple(p + d for p, d in zip(position, rotate(q, camera.point_drone(v))))


def direction_map(pose, camera, v):
    """Direction v du repère optique (sans translation) -> repère map."""
    return rotate(pose[1], camera.direction_drone(v))


def pixel_sol(pose, camera, k, u, v, hauteur):
    """Point du sol vu au pixel (u, v) : rayon de la caméra coupé par le plan du sol.

    k : matrice de calibration (9 valeurs de camera_info) ; hauteur : distance lidar (m), mesurée dans
    l'axe du drone (corrigée de l'inclinaison). None si le rayon ne descend pas vers le sol."""
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    rayon = direction_map(pose, camera, ((u - cx) / fx, (v - cy) / fy, 1.0))
    if rayon[2] > -1e-3:
        return None
    sol = pose[0][2] - hauteur * rotate(pose[1], (0.0, 0.0, 1.0))[2]
    origine = point_map(pose, camera, (0.0, 0.0, 0.0))
    t = (sol - origine[2]) / rayon[2]
    return tuple(o + t * r for o, r in zip(origine, rayon))


class Fenetre:
    """Fenêtre de taille connue, repérée par ses ArUco (recto et verso).

    Repère de la fenêtre : centre de l'ouverture ; normale n horizontale, vers le côté recto ; u horizontal,
    vers la droite vu du recto (u = -n x z) ; v vertical vers le haut. Mêmes axes vus des deux côtés :
    un ArUco du verso a le même (u, v) qu'on le regarde de devant ou de derrière.
    Cap de la fenêtre = direction de n (rad, sens trigo depuis l'axe x de la carte).
    """

    VERTICALE_MIN = 0.5   # composante horizontale minimale de la normale d'un ArUco de fenêtre (sinon pose ambiguë)

    def __init__(self, arucos):
        """arucos : {identifiant: (face 'recto' | 'verso', u, v)}, en m depuis le centre de l'ouverture."""
        for face, _, _ in arucos.values():
            if face not in ('recto', 'verso'):
                raise ValueError(f'face inconnue : {face}')
        self.arucos = dict(arucos)

    @classmethod
    def depuis_parametres(cls, lignes):
        """lignes : ['4 recto 0.43 -0.68', ...] (paramètre fenetre_arucos de drone.yaml)."""
        arucos = {}
        for ligne in lignes:
            ident, face, u, v = ligne.split()
            arucos[int(ident)] = (face, float(u), float(v))
        return cls(arucos)

    def face(self, ident):
        return self.arucos[ident][0] if ident in self.arucos else None

    def depuis_aruco(self, ident, position, normale):
        """Centre (x, y, z) et cap de la fenêtre à partir d'un de ses ArUco : position de son centre et
        normale de sa face imprimée, dans le repère map. None si l'ArUco n'est pas vertical."""
        face, u, v = self.arucos[ident]
        nx, ny = normale[0], normale[1]
        if face == 'verso':
            nx, ny = -nx, -ny
        norme = math.hypot(nx, ny)
        if norme < self.VERTICALE_MIN:
            return None
        nx, ny = nx / norme, ny / norme
        ux, uy = -ny, nx   # -n x z
        centre = (position[0] - u * ux, position[1] - u * uy, position[2] - v)
        return centre, math.atan2(ny, nx)
