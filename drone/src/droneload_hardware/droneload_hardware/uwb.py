"""Tag UWB DWM1001-DEV (firmware PANS) en USB : position -> /uwb/pose, transmise à la Pixhawk par position_ext.

Le tag est interrogé par l'API binaire TLV (dwm_loc_get, voir uwb_tlv.py), recommandée par Qorvo pour un
programme : elle ne dépend pas de l'état du tag, qui reste alimenté quand ce nœud redémarre.
- /uwb/pose : PoseWithCovarianceStamped, repère des ancres (frame_id uwb), en m, horodatée à la réception,
  orientation à l'identité (PANS ne donne pas de cap). Positions de qualité < qf_min ou calculées avec moins de ancres_min ancres
  ignorées (avec 3 ancres, deux positions possibles, symétriques par rapport à leur plan), position
  identique à la précédente non republiée (le tag n'a pas fait de nouvelle mesure).
- Un seul lecteur sur le port (deux lecteurs se partagent les octets : panne du concours 2026) : le port
  n'est pas ouvert s'il est déjà ouvert par un autre programme (cherché dans /proc, le programme est nommé
  sur le dashboard), puis il est verrouillé par le noyau (TIOCEXCL) : toute autre ouverture est refusée
  (« Device or resource busy »), sauf par root.
- Tag absent, port occupé ou erreur : le nœud reste lancé, le signale sur /rosout (donc sur le dashboard)
  et rouvre toutes les RETRY_PERIOD s ; tag muet MUTE_TIMEOUT s : port rouvert.
- Tag laissé en mode shell (terminal ouvert à la main) : « quit » le ramène en mode TLV.

En simulation, /uwb/pose vient du service uwb (sim/scripts/uwb_sim.py) : ce nœud n'est pas lancé.
Non testé sur le vrai tag (testé contre un faux tag : sim/scripts/uwb_pilote_test.py).
"""
import fcntl
import os
import termios
import time
from pathlib import Path

import rclpy
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from droneload_hardware.uwb_tlv import QUITTER_SHELL, REQUETE_LOC, est_shell, lire_reponse

RETRY_PERIOD = 5.0     # s entre deux tentatives d'ouverture du port
MUTE_TIMEOUT = 2.0     # s sans réponse du tag avant de rouvrir le port
REPLY_TIMEOUT = 0.2    # s : requête renvoyée si la précédente reste sans réponse
ERROR_THROTTLE = 30.0  # s entre deux rappels de la même erreur
MAX_BUFFER = 4096      # octets gardés au plus en attente de décodage


def autre_lecteur(chemin):
    """Processus (pid, commande) qui a déjà ouvert le port, None sinon. Même principe que probleme_port de
    mavros.launch.py ; seuls les processus du même utilisateur sont visibles (pas ceux lancés avec sudo)."""
    try:
        port = os.path.realpath(chemin)
    except OSError:
        return None
    moi = str(os.getpid())
    for fd in Path('/proc').glob('[0-9]*/fd/*'):
        try:
            if fd.parts[2] != moi and os.path.realpath(fd) == port:
                commande = Path(f'/proc/{fd.parts[2]}/cmdline').read_bytes().replace(b'\0', b' ').decode().strip()
                return fd.parts[2], commande[:80]
        except OSError:
            continue   # processus terminé ou d'un autre utilisateur
    return None


class Uwb(Node):
    def __init__(self):
        super().__init__('uwb')
        self.declare_parameter('port', '/dev/uwb')   # lien créé par udev (install/, rôle pi)
        self.declare_parameter('debit', 115200)
        self.declare_parameter('periode', 0.05)      # s entre deux requêtes (20 Hz : le tag calcule à 10 Hz)
        self.declare_parameter('qf_min', 50)         # qualité minimale de la position (0-100)
        self.declare_parameter('ancres_min', 4)      # ancres utilisées au minimum (3 : deux positions possibles)
        self.declare_parameter('bruit_xy', 0.10)     # m, écart type annoncé dans la covariance
        self.declare_parameter('bruit_z', 0.30)
        self.port = self.get_parameter('port').value
        self.debit = self.get_parameter('debit').value
        self.qf_min = self.get_parameter('qf_min').value
        self.ancres_min = self.get_parameter('ancres_min').value
        bruit_xy = self.get_parameter('bruit_xy').value
        bruit_z = self.get_parameter('bruit_z').value
        self.covariance = [0.0] * 36
        self.covariance[0] = self.covariance[7] = bruit_xy ** 2
        self.covariance[14] = bruit_z ** 2
        self.covariance[21] = self.covariance[28] = self.covariance[35] = 1e6   # orientation inconnue

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/pose', 10)
        self.serie = None
        self.tampon = bytearray()
        self.prochain_essai = 0.0
        self.derniere_reponse = 0.0
        self.requete_envoyee = None    # heure de la requête en attente de réponse
        self.derniere_position = None
        self.erreur = (None, 0.0)      # dernier message d'erreur et son heure (rappels espacés)
        self.create_timer(self.get_parameter('periode').value, self._tick)

    def _signaler(self, message):
        """Erreur sur /rosout, rappelée toutes les ERROR_THROTTLE s si elle se répète."""
        maintenant = time.monotonic()
        if message != self.erreur[0] or maintenant - self.erreur[1] > ERROR_THROTTLE:
            self.get_logger().error(message)
            self.erreur = (message, maintenant)

    def _ouvrir(self):
        self.prochain_essai = time.monotonic() + RETRY_PERIOD
        lecteur = autre_lecteur(self.port)
        if lecteur:
            self._signaler(f'{self.port} déjà ouvert par le processus {lecteur[0]} ({lecteur[1]}) : port non ouvert '
                           f'(deux lecteurs se partagent les octets), nouvel essai dans {RETRY_PERIOD:.0f} s')
            return
        try:
            # timeout 0 : lectures et écritures sans attente ; exclusive : verrou flock (autre nœud uwb)
            self.serie = serial.Serial(self.port, self.debit, timeout=0, write_timeout=0, exclusive=True)
            # Verrou du noyau : toute autre ouverture du port échoue désormais (screen, minicom, script...)
            fcntl.ioctl(self.serie.fileno(), termios.TIOCEXCL)
        except (serial.SerialException, OSError) as e:
            if self.serie is not None:
                self.serie.close()
                self.serie = None
            self._signaler(f'Tag UWB indisponible sur {self.port} ({e}) : nouvel essai dans {RETRY_PERIOD:.0f} s')
            return
        self.tampon.clear()
        self.requete_envoyee = None
        self.derniere_reponse = time.monotonic()
        self.derniere_position = None
        self.erreur = (None, 0.0)
        self.get_logger().info(f'Tag UWB ouvert sur {self.port}')

    def _fermer(self, raison, attente):
        self._signaler(f'Tag UWB : {raison}, port rouvert dans {attente:.0f} s')
        self.fermer_port()
        self.prochain_essai = time.monotonic() + attente

    def fermer_port(self):
        """Lève le verrou du noyau (il resterait si un autre programme gardait le port ouvert) et ferme."""
        if self.serie is None:
            return
        try:
            fcntl.ioctl(self.serie.fileno(), termios.TIOCNXCL)
            self.serie.close()
        except (serial.SerialException, OSError):
            pass
        self.serie = None

    def _tick(self):
        if self.serie is None:
            if time.monotonic() >= self.prochain_essai:
                self._ouvrir()
            return
        try:
            self._echanger()
        except (serial.SerialException, OSError) as e:
            self._fermer(f'erreur du port ({e})', RETRY_PERIOD)

    def _echanger(self):
        maintenant = time.monotonic()
        recu = self.serie.read(MAX_BUFFER)
        if recu:
            self.tampon += recu
            self.derniere_reponse = maintenant

        if est_shell(self.tampon):
            self.get_logger().warn('Tag UWB en mode shell : retour au mode TLV (quit)')
            self.serie.write(QUITTER_SHELL)
            self.tampon.clear()
            self.requete_envoyee = None
            return

        while self.tampon:
            resultat, consommes = lire_reponse(self.tampon)
            if consommes == 0:
                break
            del self.tampon[:consommes]
            if resultat is None:
                continue
            self.requete_envoyee = None
            if isinstance(resultat, int):
                self._signaler(f'Tag UWB : dwm_loc_get refusé (code {resultat})')
            else:
                self._publier(resultat)
        del self.tampon[:-MAX_BUFFER]

        if maintenant - self.derniere_reponse > MUTE_TIMEOUT:
            self._fermer(f'aucune réponse depuis {MUTE_TIMEOUT:.0f} s', 1.0)
            return
        if self.requete_envoyee is None or maintenant - self.requete_envoyee > REPLY_TIMEOUT:
            self.serie.write(REQUETE_LOC)
            self.requete_envoyee = maintenant

    def _publier(self, pos):
        if pos.qf < self.qf_min or pos == self.derniere_position:
            return
        if pos.ancres < self.ancres_min:
            self._signaler(f'Tag UWB : position ignorée, {pos.ancres} ancres (minimum {self.ancres_min})')
            return
        if self.derniere_position is None:
            self.get_logger().info(f'Tag UWB : première position ({pos.ancres} ancres, qualité {pos.qf})')
        self.derniere_position = pos
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uwb'
        msg.pose.pose.position.x = pos.x / 1000.0
        msg.pose.pose.position.y = pos.y / 1000.0
        msg.pose.pose.position.z = pos.z / 1000.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = self.covariance
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Uwb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fermer_port()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
