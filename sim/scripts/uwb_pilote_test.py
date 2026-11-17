"""Test du pilote du tag UWB (nœud uwb, droneload_hardware) contre un faux tag, sans matériel.

Le faux tag tourne dans ce script sur un pseudo-terminal et répond à dwm_loc_get comme un DWM1001
sous PANS (encodage de uwb_tlv.py). Le nœud uwb est lancé par ros2 run, sur un domaine ROS à part
(ROS_DOMAIN_ID 42 : rien d'autre ne publie /uwb/pose). Cas vérifiés :
1. port absent au lancement : le nœud reste lancé, puis ouvre le port dès qu'il existe ;
2. port déjà ouvert par un autre programme : le nœud ne l'ouvre pas et nomme le programme, puis l'ouvre
   une fois le port libéré ;
3. tag laissé en mode shell : le nœud le ramène en mode TLV (quit) ;
4. port verrouillé par le noyau tant que le nœud le lit : une autre ouverture est refusée ;
5. positions converties de mm en m, à la cadence du tag (10 Hz) ;
6. positions de mauvaise qualité (qf < qf_min) ou calculées avec 3 ancres ignorées ;
7. octets parasites entre les réponses : le nœud se resynchronise ;
8. tag muet 3 s : le nœud rouvre le port et les positions reviennent.

Ne vérifie pas que le format TLV est le bon : à confirmer sur le vrai tag (docs/architecture.md, « UWB »).
Prérequis : paquets du drone construits (service drone, ou colcon build dans companion).
Usage : docker compose exec companion bash -lc '. drone/install/setup.bash && python3 sim/scripts/uwb_pilote_test.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import errno
import os
import random
import subprocess
import sys
import threading
import time
import tty
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '42'   # avant rclpy : isolé de la simu

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'drone' / 'src' / 'droneload_hardware'))
from droneload_hardware.uwb_tlv import QUITTER_SHELL, REQUETE_LOC, encoder_loc   # noqa: E402

PORT = Path('/tmp/uwb_pilote_test')   # lien vers le pseudo-terminal, créé pendant le test
Y_MM, Z_MM = -2500, 1200


class FauxTag(threading.Thread):
    """Répond aux requêtes dwm_loc_get ; position calculée à 10 Hz (x augmente de 1 mm à chaque calcul)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.maitre, esclave = os.openpty()
        tty.setraw(esclave)   # pas d'écho ni de traitement des octets par le terminal
        self.chemin = os.ttyname(esclave)
        os.close(esclave)     # seul le nœud doit avoir le port ouvert (il vérifie qu'aucun autre ne l'a)
        self.shell = True          # démarre comme un tag laissé en mode shell
        self.qf = 90
        self.ancres = 4
        self.parasites = False
        self.muet = False
        self.requetes = 0
        self.debut = time.monotonic()

    def run(self):
        tampon = b''
        while True:
            try:
                tampon += os.read(self.maitre, 256)
            except OSError:
                time.sleep(0.05)
                continue
            if self.shell:
                if QUITTER_SHELL in tampon:
                    self.shell = False
                    tampon = b''
                else:
                    os.write(self.maitre, b'\r\nPOS,1.00,2.00,0.50,80\r\ndwm> ')
                continue
            while REQUETE_LOC in tampon:
                tampon = tampon.split(REQUETE_LOC, 1)[1]
                self.requetes += 1
                if self.muet:
                    continue
                x = 1000 + int((time.monotonic() - self.debut) * 10)
                reponse = encoder_loc(x, Y_MM, Z_MM, self.qf, self.ancres)
                if self.parasites:
                    reponse = bytes(random.randrange(256) for _ in range(random.randint(1, 8))) + reponse
                os.write(self.maitre, reponse)


class Ecoute(Node):
    def __init__(self):
        super().__init__('uwb_pilote_test')
        self.messages = []
        self.create_subscription(PoseWithCovarianceStamped, '/uwb/pose', self.messages.append, 10)

    def attendre(self, duree):
        fin = time.monotonic() + duree
        while time.monotonic() < fin:
            rclpy.spin_once(self, timeout_sec=0.05)

    def compter(self, duree):
        self.attendre(0.3)   # réponses déjà en route
        self.messages.clear()
        self.attendre(duree)
        return list(self.messages)


def main():
    PORT.unlink(missing_ok=True)
    journal = open('/tmp/uwb_pilote_test.log', 'w')
    noeud = subprocess.Popen(['ros2', 'run', 'droneload_hardware', 'uwb', '--ros-args', '-p', f'port:={PORT}'],
                             stdout=journal, stderr=subprocess.STDOUT)
    rclpy.init()
    ecoute = Ecoute()
    tag = FauxTag()
    resultats = []

    def etape(nom, ok, detail=''):
        print(f"[{'OK' if ok else 'ÉCHEC'}] {nom}" + (f' ({detail})' if detail else ''), flush=True)
        resultats.append(ok)

    try:
        ecoute.attendre(3.0)
        etape('port absent : nœud toujours lancé, rien publié', noeud.poll() is None and not ecoute.messages)

        tag.start()
        autre = os.open(tag.chemin, os.O_RDWR | os.O_NOCTTY)   # comme un screen laissé ouvert
        PORT.symlink_to(tag.chemin)
        ecoute.attendre(7.0)   # nouvel essai d'ouverture toutes les 5 s
        log = Path('/tmp/uwb_pilote_test.log').read_text()
        etape('port déjà ouvert ailleurs : non ouvert, programme nommé',
              f'déjà ouvert par le processus {os.getpid()}' in log and tag.shell and not ecoute.messages)
        os.close(autre)
        ecoute.attendre(7.0)
        etape('tag en mode shell ramené en mode TLV (quit)', not tag.shell)

        recus = ecoute.compter(3.0)
        cadence = len(recus) / 3.0
        etape('positions reçues à la cadence du tag', 8 <= cadence <= 12, f'{cadence:.1f}/s')
        if recus:
            p = recus[-1].pose.pose.position
            etape('mm convertis en m', abs(p.y - Y_MM / 1000) < 1e-6 and abs(p.z - Z_MM / 1000) < 1e-6
                  and abs(p.x - 1.0) < 1.0 and recus[-1].header.frame_id == 'uwb',
                  f'x {p.x:.3f} y {p.y:.3f} z {p.z:.3f}')

        try:
            os.close(os.open(tag.chemin, os.O_RDWR | os.O_NOCTTY))
            refus = 'ouverture acceptée'
        except OSError as e:
            refus = e.errno == errno.EBUSY or f'{e}'
        recus = ecoute.compter(1.0)
        etape('autre ouverture refusée par le noyau pendant la lecture', refus is True and len(recus) >= 8,
              'EBUSY' if refus is True else refus)

        tag.qf = 30
        recus = ecoute.compter(1.5)
        etape('positions de qualité 30 ignorées', not recus, f'{len(recus)} publiées')
        tag.qf = 90

        tag.ancres = 3
        recus = ecoute.compter(1.5)
        etape('positions calculées avec 3 ancres ignorées', not recus, f'{len(recus)} publiées')
        tag.ancres = 4

        tag.parasites = True
        recus = ecoute.compter(3.0)
        etape('resynchronisation sur octets parasites', len(recus) / 3.0 >= 6
              and all(abs(m.pose.pose.position.y - Y_MM / 1000) < 1e-6 for m in recus), f'{len(recus) / 3.0:.1f}/s')
        tag.parasites = False

        tag.muet = True
        ecoute.attendre(3.0)
        tag.muet = False
        ecoute.attendre(2.0)   # port rouvert 1 s après la détection
        recus = ecoute.compter(2.0)
        etape('positions revenues après un tag muet 3 s', len(recus) / 2.0 >= 8, f'{len(recus) / 2.0:.1f}/s')

        etape('nœud toujours lancé', noeud.poll() is None)
    finally:
        noeud.terminate()
        noeud.wait(timeout=10)
        journal.close()
        PORT.unlink(missing_ok=True)
        ecoute.destroy_node()
        rclpy.try_shutdown()

    log = Path('/tmp/uwb_pilote_test.log').read_text()
    print('--- journal du nœud uwb ---\n' + log.strip())
    print('TEST PILOTE UWB :', 'OK' if all(resultats) else 'ÉCHEC')
    return 0 if all(resultats) else 1


if __name__ == '__main__':
    sys.exit(main())
