#!/usr/bin/env python3
"""Lance les tests du drone (PC : macOS, Linux, Windows ; Python standard seulement).

    python3 tests/run.py --liste                  tests, suites et ce qu'il leur faut
    python3 tests/run.py smoke mission            un ou plusieurs tests (ou suites)
    python3 tests/run.py tout                     tous les tests

Pour chaque test de simulation, le lanceur prépare la simu Docker comme il faut : il la démarre si elle
ne tourne pas, avec ou sans le code du drone (service drone, arrêté pour smoke), remet le drone au départ
(sim/scripts/reset_drone.sh) puis lance le test dans le bon conteneur. Les tests sont regroupés pour
changer de config le moins possible. Les lignes [OK] / [ÉCHEC] s'affichent (-v : toute la sortie) ;
la sortie complète va dans tests/resultats/<date>/<test>.log (ignoré par git).

À la fin, la simu est arrêtée si le lanceur l'a démarrée (--garder pour la laisser tourner), laissée
sinon. Code de sortie 0 si tous les tests sont OK.
Windows : Git Bash est nécessaire pour reset_drone.sh. Doc : docs/tests.md.
"""
import argparse
import platform
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
REPO = TESTS_DIR.parent
SIM = REPO / 'sim'
RESULTATS = TESTS_DIR / 'resultats'
DRONE_PRET = 'Nœud principal prêt'   # log de main_node une fois le code du drone construit et lancé
DRONE_DELAI = 300                    # s pour construire (colcon) et lancer le code du drone


@dataclass
class Test:
    description: str
    commande: str = ''          # dans le conteneur, depuis /workspace (racine du repo) ; vide : test sur le PC
    conteneur: str = 'companion'
    drone: bool = True          # service drone (code du drone) lancé ; False : arrêté pendant le test
    reset: bool = True          # drone remis au départ avant le test


TESTS = {
    'smoke': Test('autopilote seul : MAVROS, capteurs, décollage, stationnaire, atterrissage',
                  'python3 tests/simu/smoke.py', drone=False),
    'mission': Test('code du drone : 2 cycles décollage / attente / atterrissage par MQTT, ArUco 0',
                    'python3 tests/simu/mission.py'),
}
SUITES = {
    'base': ['smoke', 'mission'],
}
SUITES['tout'] = SUITES['base']


# --- Docker compose -----------------------------------------------------------------------------

class Simu:
    def __init__(self, verbeux):
        self.verbeux = verbeux
        self.fichiers = ['-f', 'compose.yml', '-f', 'compose.linux.yml'] if platform.system() == 'Linux' else []
        self.demarree = False    # démarrée par le lanceur

    def compose(self, *args, capture=True):
        return subprocess.run(['docker', 'compose', *self.fichiers, *args], cwd=SIM,
                              capture_output=capture, text=True)

    def services(self):
        r = self.compose('--profile', '*', 'ps', '--services', '--status', 'running')
        return set(r.stdout.split()) if r.returncode == 0 else set()

    def preparer(self, test):
        """Simu dans la config du test (démarrée, service drone)."""
        services = self.services()
        if 'gz-server' not in services:
            print(f'  simu démarrée{" avec le code du drone" if test.drone else ""}', flush=True)
            self.compose(*(['--profile', 'drone'] if test.drone else []), 'up', '-d')
            self.demarree = True
            if test.drone:
                self.attendre_drone()
            return
        if test.drone and 'drone' not in services:
            print('  code du drone lancé (service drone)', flush=True)
            self.compose('--profile', 'drone', 'up', '-d')
            self.attendre_drone()
        elif not test.drone and 'drone' in services:
            print('  code du drone arrêté le temps du test (service drone)', flush=True)
            self.compose('stop', 'drone')

    def attendre_drone(self):
        debut = time.monotonic()
        while time.monotonic() - debut < DRONE_DELAI:
            if DRONE_PRET in self.compose('logs', '--since', f'{int(time.monotonic() - debut) + 5}s', 'drone').stdout:
                return True
            time.sleep(3)
        print(f'  ATTENTION : code du drone pas prêt après {DRONE_DELAI} s (docker compose logs drone)', flush=True)
        return False

    def reset(self, attente):
        script = SIM / 'scripts' / 'reset_drone.sh'
        commande = [str(script)]
        if platform.system() == 'Windows':
            bash = shutil.which('bash')   # Git Bash dans le PATH
            if bash is None:
                print('  Git Bash introuvable : drone pas remis au départ', flush=True)
                return
            commande = [bash, script.as_posix()]
        subprocess.run(commande, cwd=SIM, capture_output=True, text=True)
        print(f'  drone remis au départ, {attente:.0f} s pour que le SITL soit prêt à armer', flush=True)
        time.sleep(attente)

    def executer(self, test, journal):
        """Lance le test dans son conteneur ; affiche les lignes de résultat, journalise tout. Code de sortie."""
        cmd = ['docker', 'compose', *self.fichiers, 'exec', '-T', test.conteneur, 'bash', '-lc', test.commande]
        with open(journal, 'w') as f, subprocess.Popen(cmd, cwd=SIM, stdout=subprocess.PIPE,
                                                       stderr=subprocess.STDOUT, text=True) as p:
            for ligne in p.stdout:
                f.write(ligne)
                if self.verbeux or ligne.startswith(('[OK]', '[ÉCHEC]')) or 'OBSERVATION' in ligne or 'Traceback' in ligne \
                        or ' : OK' in ligne or ' : ÉCHEC' in ligne:
                    print('    ' + ligne.rstrip(), flush=True)
        return p.returncode

    def arreter(self):
        print('Simu arrêtée (docker compose down)', flush=True)
        self.compose('--profile', '*', 'down', '--remove-orphans')


# --- Tests --------------------------------------------------------------------------------------

def unitaires(journal):
    """pytest s'il est installé, sinon chaque fichier lancé directement (ils se suffisent à eux-mêmes)."""
    with open(journal, 'w') as f:
        if subprocess.run([sys.executable, '-c', 'import pytest'], capture_output=True).returncode == 0:
            r = subprocess.run([sys.executable, '-m', 'pytest', '-q', str(TESTS_DIR / 'unitaires')],
                               cwd=REPO, capture_output=True, text=True)
            f.write(r.stdout + r.stderr)
            print('    ' + (r.stdout.strip().splitlines() or ['?'])[-1], flush=True)
            return r.returncode
        code = 0
        for fichier in sorted((TESTS_DIR / 'unitaires').glob('test_*.py')):
            r = subprocess.run([sys.executable, str(fichier)], cwd=REPO, capture_output=True, text=True)
            f.write(r.stdout + r.stderr)
            print(f"    {fichier.name} : {'OK' if r.returncode == 0 else 'ÉCHEC'} (sans pytest)", flush=True)
            code = code or r.returncode
        return code


def liste():
    print('Tests :')
    for nom, t in TESTS.items():
        besoin = ('PC' if not t.commande else
                  f"{t.conteneur}, {'code du drone' if t.drone else 'sans le code du drone'}")
        print(f'  {nom:<16} {t.description}\n  {"":<16} ({besoin})')
    print('Suites :')
    for nom, tests in SUITES.items():
        print(f"  {nom:<16} {' '.join(tests)}")


def ordre(noms):
    """Tests dans l'ordre demandé, regroupés pour changer de config le moins possible : PC, puis simu
    sans le code du drone (smoke), puis avec."""
    def cle(nom):
        t = TESTS[nom]
        if not t.commande:
            return 0
        return 2 if t.drone else 1
    return sorted(dict.fromkeys(noms), key=cle)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument('tests', nargs='*', help='tests ou suites (--liste)')
    parser.add_argument('--liste', action='store_true', help='tests et suites disponibles')
    parser.add_argument('--garder', action='store_true', help='laisser tourner la simu démarrée par le lanceur')
    parser.add_argument('--attente', type=float, default=40.0, help='s après la remise au départ (SITL prêt à armer)')
    parser.add_argument('-v', '--verbeux', action='store_true', help='afficher toute la sortie des tests')
    args = parser.parse_args()
    if args.liste or not args.tests:
        liste()
        return 0
    noms = []
    for nom in args.tests:
        if nom in SUITES:
            noms += SUITES[nom]
        elif nom in TESTS:
            noms.append(nom)
        else:
            parser.error(f'test ou suite inconnu : {nom} (--liste)')

    dossier = RESULTATS / datetime.now().strftime('%Y%m%d-%H%M%S')
    dossier.mkdir(parents=True, exist_ok=True)
    simu = Simu(args.verbeux)
    bilan = []
    try:
        for nom in ordre(noms):
            test = TESTS[nom]
            print(f'== {nom} : {test.description}', flush=True)
            journal = dossier / f'{nom}.log'
            debut = time.monotonic()
            if not test.commande:
                code = unitaires(journal)
            else:
                simu.preparer(test)
                if test.reset:
                    simu.reset(args.attente)
                code = simu.executer(test, journal)
            bilan.append((nom, code == 0, time.monotonic() - debut))
    except KeyboardInterrupt:
        print('\nInterrompu', flush=True)
    finally:
        if simu.demarree and not args.garder:
            simu.arreter()
        elif simu.demarree:
            print('Simu laissée en marche (--garder) : cd sim && docker compose --profile \'*\' down', flush=True)

    print('\nBilan :')
    for nom, ok, duree in bilan:
        print(f"  {'OK   ' if ok else 'ÉCHEC'}  {nom:<16} {duree / 60:4.1f} min")
    print(f'Journaux : {dossier}')
    return 0 if bilan and all(ok for _, ok, _ in bilan) and len(bilan) == len(set(noms)) else 1


if __name__ == '__main__':
    sys.exit(main())
