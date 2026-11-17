#!/usr/bin/env python3
"""Lance les tests du drone (PC : macOS, Linux, Windows ; Python standard seulement).

    python3 tests/run.py --liste                  tests, suites et ce qu'il leur faut
    python3 tests/run.py mission uwb              un ou plusieurs tests (ou suites)
    python3 tests/run.py tout                     tous les tests

Pour chaque test de simulation, le lanceur prépare la simu Docker comme il faut : il la démarre si elle
ne tourne pas, avec ou sans l'UWB simulé (PARAM_UWB), avec ou sans le code du drone (service drone,
arrêté pour smoke), remet le drone au départ
(sim/scripts/reset_drone.sh) puis lance le test dans le bon conteneur. Les tests sont regroupés pour
changer de config le moins possible. Changer la config UWB demande de redémarrer toute la simu (un
SITL recréé seul laisse MAVROS et le code du drone sur les paramètres de l'ancien) : le lanceur le fait
pour une simu qu'il a démarrée ; une simu lancée à la main dans l'autre config n'est jamais touchée, le
test est alors sauté (ÉCHEC) avec la commande pour la relancer. Les lignes [OK] / [ÉCHEC] s'affichent (-v : toute la sortie) ;
la sortie complète va dans tests/resultats/<date>/<test>.log (ignoré par git).

À la fin, la simu est arrêtée si le lanceur l'a démarrée (--garder pour la laisser tourner), laissée
sinon (dans la config du dernier test). Code de sortie 0 si tous les tests sont OK.
Windows : Git Bash est nécessaire pour reset_drone.sh. Doc : docs/tests.md.
"""
import argparse
import os
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
    uwb: object = None          # True : PARAM_UWB=uwb ; False : sans ; None : indifférent
    drone: bool = True          # service drone (code du drone) lancé ; False : arrêté pendant le test
    reset: bool = True          # drone remis au départ avant le test


TESTS = {
    'smoke': Test('autopilote seul : MAVROS, capteurs, décollage, stationnaire, atterrissage',
                  'python3 tests/simu/smoke.py', drone=False),
    'mission': Test('code du drone : 2 cycles décollage / attente / atterrissage par MQTT, ArUco 0',
                    'python3 tests/simu/mission.py'),
    'position': Test('perte de position en mission : pause, recalage, abandon, reprise en main par le pilote',
                     'python3 tests/simu/position.py', uwb=True),
    'uwb': Test('position UWB dans l\'EKF : stationnaire, sauts, carrés, pannes, jeu de sources 2',
                'python3 tests/simu/uwb.py', uwb=True),
    'uwb_pilote': Test('pilote du tag UWB contre un faux tag (sans Gazebo)',
                       '. drone/install/setup.bash && python3 tests/simu/uwb_pilote.py', reset=False),
}
SUITES = {
    'base': ['smoke', 'mission'],
    'avec_uwb': ['uwb', 'position', 'uwb_pilote'],
}
SUITES['tout'] = SUITES['base'] + SUITES['avec_uwb']


# --- Docker compose -----------------------------------------------------------------------------

class ConfigIncompatible(Exception):
    """Simu lancée à la main dans une autre config UWB que celle du test (jamais modifiée par le lanceur)."""


class Simu:
    def __init__(self, verbeux):
        self.verbeux = verbeux
        self.fichiers = ['-f', 'compose.yml', '-f', 'compose.linux.yml'] if platform.system() == 'Linux' else []
        self.uwb = None          # config UWB de la simu (True / False), None tant qu'inconnue
        self.demarree = False    # démarrée par le lanceur

    def compose(self, *args, capture=True, uwb=None):
        env = dict(os.environ)
        uwb = self.uwb if uwb is None else uwb
        env['PARAM_UWB'] = 'uwb' if uwb else ''
        r = subprocess.run(['docker', 'compose', *self.fichiers, *args], cwd=SIM, env=env,
                           capture_output=capture, text=True)
        return r

    def services(self):
        r = self.compose('--profile', '*', 'ps', '--services', '--status', 'running')
        return set(r.stdout.split()) if r.returncode == 0 else set()

    def uwb_actuel(self):
        """Config UWB du SITL en cours (uwb.param dans sa commande), None s'il ne tourne pas."""
        ident = self.compose('ps', '-q', 'sitl').stdout.strip()
        if not ident:
            return None
        r = subprocess.run(['docker', 'inspect', '-f', '{{json .Config}}', ident], capture_output=True, text=True)
        return 'uwb.param' in r.stdout

    def preparer(self, test):
        """Simu dans la config du test (démarrée, UWB, service drone). Exception ConfigIncompatible si une simu
        lancée à la main n'a pas la config UWB du test."""
        services = self.services()
        if 'gz-server' in services:
            self.uwb = self.uwb_actuel()
            if test.uwb is not None and test.uwb != self.uwb:
                if not self.demarree:
                    raise ConfigIncompatible(
                        f'simu lancée {"avec" if self.uwb else "sans"} UWB, test {"avec" if test.uwb else "sans"} : '
                        f'l\'arrêter (cd sim && docker compose --profile \'*\' down) pour que le lanceur la démarre, '
                        f'ou la relancer {"avec" if test.uwb else "sans"} UWB (PARAM_UWB={"uwb" if test.uwb else ""} '
                        'docker compose --profile drone up -d, après le down)')
                print(f'  config UWB différente : simu redémarrée {"avec" if test.uwb else "sans"} UWB', flush=True)
                self.compose('--profile', '*', 'down', '--remove-orphans')
                services = set()
        if 'gz-server' not in services:
            self.uwb = bool(test.uwb)
            print(f'  simu démarrée ({"avec" if self.uwb else "sans"} UWB'
                  f'{", code du drone" if test.drone else ""})', flush=True)
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
        env = dict(os.environ, PARAM_UWB='uwb' if self.uwb else '')
        subprocess.run(commande, cwd=SIM, env=env, capture_output=True, text=True)
        print(f'  drone remis au départ, {attente:.0f} s pour que le SITL soit prêt à armer', flush=True)
        time.sleep(attente)

    def executer(self, test, journal):
        """Lance le test dans son conteneur ; affiche les lignes de résultat, journalise tout. Code de sortie."""
        # DRONELOAD_UWB : config UWB attendue, vérifiée par les tests qui s'adaptent à la simu (commun.verifier_uwb)
        attendu = [] if test.uwb is None else ['-e', f'DRONELOAD_UWB={int(test.uwb)}']
        cmd = ['docker', 'compose', *self.fichiers, 'exec', '-T', *attendu, test.conteneur, 'bash', '-lc', test.commande]
        env = dict(os.environ, PARAM_UWB='uwb' if self.uwb else '')
        with open(journal, 'w') as f, subprocess.Popen(cmd, cwd=SIM, env=env, stdout=subprocess.PIPE,
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
                  f"{t.conteneur}, {'avec UWB' if t.uwb else 'sans UWB' if t.uwb is False else 'UWB indifférent'}, "
                  f"{'code du drone' if t.drone else 'sans le code du drone'}")
        print(f'  {nom:<16} {t.description}\n  {"":<16} ({besoin})')
    print('Suites :')
    for nom, tests in SUITES.items():
        print(f"  {nom:<16} {' '.join(tests)}")


def ordre(noms):
    """Tests dans l'ordre demandé, regroupés pour changer de config le moins possible : PC, puis simu
    sans UWB (smoke en premier, sans le code du drone), puis indifférents, puis avec UWB."""
    def cle(nom):
        t = TESTS[nom]
        if not t.commande:
            return 0
        if t.uwb is False or (t.uwb is None and not t.drone):
            return 1 if not t.drone else 2
        return 3 if t.uwb is None else 4
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
                try:
                    simu.preparer(test)
                except ConfigIncompatible as e:
                    print(f'  TEST SAUTÉ : {e}', flush=True)
                    journal.write_text(f'Test sauté : {e}\n')
                    bilan.append((nom, False, time.monotonic() - debut))
                    continue
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
