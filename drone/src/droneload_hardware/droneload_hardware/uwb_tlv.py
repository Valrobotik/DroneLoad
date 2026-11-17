"""API binaire TLV du firmware PANS (DWM1001) : requête dwm_loc_get et décodage de sa réponse.

Fonctions pures (ni ROS ni port série) : utilisées par le nœud uwb et par le faux tag de
tests/simu/uwb_pilote.py. Format d'après le DWM1001 Firmware API Guide, À VÉRIFIER sur le vrai tag :

  requête  : 0C 00                                    (dwm_loc_get, sans argument)
  réponse  : 40 01 <err>                              (code de retour, 0 = OK)
             41 0D <x> <y> <z> <qf>                   (position du tag : int32 little-endian en mm, qf 0-100)
             49 <len> <n> n x [<adresse 2> <distance 4> <qf 1> <position 13>]   (ancres utilisées)

Après un reset, le module est en mode TLV. S'il a été laissé en mode shell (terminal ouvert à la main),
il affiche « dwm> » ou des lignes « POS,... » : « quit » le ramène en mode TLV.
"""
import struct
from collections import namedtuple

REQUETE_LOC = b'\x0c\x00'
QUITTER_SHELL = b'quit\r'
Position = namedtuple('Position', 'x y z qf ancres')   # x, y, z en mm ; ancres = nombre d'ancres utilisées

_RETOUR = b'\x40\x01'
_POSITION = b'\x41\x0d'
_ANCRES = 0x49
_TAILLE_ANCRE = 20


def lire_reponse(tampon):
    """Décode le début du tampon : (résultat, octets consommés).

    résultat : Position, int (code d'erreur du module) ou None. (None, 0) : réponse incomplète, attendre la
    suite ; (None, 1) : octet inattendu sauté (resynchronisation).
    """
    if len(tampon) < 3:
        return None, 0
    if tampon[:2] != _RETOUR:
        return None, 1
    if tampon[2] != 0:
        return tampon[2], 3
    if len(tampon) < 20:
        return None, 0
    if tampon[3:5] != _POSITION or tampon[18] != _ANCRES:
        return None, 1
    fin = 20 + tampon[19]
    if len(tampon) < fin:
        return None, 0
    x, y, z, qf = struct.unpack_from('<iiiB', tampon, 5)
    ancres = tampon[20] if tampon[19] else 0
    return Position(x, y, z, qf, ancres), fin


def est_shell(tampon):
    """Le module répond en mode shell (texte) au lieu du mode TLV."""
    return b'dwm>' in tampon or b'POS,' in tampon


def encoder_loc(x, y, z, qf, ancres=4):
    """Réponse à dwm_loc_get telle que l'envoie un tag (x, y, z en mm) : pour le faux tag des tests."""
    position = struct.pack('<iiiB', x, y, z, qf)
    liste = bytes([ancres]) + b''.join(
        struct.pack('<HIB', 0x1000 + i, 2000, 100) + struct.pack('<iiiB', 0, 0, 0, 100) for i in range(ancres))
    return _RETOUR + b'\x00' + _POSITION + position + bytes([_ANCRES, len(liste)]) + liste
