# Test sin robot de ruta_bloqueada (fase1_node): hasta dónde mira la comprobación de bloqueo.
# No simula el robot: llama al método real con un objeto que solo tiene los atributos que usa.
# Geometría del 06/11 (13:17): silla a 15 cm de E, sustituto E~ elegido y el robot a 8 cm de él.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_bloqueo.py -v
import os
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import Fase1Node  # noqa: E402

ROBOT = (4.982, 5.680)
SILLA = (5.222, 6.167)


def nodo_falso(objetivos, obstaculo):
    return SimpleNamespace(
        objetivos=objetivos, i_objetivo=0, alcance_bloqueo=0.40, inflado=0.25,
        obligatorios=['E', 'Door', 'Q'], sustitutos={'E~': ('E', 5.082, 6.122)},
        obstaculos={(0, 0): [obstaculo[0], obstaculo[1], 0.0, 0]})


def test_no_mira_mas_alla_del_sustituto():
    # El tramo E~ -> E-Door_1 pasa a 0.236 m de la silla, pero aún no toca: el robot puede llegar a E~
    n = nodo_falso([('E~', 4.965, 5.755, 0.07), ('E-Door_1', 5.115, 7.005, 0.07)], SILLA)
    assert Fase1Node.ruta_bloqueada(n, *ROBOT) is None


def test_un_obstaculo_antes_del_sustituto_si_bloquea():
    n = nodo_falso([('E~', 4.965, 5.755, 0.07), ('E-Door_1', 5.115, 7.005, 0.07)], (4.90, 5.72))
    assert Fase1Node.ruta_bloqueada(n, *ROBOT) is not None


def test_sigue_mirando_tras_un_waypoint_intermedio():
    # Pasado un waypoint que no es obligatorio, se sigue mirando hasta completar block_lookahead
    n = nodo_falso([('replan1_1', 4.982, 5.780, 0.07), ('E', 5.082, 6.122, 0.02)], (5.10, 5.95))
    assert Fase1Node.ruta_bloqueada(n, *ROBOT) is not None
