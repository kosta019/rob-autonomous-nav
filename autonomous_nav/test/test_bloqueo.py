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


# reloc_adelantable: un bloqueo justo antes del waypoint de relocalización (06/11 13:44: el marco de la puerta visto en
# el hueco con la Y +0.37 m, a 0.21 m de ese waypoint) relocaliza ahí en vez de esperar.
ANTES_PUERTA = (5.937, 11.241)


def nodo_reloc(objetivos, hecha=False):
    return SimpleNamespace(objetivos=objetivos, i_objetivo=0, reloc_antes_de='Door', reloc_bloqueo_dist=0.50,
                           reloc_adelantada=hecha)


def test_bloqueo_junto_a_la_puerta_relocaliza():
    n = nodo_reloc([('replan6_3', 5.87, 11.41, 0.07), ('Door', 6.49, 11.682, 0.02)])
    assert Fase1Node.reloc_adelantable(n, *ANTES_PUERTA)


def test_solo_una_vez():
    n = nodo_reloc([('replan6_3', 5.87, 11.41, 0.07), ('Door', 6.49, 11.682, 0.02)], hecha=True)
    assert not Fase1Node.reloc_adelantable(n, *ANTES_PUERTA)


def test_lejos_del_waypoint_no():
    n = nodo_reloc([('E-Door_2', 5.915, 11.455, 0.07), ('Door', 6.49, 11.682, 0.02)])
    assert not Fase1Node.reloc_adelantable(n, 5.70, 10.60)


def test_si_el_siguiente_no_es_la_puerta_no():
    n = nodo_reloc([('E-Door_1', 5.115, 7.005, 0.07), ('E-Door_2', 5.915, 11.455, 0.07)])
    assert not Fase1Node.reloc_adelantable(n, 5.10, 6.90)
