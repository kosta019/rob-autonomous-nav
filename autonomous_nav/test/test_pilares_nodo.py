# Tests de Fase1Node.detectar_pilares con los scans reales de la aproximación a R (06/11, ver test_pilares.py):
# qué escribe en el Mission Log y cuándo.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_pilares_nodo.py -v
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import Fase1Node  # noqa: E402

DATOS = np.load(os.path.join(RAIZ, 'test', 'datos', 'scans_estaciones.npz'))
PASILLO = [-0.55, 9.57, 13.00, 17.15]


def nodo_falso(zona=PASILLO, w=0.0):
    log = []
    n = SimpleNamespace(scan=None, scan_pilares=None, w_anterior=w, giro_max_obstaculos=0.2, zona_pilares=zona,
                        pilar_alcance=1.6, lidar_offset_x=-0.032, grupos_pilares=[], pilar_olvido=3.0,
                        pilar_min_det=3, estacion_lado=0.44, estacion_tol=0.05, estaciones=[], estacion_confirmar=5,
                        get_logger=lambda: SimpleNamespace(info=lambda *a: None),
                        escribir_log=lambda x, y, th, ev, err=None, ox=None, oy=None: log.append((ev, err, ox, oy)))
    return n, log


def pasar(n, tramo, veces=1):
    for r, (t, x, y, th) in zip(DATOS[tramo + '_scans'], DATOS[tramo + '_poses']):
        n.scan = SimpleNamespace(ranges=r, angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12,
                                 range_max=3.5)
        for _ in range(veces):      # el mismo mensaje, como cuando el timer va más rápido que el LiDAR
            Fase1Node.detectar_pilares(n, x, y, th, t)


def test_estacion_R_al_log_una_vez_con_sus_4_patas():
    n, log = nodo_falso()
    pasar(n, 'R_parado')
    assert [ev for ev, *_ in log] == ['ESTACION_CUADRADA'] + ['PILAR'] * 4
    _, ang, cx, cy = log[0]
    assert math.hypot(cx - 7.17, cy - 16.51) < 0.05      # cinta
    assert abs(ang) < 3                                 # grados: paralela a las paredes
    for _, _, px, py in log[1:]:
        assert abs(math.dist((px, py), (cx, cy)) - 0.44 / math.sqrt(2)) < 0.05
    assert len(n.estaciones) == 1 and n.estaciones[0]['vistas'] > 20


def test_mismo_scan_dos_veces_cuenta_una():
    # El timer va más rápido que el LiDAR: el mismo scan no se vuelve a contar
    n, _ = nodo_falso()
    pasar(n, 'R_parado', veces=1)
    vistas = n.estaciones[0]['vistas']
    n2, _ = nodo_falso()
    pasar(n2, 'R_parado', veces=3)
    assert n2.estaciones[0]['vistas'] == vistas


def test_fuera_de_la_zona_no_cuenta():
    n, log = nodo_falso(zona=[-0.55, 9.57, 13.00, 16.0])   # zona que deja fuera la estación
    pasar(n, 'R_parado')
    assert log == [] and n.estaciones == []


def test_girando_deprisa_no_detecta():
    n, log = nodo_falso(w=0.5)
    pasar(n, 'R_parado')
    assert log == []


def test_T_triangular_al_log():
    n, log = nodo_falso()
    pasar(n, 'T_rodeo')
    # Sus patas no son las de la estación de carga (la del enunciado es la cuadrada)
    assert [ev for ev, *_ in log] == ['ESTACION_TRIANGULAR'] + ['PILAR_DESCARTADA'] * 3
