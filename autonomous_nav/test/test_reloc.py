# Test sin robot de la relocalización con las paredes: decidir_reloc (fase1_node) y el ajuste de
# comprobar_pose (estimar) sobre scans SINTÉTICOS generados con las mismas caras del mapa (maps/paredes.yaml).
# No simula el robot: comprueba que el algoritmo recupera una pose conocida a partir de una pose errónea.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_reloc.py -v
import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav import comprobar_pose as cp  # noqa: E402
from autonomous_nav.fase1_node import decidir_reloc  # noqa: E402

N = 360
INFO = (0.0, 2.0 * math.pi / N, 0.12, 3.5)
OFF = -0.032                       # lidar_offset_x
LIMITES = dict(max_dist=0.30, max_ang=math.radians(3.0), max_rms=0.02, min_puntos=30)

# Delante de la puerta el 05/11: pose real (paredes) y la que creía el robot (odometría)
REAL_PUERTA = (5.833, 11.005, math.radians(103.8))
CREIDA_PUERTA = (5.696, 11.017, 1.827)


@pytest.fixture(scope='module')
def caras():
    return cp.cargar_caras()


def scan_sintetico(pose, caras, giro_caras_x=0.0):
    # Distancias que vería el LiDAR desde 'pose' contra las caras del mapa (nan si no hay pared o más de 3.5 m).
    # giro_caras_x: las caras X se ven giradas ese ángulo (paredes no perpendiculares).
    x, y, th = pose
    a = INFO[0] + np.arange(N) * INFO[1]
    L = (x + OFF * math.cos(th), y + OFF * math.sin(th))
    d, cara = cp.trazar(L, th + a, caras)
    if giro_caras_x:
        d2, cara2 = cp.trazar(L, th + giro_caras_x + a, caras)
        es_x = (cara >= 0) & (caras['eje'][np.maximum(cara, 0)] == 0)
        es_x2 = (cara2 >= 0) & (caras['eje'][np.maximum(cara2, 0)] == 0)
        d = np.where(es_x, np.where(es_x2, d2, np.nan), d)
    d[d > INFO[3]] = np.nan
    return d


def error(est, real):
    return math.hypot(est[0] - real[0], est[1] - real[1]), abs(math.degrees(cp.normalizar(est[2] - real[2])))


def test_recupera_la_pose_real_delante_de_la_puerta(caras):
    r = scan_sintetico(REAL_PUERTA, caras)
    x, y, th, resumen = cp.estimar(CREIDA_PUERTA, OFF, r, INFO, caras, '')
    dxy, dth = error((x, y, th), REAL_PUERTA)
    assert dxy < 0.01 and dth < 0.2, (dxy, dth)
    assert {c['eje'] for c in resumen} == {'X', 'Y'}
    assert decidir_reloc(CREIDA_PUERTA, (x, y, th), resumen, **LIMITES) == (True, '')


def test_recupera_la_pose_en_A_con_el_rumbo_de_la_pared_izquierda(caras):
    real = (0.71, 0.92, math.radians(1.0))
    r = scan_sintetico(real, caras)
    x, y, th, _ = cp.estimar((0.70, 0.90, 0.0), OFF, r, INFO, caras, 'Y')
    dxy, dth = error((x, y, th), real)
    assert dxy < 0.01 and dth < 0.2, (dxy, dth)


def test_eje_rumbo_y_ignora_una_pared_x_girada(caras):
    # La pared del fondo (caras X) se ve girada 1.5°: con eje_rumbo 'Y' el rumbo sale de la pared izquierda
    real = (0.71, 0.92, math.radians(1.0))
    r = scan_sintetico(real, caras, giro_caras_x=math.radians(1.5))
    _, _, th_y, _ = cp.estimar((0.70, 0.90, 0.0), OFF, r, INFO, caras, 'Y')
    _, _, th_todas, _ = cp.estimar((0.70, 0.90, 0.0), OFF, r, INFO, caras, '')
    assert abs(math.degrees(th_y - real[2])) < 0.2
    assert abs(math.degrees(th_todas - real[2])) > 0.3      # mezclando las dos, el rumbo se desvía


def cara(eje, n=50, rms=0.005):
    return {'eje': eje, 'n': n, 'rms': rms}


@pytest.mark.parametrize('resumen, nueva, motivo', [
    ([cara('X'), cara('X')], (0.05, 0.0, 0.0), 'un_eje'),
    ([cara('X', 10), cara('Y', 10)], (0.05, 0.0, 0.0), 'pocos_puntos'),
    ([cara('X'), cara('Y', rms=0.03)], (0.05, 0.0, 0.0), 'residuo'),
    ([cara('X'), cara('Y')], (0.25, 0.25, 0.0), 'dxy'),
    ([cara('X'), cara('Y')], (0.05, 0.0, math.radians(4.0)), 'dth'),
    ([cara('X'), cara('Y')], (0.20, 0.0, math.radians(2.0)), ''),
])
def test_decidir_reloc(resumen, nueva, motivo):
    ok, m = decidir_reloc((0.0, 0.0, 0.0), nueva, resumen, **LIMITES)
    assert m == motivo and ok == (motivo == '')
