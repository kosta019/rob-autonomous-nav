# Test sin robot de la relocalización con la pared de la puerta (ajuste_puerta de fase1_node).
# Los puntos son SINTÉTICOS: rayos del LiDAR desde una pose conocida contra la recta x = 6.39 con el hueco de la
# puerta, pasados al mundo con una pose creída errónea (como los ve el nodo). No simula el robot: comprueba que el
# algoritmo recupera la corrección y que las salvaguardas rechazan lo que deben.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_reloc.py -v
import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import ajuste_puerta  # noqa: E402

X_PARED = 6.39
HUECO = (11.265, 12.025)            # 0.76 m, como el medido
OFF = -0.032                        # lidar_offset_x
REAL = (5.965, 11.655, math.radians(81.0))   # en E-Door_1, llegando desde E
PARAM = dict(x_pared=X_PARED, y_hueco=(11.27, 12.02), ancho=0.76, tol_ancho=0.06, banda=0.35, largo=1.0,
             max_rms=0.02, min_puntos=30, max_dist=0.30, max_ang=math.radians(3.0))


def puntos(real, creida, hueco=HUECO, hoja=None, ruido=0.0, semilla=0):
    # Rayos cada 1° desde el LiDAR en la pose real contra la pared y, si hoja = 'sala' o 'vestibulo', contra la hoja
    # de la puerta abierta hacia ese lado (0.7 m perpendicular a la pared desde el borde y = hueco[0]). Devuelve los puntos en el mundo vistos
    # con la pose creída: P_creído = r_creído + R(e) (P - r_real), con e = th_creído - th_real.
    xr, yr, thr = real
    lx, ly = xr + OFF * math.cos(thr), yr + OFF * math.sin(thr)
    rng = np.random.default_rng(semilla)
    pts = []
    for ang in thr + np.radians(np.arange(360)):
        c, s = math.cos(ang), math.sin(ang)
        t = []
        if abs(c) > 1e-9:
            tp = (X_PARED - lx) / c
            yp = ly + tp * s
            if tp > 0 and not (hueco[0] < yp < hueco[1]):
                t.append(tp)
        if hoja and abs(s) > 1e-9:
            th_ = (hueco[0] - ly) / s
            xh = lx + th_ * c
            x0, x1 = (X_PARED - 0.7, X_PARED) if hoja == 'sala' else (X_PARED, X_PARED + 0.7)
            if th_ > 0 and x0 < xh < x1:
                t.append(th_)
        if t and min(t) < 3.5:
            d = min(t) + rng.normal(0.0, ruido)
            pts.append((lx + d * c, ly + d * s))
    p = np.array(pts)
    e = creida[2] - real[2]
    ce, se = math.cos(e), math.sin(e)
    qx = creida[0] + ce * (p[:, 0] - xr) - se * (p[:, 1] - yr)
    qy = creida[1] + se * (p[:, 0] - xr) + ce * (p[:, 1] - yr)
    return qx, qy


def creida(dx, dy, dth_grados, real=REAL):
    return (real[0] + dx, real[1] + dy, real[2] + math.radians(dth_grados))


def test_recupera_el_error_de_la_ruta_3():
    # Como el 05/11 delante de la puerta: ~14 cm en X, ~20 cm en Y y ~1° de rumbo
    c = creida(-0.137, -0.20, 0.9)
    px, py = puntos(REAL, c)
    dx, dy, dth, motivo, d = ajuste_puerta(px, py, c[0], c[1], **PARAM)
    assert motivo == ''
    assert abs(dx - 0.137) < 0.015 and abs(dy - 0.20) < 0.015 and abs(math.degrees(dth) + 0.9) < 0.2
    assert abs(d['ancho'] - 0.76) < 0.03


def test_hoja_abierta_hacia_el_vestibulo_se_ignora():
    # Solo añade unos puntos junto al borde del hueco: la corrección sigue siendo buena
    c = creida(-0.10, 0.12, -0.5)
    px, py = puntos(REAL, c, hoja='vestibulo')
    dx, dy, dth, motivo, _ = ajuste_puerta(px, py, c[0], c[1], **PARAM)
    assert motivo == ''
    assert abs(dx - 0.10) < 0.015 and abs(dy + 0.12) < 0.015 and abs(math.degrees(dth) - 0.5) < 0.2


def test_hoja_abierta_hacia_la_sala_se_rechaza():
    # Desde E-Door_1 la hoja tapa la pared de ese lado: el hueco no se puede medir y NO se debe corregir
    c = creida(-0.10, 0.12, -0.5)
    px, py = puntos(REAL, c, hoja='sala')
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3] in ('residuo', 'ancho', 'pocos_puntos')


def test_ruido_de_1_cm_se_acepta():
    c = creida(0.05, -0.05, 0.3)
    px, py = puntos(REAL, c, ruido=0.01)
    dx, dy, dth, motivo, _ = ajuste_puerta(px, py, c[0], c[1], **PARAM)
    assert motivo == ''
    assert abs(dx + 0.05) < 0.02 and abs(dy - 0.05) < 0.02


def test_ruido_de_4_cm_se_rechaza_por_residuo():
    c = creida(0.05, -0.05, 0.3)
    px, py = puntos(REAL, c, ruido=0.04)
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3] == 'residuo'


def test_puerta_cerrada_no_hay_hueco():
    c = creida(0.05, 0.05, 0.0)
    px, py = puntos(REAL, c, hueco=(0.0, 0.0))
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3] == 'ancho'


def test_hueco_de_50_cm_no_es_la_puerta():
    c = creida(0.05, 0.05, 0.0)
    px, py = puntos(REAL, c, hueco=(11.40, 11.90))
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3] == 'ancho'


def test_pocos_puntos():
    c = creida(0.05, 0.05, 0.0)
    px, py = puntos(REAL, c)
    assert ajuste_puerta(px[:20], py[:20], c[0], c[1], **PARAM)[3] == 'pocos_puntos'


def test_correccion_de_posicion_demasiado_grande():
    c = creida(0.0, -0.33, 0.0)
    px, py = puntos(REAL, c)
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3] == 'dxy'


def test_correccion_de_rumbo_demasiado_grande():
    c = creida(0.0, 0.0, 3.5)
    px, py = puntos(REAL, c)
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3] == 'dth'
