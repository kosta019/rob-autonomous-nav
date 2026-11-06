# Test sin robot de ajuste_mapa (corrección de la pose con el mapa) y del respaldo de la Y de ajuste_puerta.
# Comprueba el algoritmo con geometría conocida; no simula el robot.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_corr_mapa.py -v
import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import ajuste_mapa, ajuste_puerta  # noqa: E402

META = {'resolucion': 0.02, 'origen_x': -3.0, 'origen_y': -3.0}
PAR = dict(ventana=0.15, sig_medida=0.03, sig_xy=0.03, sig_th=math.radians(1.0), min_puntos=20)
OFF = -0.032


def mapa_paredes(esquina=True):
    # Pared x = 1 y, si esquina, también y = 1. Distancia y gradiente analíticos.
    xs = -3.0 + (np.arange(300) + 0.5) * 0.02
    ys = -3.0 + (np.arange(300) + 0.5) * 0.02
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    d, gx, gy = np.abs(X - 1.0), np.sign(X - 1.0), np.zeros_like(X)
    if esquina:
        d2 = np.abs(Y - 1.0)
        c = d2 < d
        d, gx, gy = np.where(c, d2, d), np.where(c, 0.0, gx), np.where(c, np.sign(Y - 1.0), gy)
    return np.stack([d, gx, gy])


def scan(real, creida_th, esquina=True):
    # Rayos desde la pose real contra las paredes; (r, a) en el sistema del robot, como los da el LiDAR
    x, y, th = real
    lx, ly = x + OFF * math.cos(th), y + OFF * math.sin(th)
    r, a = [], []
    for i in range(0, 360, 2):
        ang = math.radians(i)
        c, s = math.cos(th + ang), math.sin(th + ang)
        cand = []
        if c > 1e-6:
            cand.append((1.0 - lx) / c)
        if esquina and s > 1e-6:
            cand.append((1.0 - ly) / s)
        cand = [t for t in cand if t < 3.4]
        if cand:
            r.append(min(cand))
            a.append(ang)
    return np.array(r), np.array(a)


def test_esquina_recupera_x_y_y_rumbo():
    real = (0.0, 0.0, 0.3)
    r, a = scan(real, real[2])
    d = ajuste_mapa(r, a, 0.05, -0.04, 0.3 + math.radians(1.5), OFF, mapa_paredes(), META, **PAR)
    assert d[0] == pytest.approx(-0.05, abs=0.012)
    assert d[1] == pytest.approx(0.04, abs=0.012)
    assert d[2] == pytest.approx(-math.radians(1.5), abs=math.radians(0.3))


def test_una_pared_no_mueve_a_lo_largo_de_ella():
    # Lo importante: la coordenada que la pared no observa no se toca, aunque esté mal
    real = (0.0, 0.0, 0.0)
    r, a = scan(real, 0.0, esquina=False)
    d = ajuste_mapa(r, a, 0.06, 0.25, 0.0, OFF, mapa_paredes(esquina=False), META, **PAR)
    assert d[0] == pytest.approx(-0.06, abs=0.012)
    assert abs(d[1]) < 0.005


def test_objetos_lejos_de_la_pared_no_cuentan():
    real = (0.0, 0.0, 0.0)
    r, a = scan(real, 0.0)
    limpio = ajuste_mapa(r, a, 0.03, 0.0, 0.0, OFF, mapa_paredes(), META, **PAR)
    r2 = r.copy()
    r2[::6] = 0.5                       # patas de mesa a 0.5 m: a más de 'ventana' de cualquier pared
    d = ajuste_mapa(r2, a, 0.03, 0.0, 0.0, OFF, mapa_paredes(), META, **PAR)
    assert d[0] == pytest.approx(limpio[0], abs=0.01)
    assert d[1] == pytest.approx(limpio[1], abs=0.01)


def test_pocos_puntos_no_corrige():
    r, a = scan((0.0, 0.0, 0.0), 0.0)
    d = ajuste_mapa(r[:5], a[:5], 0.03, 0.0, 0.0, OFF, mapa_paredes(), META, **PAR)
    assert d[:3] == (0.0, 0.0, 0.0)


PUERTA = dict(x_pared=6.344, y_borde=11.245, ancho=0.76, tol_ancho=0.06, banda=0.35, largo=1.0, max_rms=0.02,
              min_puntos=20, min_cercano=10, max_dx=0.30, max_dy=0.45, max_ang=math.radians(3.0), fondo_marco=0.08)


def puntos_canaleta(desfase_y, escalon=True):
    # Canaleta hasta Y = 11.14 + desfase_y (la pose creída tiene la Y de más) y escalón de yeso, sin lateral del marco
    y = np.arange(10.2, 11.14, 0.012) + desfase_y
    x = np.full_like(y, 6.344)
    if escalon:
        y2 = np.array([11.16, 11.18, 11.20]) + desfase_y
        x2 = np.array([6.37, 6.38, 6.39])
        return np.r_[x, x2], np.r_[y, y2]
    return x, y


def test_sin_marco_la_y_sale_de_la_canaleta():
    px, py = puntos_canaleta(0.33)
    dx, dy, dth, nivel, motivo, _ = ajuste_puerta(px, py, 5.9, 11.2, y_fin_canaleta=11.14, **PUERTA)
    assert (nivel, motivo) == ('total', 'canaleta')
    assert dy == pytest.approx(-0.33, abs=0.02)


def test_sin_escalon_no_se_fia_del_final():
    px, py = puntos_canaleta(0.33, escalon=False)
    assert ajuste_puerta(px, py, 5.9, 11.2, y_fin_canaleta=11.14, **PUERTA)[3:5] == ('parcial', 'sin_borde')


def test_sin_referencia_sigue_como_antes():
    px, py = puntos_canaleta(0.33)
    assert ajuste_puerta(px, py, 5.9, 11.2, **PUERTA)[3:5] == ('parcial', 'sin_borde')
