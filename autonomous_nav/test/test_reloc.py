# Test sin robot de la relocalización con la pared de la puerta (ajuste_puerta de fase1_node).
# Los puntos son SINTÉTICOS: rayos del LiDAR desde una pose conocida contra la recta x = 6.39 con el hueco de la
# puerta (y, si se pide, la hoja abierta), pasados al mundo con una pose creída errónea (como los ve el nodo).
# No simula el robot: comprueba que el algoritmo recupera la corrección y el nivel (total / parcial / nada).
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
REAL = (5.915, 11.505, math.radians(81.0))   # en E-Door_1 (con la zona hoja_puerta), llegando desde E
PARAM = dict(x_pared=X_PARED, y_hueco=(11.27, 12.02), ancho=0.76, tol_ancho=0.06, banda=0.35, largo=1.0,
             max_rms=0.02, min_puntos=30, min_cercano=10, max_dist=0.30, max_ang=math.radians(3.0))


def puntos(real, creida, hueco=HUECO, hoja=None, ruido=0.0, semilla=0):
    # Rayos cada 1° desde el LiDAR en la pose real contra la pared con el hueco y, si se pide, la hoja de la puerta
    # (0.7 m, perpendicular a la pared): 'sala_lejano' (bisagras en el borde lejano, abierta hacia la sala: como la
    # real) o 'sala_cercano' (en el borde cercano, hacia la sala).
    # Devuelve los puntos vistos con la pose creída: P_creído = r_creído + R(e) (P - r_real), e = th_creído - th_real.
    xr, yr, thr = real
    lx, ly = xr + OFF * math.cos(thr), yr + OFF * math.sin(thr)
    y_hoja = hueco[1] if hoja == 'sala_lejano' else hueco[0]
    x0, x1 = X_PARED - 0.7, X_PARED
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
            th_ = (y_hoja - ly) / s
            xh = lx + th_ * c
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


def ajustar(c, **kw):
    px, py = puntos(REAL, c, **kw)
    return ajuste_puerta(px, py, c[0], c[1], **PARAM)


def bien(r, dx, dy, dth_grados, tol=0.015, tol_grados=0.2):
    return abs(r[0] - dx) < tol and abs(r[1] - dy) < tol and abs(math.degrees(r[2]) - dth_grados) < tol_grados


# ---------- Corrección total ----------

def test_recupera_el_error_de_la_ruta_3():
    # Como el 05/11 delante de la puerta: ~14 cm en X, ~20 cm en Y y ~1° de rumbo
    r = ajustar(creida(-0.137, -0.20, 0.9))
    assert r[3] == 'total' and bien(r, 0.137, 0.20, -0.9)
    assert not r[5]['aviso_ancho']


def test_hoja_abierta_hacia_la_sala_en_el_lado_lejano():
    # Como la real: tapa el tramo de pared del lado lejano. La Y sale del lado cercano: corrección total.
    r = ajustar(creida(-0.10, 0.12, -0.5), hoja='sala_lejano')
    assert r[3] == 'total' and bien(r, 0.10, -0.12, 0.5)


def test_ruido_de_1_cm():
    # El rumbo sale solo del tramo cercano (~35 puntos en ~1 m): con 1 cm de ruido, sigma de la pendiente
    # ~ 1 cm / (raíz(35) * 0.3 m) ~ 0.17°, así que se admiten 0.4° (2 sigma). En el bag real el rms fue ~0.65 cm.
    r = ajustar(creida(0.05, -0.05, 0.3), ruido=0.01)
    assert r[3] == 'total' and bien(r, -0.05, 0.05, -0.3, tol=0.02, tol_grados=0.4)


def test_hueco_estrecho_solo_avisa():
    # Riesgo aceptado: el ancho es solo un aviso. Hueco de 0.50 m con el borde cercano en 11.40 (13 cm más allá):
    # la Y se corrige mal (13 cm) y lo único que lo delata es el aviso de ancho.
    r = ajustar(creida(0.05, 0.05, 0.0), hueco=(11.40, 11.90))
    assert r[3] == 'total' and r[5]['aviso_ancho']


# ---------- Corrección parcial (dx y dth sí, dy no) ----------

def test_puerta_cerrada():
    r = ajustar(creida(0.05, 0.05, 0.0), hueco=(0.0, 0.0))
    assert r[3] == 'parcial' and r[4] == 'dy'


def test_error_de_y_demasiado_grande():
    r = ajustar(creida(0.0, -0.33, 0.0))
    assert r[3] == 'parcial' and r[4] == 'dy' and abs(r[0]) < 0.015


# ---------- Rechazo (la recta no es fiable: nada) ----------

def test_sin_pared_clara():
    # Una nube de puntos en la banda (sillas, personas) en lugar de la pared: no se corrige nada
    rng = np.random.default_rng(3)
    px = rng.uniform(6.05, 6.70, 200)
    py = rng.uniform(10.0, 12.5, 200)
    c = creida(0.0, 0.0, 0.0)
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3:5] in (('', 'residuo'), ('', 'pocos_puntos'))


def test_pocos_puntos():
    c = creida(0.05, 0.05, 0.0)
    px, py = puntos(REAL, c)
    assert ajuste_puerta(px[:20], py[:20], c[0], c[1], **PARAM)[3:5] == ('', 'pocos_puntos')


def test_error_de_x_demasiado_grande():
    assert ajustar(creida(-0.33, 0.0, 0.0))[3:5] == ('', 'dxy')


def test_error_de_rumbo_demasiado_grande():
    assert ajustar(creida(0.0, 0.0, 3.5))[3:5] == ('', 'dth')
