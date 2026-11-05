# Test sin robot de la relocalización con la pared de la puerta (ajuste_puerta de fase1_node).
# Los puntos son SINTÉTICOS: rayos del LiDAR (1 por grado) desde una pose conocida contra la geometría vista en vivo
# el 05/11 (canaleta, escalón de yeso, lateral del marco, pared lejana y hoja de la puerta), pasados al mundo con una
# pose creída errónea, como los ve el nodo. No simula el robot: comprueba que el algoritmo recupera la corrección,
# el nivel (total / parcial / nada) y que al repetir desde la pose corregida sale ~0.
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

OFF = -0.032                                   # lidar_offset_x
REAL = (5.915, 11.455, math.radians(79.8))     # último punto antes de Door (E-Door_2), llegando desde E-Door_1
PARAM = dict(x_pared=6.344, y_borde=11.245, ancho=0.76, tol_ancho=0.06, banda=0.35, largo=1.0, max_rms=0.02,
             min_puntos=20, min_cercano=10, max_dx=0.30, max_dy=0.45, max_ang=math.radians(3.0),
             fondo_marco=0.08)

# Geometría vista en vivo (segmentos en el mundo)
MARCO_CERCANO = ((6.39, 11.245), (6.55, 11.245))
PARED = [((6.344, 10.07), (6.344, 11.14)),     # canaleta del lado cercano
         ((6.344, 11.14), (6.39, 11.14)),      # final de la canaleta
         ((6.39, 11.14), (6.39, 11.245)),      # escalón de yeso
         MARCO_CERCANO,                        # lateral del marco (borde cercano)
         ((6.55, 12.015), (6.39, 12.015)),     # lateral del marco (borde lejano): hueco de 0.77 m
         ((6.344, 12.02), (6.344, 12.37))]     # canaleta del lado lejano
HOJA = [((5.59, 12.015), (6.344, 12.015))]     # hoja abierta hacia la sala, bisagras en el borde lejano
PERSONA = [((6.30, 11.40), (6.40, 11.40)), ((6.40, 11.40), (6.40, 11.55)),
           ((6.40, 11.55), (6.30, 11.55)), ((6.30, 11.55), (6.30, 11.40))]   # alguien en el hueco


def puntos(real, creida, segmentos=PARED, ruido=0.0, semilla=0):
    # Rayo cada 1° desde el LiDAR en la pose real; el impacto más cercano con los segmentos (hasta 3.5 m), visto
    # con la pose creída: P_creído = r_creído + R(e) (P - r_real), con e = th_creído - th_real.
    xr, yr, thr = real
    lx, ly = xr + OFF * math.cos(thr), yr + OFF * math.sin(thr)
    rng = np.random.default_rng(semilla)
    pts = []
    for ang in thr + np.radians(np.arange(360)):
        c, s = math.cos(ang), math.sin(ang)
        mejor = None
        for (x1, y1), (x2, y2) in segmentos:
            ex, ey = x2 - x1, y2 - y1
            den = c * ey - s * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - lx) * ey - (y1 - ly) * ex) / den          # distancia a lo largo del rayo
            u = ((x1 - lx) * s - (y1 - ly) * c) / den            # posición en el segmento (0..1)
            if t > 0 and 0.0 <= u <= 1.0 and (mejor is None or t < mejor):
                mejor = t
        if mejor is not None and mejor < 3.5:
            d = mejor + rng.normal(0.0, ruido)
            pts.append((lx + d * c, ly + d * s))
    p = np.array(pts)
    e = creida[2] - real[2]
    ce, se = math.cos(e), math.sin(e)
    qx = creida[0] + ce * (p[:, 0] - xr) - se * (p[:, 1] - yr)
    qy = creida[1] + se * (p[:, 0] - xr) + ce * (p[:, 1] - yr)
    return qx, qy


def creida(dx, dy, dth_grados, real=REAL):
    return (real[0] + dx, real[1] + dy, real[2] + math.radians(dth_grados))


def ajustar(c, real=REAL, **kw):
    px, py = puntos(real, c, **kw)
    return ajuste_puerta(px, py, c[0], c[1], **PARAM)


def bien(r, dx, dy, dth_grados, tol=0.015, tol_grados=0.2):
    return abs(r[0] - dx) < tol and abs(r[1] - dy) < tol and abs(math.degrees(r[2]) - dth_grados) < tol_grados


# ---------- Corrección total ----------

def test_recupera_el_error_de_la_ruta_3():
    # ~14 cm en X, ~20 cm en Y y ~1° de rumbo; el escalón de yeso y el lateral del marco no tuercen la recta
    r = ajustar(creida(-0.137, -0.20, 0.9))
    assert r[3] == 'total' and bien(r, 0.137, 0.20, -0.9)


def test_con_la_hoja_abierta():
    r = ajustar(creida(-0.10, 0.12, -0.5), segmentos=PARED + HOJA)
    assert r[3] == 'total' and bien(r, 0.10, -0.12, 0.5)


def test_robot_movido_10_cm_hacia_E():
    # Como la prueba en vivo (b): el robot está 10 cm más hacia -Y de lo que cree
    r = ajustar(creida(0.0, 0.10, 0.0))
    assert r[3] == 'total' and bien(r, 0.0, -0.10, 0.0)


def test_ruido_de_1_cm():
    # ~30-36 puntos en ~1 m de canaleta: con 1 cm de ruido la pendiente tiene sigma ~0.2°; se admiten 0.5°.
    # En vivo el rms fue ~0.5 cm.
    r = ajustar(creida(0.05, -0.05, 0.3), ruido=0.01)
    assert r[3] == 'total' and bien(r, -0.05, 0.05, -0.3, tol=0.02, tol_grados=0.5)


def test_repetir_desde_la_pose_corregida_da_cero():
    # Lo que hace el nodo tras corregir (si no, RELOC_INCONSISTENTE): |dx|, |dy| < 3 cm y |dth| < 0.5°
    c = creida(-0.137, -0.20, 0.9)
    r = ajustar(c)
    r2 = ajustar((c[0] + r[0], c[1] + r[1], c[2] + r[2]))
    assert r2[3] == 'total' and abs(r2[0]) < 0.03 and abs(r2[1]) < 0.03 and abs(math.degrees(r2[2])) < 0.5


# ---------- Corrección parcial (dx y dth sí, dy no) ----------

def test_alguien_en_el_hueco():
    # Algo sobre la línea de pared justo después del borde: no se fía de la Y. Según dónde esté, el tramo de pared
    # se alarga hasta ello y el marco no aparece ('sin_borde') o aparece con algo detrás ('borde').
    r = ajustar(creida(-0.05, 0.05, 0.0), segmentos=PARED + PERSONA)
    assert r[3] == 'parcial' and r[4] in ('borde', 'sin_borde')
    assert abs(r[0] - 0.05) < 0.015


def test_marco_no_visible():
    # Sin el lateral del marco (tapado): no hay borde
    r = ajustar(creida(-0.05, 0.05, 0.0), segmentos=[s for s in PARED if s != MARCO_CERCANO])
    assert r[3] == 'parcial' and r[4] == 'sin_borde'


def test_error_de_y_demasiado_grande():
    # 0.50 m de error en Y: no se acepta la Y (o, si la ventana ya no contiene el borde, ni siquiera la recta)
    r = ajustar(creida(0.0, 0.50, 0.0))
    assert r[3] != 'total' and r[4] != ''


# ---------- Rechazo (la recta no es fiable: nada) ----------

def test_sin_pared_clara():
    rng = np.random.default_rng(3)
    px = rng.uniform(6.05, 6.70, 200)
    py = rng.uniform(10.0, 12.5, 200)
    c = creida(0.0, 0.0, 0.0)
    assert ajuste_puerta(px, py, c[0], c[1], **PARAM)[3:5] in (('', 'residuo'), ('', 'pocos_puntos'))


def test_pocos_puntos():
    c = creida(0.05, 0.05, 0.0)
    px, py = puntos(REAL, c)
    assert ajuste_puerta(px[:15], py[:15], c[0], c[1], **PARAM)[3:5] == ('', 'pocos_puntos')


def test_error_de_x_demasiado_grande():
    assert ajustar(creida(-0.33, 0.0, 0.0))[3:5] == ('', 'dx')


def test_error_de_rumbo_demasiado_grande():
    assert ajustar(creida(0.0, 0.0, 3.5))[3:5] == ('', 'dth')
