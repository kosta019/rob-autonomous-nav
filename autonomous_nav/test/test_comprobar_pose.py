# Test sin robot de los avisos de comprobar_pose: ventana sucia (fraccion_con_ecos) y giróscopo (aviso_giroscopo).
# Los scans son como los que guarda comprobar_pose: arrays con nan donde no hay dato.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_comprobar_pose.py -v
import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.comprobar_pose import ECOS_MAX, aviso_giroscopo, fraccion_con_ecos  # noqa: E402

DIST = 0.20   # obstacle_min_range de fase1.yaml


def scans(con_eco, total=25):
    # 'total' scans con paredes a ~1 m y algún nan; los 'con_eco' primeros llevan además una lectura a 0.15 m
    lista = []
    for i in range(total):
        s = np.full(360, 1.0)
        s[10:20] = np.nan
        if i < con_eco:
            s[146] = 0.15
        lista.append(s)
    return lista


def test_sin_ecos():
    assert fraccion_con_ecos(scans(0), DIST) == 0.0


def test_un_eco_de_25_no_avisa():
    assert fraccion_con_ecos(scans(1), DIST) == pytest.approx(0.04)
    assert not fraccion_con_ecos(scans(1), DIST) > ECOS_MAX


def test_dos_ecos_de_25_avisa():
    assert fraccion_con_ecos(scans(2), DIST) > ECOS_MAX


def test_nan_no_cuenta_como_eco():
    todo_nan = [np.full(360, np.nan) for _ in range(25)]
    assert fraccion_con_ecos(todo_nan, DIST) == 0.0


def test_lectura_justo_en_el_limite_no_es_eco():
    s = np.full(360, 1.0)
    s[0] = DIST
    assert fraccion_con_ecos([s], DIST) == 0.0


# ---------- Aviso del giróscopo (robot quieto) ----------

def test_giroscopo_bien():
    assert aviso_giroscopo([0.0, 0.001, -0.001] * 30, [0.1] * 100) is None


def test_giroscopo_con_giro_falso_avisa():
    # 05/11: la OpenCR se encendió moviendo el robot y /imu marcaba 0.63 rad/s quieto
    aviso = aviso_giroscopo([0.63] * 100, [0.1] * 100)
    assert aviso is not None and 'reinicia la OpenCR' in aviso


def test_deriva_del_rumbo_avisa():
    # /imu casi quieto pero el rumbo de /odom cambia 0.3° en la ventana
    yaws = list(np.linspace(0.0, math.radians(0.3), 100))
    assert aviso_giroscopo([0.0] * 100, yaws) is not None


def test_cruzar_180_grados_no_es_deriva():
    yaws = [math.pi - 0.0005, -math.pi + 0.0005] * 50
    assert aviso_giroscopo([0.0] * 100, yaws) is None


def test_sin_imu_avisa():
    assert aviso_giroscopo([], [0.1] * 100) is not None


# ---------- Rumbo de una sola pared (estimar con eje_rumbo) sobre scans sintéticos con las caras del mapa ----------

from autonomous_nav import comprobar_pose as cp  # noqa: E402

INFO = (0.0, 2.0 * math.pi / 360, 0.12, 3.5)
OFF = -0.032


@pytest.fixture(scope='module')
def caras():
    return cp.cargar_caras()


def scan_sintetico(pose, caras, giro_caras_x=0.0):
    # Distancias desde 'pose' contra las caras del mapa (nan sin pared o a más de 3.5 m).
    # giro_caras_x: las caras X se ven giradas ese ángulo (paredes no perpendiculares).
    x, y, th = pose
    a = INFO[0] + np.arange(360) * INFO[1]
    L = (x + OFF * math.cos(th), y + OFF * math.sin(th))
    d, cara = cp.trazar(L, th + a, caras)
    if giro_caras_x:
        d2, cara2 = cp.trazar(L, th + giro_caras_x + a, caras)
        es_x = (cara >= 0) & (caras['eje'][np.maximum(cara, 0)] == 0)
        es_x2 = (cara2 >= 0) & (caras['eje'][np.maximum(cara2, 0)] == 0)
        d = np.where(es_x, np.where(es_x2, d2, np.nan), d)
    d[d > INFO[3]] = np.nan
    return d


def test_recupera_la_pose_en_A_con_el_rumbo_de_la_pared_izquierda(caras):
    real = (0.71, 0.92, math.radians(1.0))
    x, y, th, _ = cp.estimar((0.70, 0.90, 0.0), OFF, scan_sintetico(real, caras), INFO, caras, 'Y')
    assert math.hypot(x - real[0], y - real[1]) < 0.01 and abs(math.degrees(th - real[2])) < 0.2


def test_eje_rumbo_y_ignora_una_pared_x_girada(caras):
    # La pared del fondo (caras X) se ve girada 1.5°: con 'Y' el rumbo sale de la pared izquierda
    real = (0.71, 0.92, math.radians(1.0))
    r = scan_sintetico(real, caras, giro_caras_x=math.radians(1.5))
    _, _, th_y, _ = cp.estimar((0.70, 0.90, 0.0), OFF, r, INFO, caras, 'Y')
    _, _, th_todas, _ = cp.estimar((0.70, 0.90, 0.0), OFF, r, INFO, caras, '')
    assert abs(math.degrees(th_y - real[2])) < 0.2
    assert abs(math.degrees(th_todas - real[2])) > 0.3      # mezclando las dos, el rumbo se desvía
