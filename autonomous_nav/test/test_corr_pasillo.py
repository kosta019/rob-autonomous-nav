# Tests sin robot de la corrección en el pasillo con sus paredes medidas (ajuste_paredes) y de cuándo la usa el nodo
# (corregir_con_mapa: dentro de corridor_corr_zone, aunque la del mapa esté apagada). Geometría conocida; no simula el robot.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_corr_pasillo.py -v
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import Fase1Node, ajuste_paredes  # noqa: E402

EXTINTOR = ('y', 17.15, -0.55, 9.57)
PAR = dict(ventana=0.15, sig_medida=0.03, sig_xy=0.03, min_puntos=20)


def pared_y(y, x0, x1, n=50):
    # n puntos sobre la recta y = y, entre x0 y x1 (como los ve el LiDAR con la pose creída)
    xs = np.linspace(x0, x1, n)
    return xs, np.full(n, y)


def test_pared_y_corrige_solo_la_y():
    # La pose creída tiene 0.10 m de Y de más: la pared sale en 17.25. Con 50 puntos y las dos sigmas iguales, la
    # corrección es -0.10 * 50 / 51 (lo que pesan 50 medidas frente a la predicción de la odometría)
    px, py = pared_y(17.25, 2.0, 5.0)
    dx, dy, nx, ny = ajuste_paredes(px, py, [EXTINTOR], **PAR)
    assert dy == pytest.approx(-0.10 * 50 / 51, abs=1e-6)
    assert dx == 0.0 and nx == 0 and ny == 50


def test_fuera_del_tramo_no_cuenta():
    # Puntos a la Y de la pared pero más allá de su tramo (X > 9.57): no son de esa pared
    px, py = pared_y(17.25, 9.7, 11.0)
    assert ajuste_paredes(px, py, [EXTINTOR], **PAR) == (0.0, 0.0, 0, 0)


def test_lejos_de_la_pared_no_cuenta():
    # Lo que está a más de la ventana de la pared (una estación, una persona) no mueve la pose
    px, py = pared_y(17.15 - 0.40, 2.0, 5.0)
    assert ajuste_paredes(px, py, [EXTINTOR], **PAR) == (0.0, 0.0, 0, 0)


def test_pocos_puntos_no_corrige():
    px, py = pared_y(17.25, 2.0, 5.0, n=10)
    dx, dy, nx, ny = ajuste_paredes(px, py, [EXTINTOR], **PAR)
    assert (dx, dy, ny) == (0.0, 0.0, 10)


def test_pared_x_corrige_solo_la_x():
    # Una pared "x" (como sería el extremo X -0.55, aún sin medir): la X creída tiene 0.05 m de menos
    py = np.linspace(14.5, 16.5, 40)
    px = np.full(40, -0.55 - 0.05)
    dx, dy, nx, ny = ajuste_paredes(px, py, [EXTINTOR, ('x', -0.55, 13.78, 17.15)], **PAR)
    assert dx == pytest.approx(0.05 * 40 / 41, abs=1e-6)
    assert dy == 0.0 and nx == 40 and ny == 0


def nodo_falso(x, y, modo_mapa='apagado'):
    # Un scan que ve la pared del extintor 0.10 m más cerca de lo que dice la pose creída: el robot está en realidad
    # 0.10 m más cerca de ella (Y mayor)
    log = []
    n_rayos = 360
    ang = np.radians(np.arange(n_rayos))
    r = np.full(n_rayos, math.inf)
    lx = x - 0.032
    for i, a in enumerate(ang):
        if math.sin(a) > 0.5:     # rayos hacia +Y (rumbo 0): la pared a 17.15 - 0.10 - y
            r[i] = (17.15 - 0.10 - y) / math.sin(a)
    msg = SimpleNamespace(ranges=list(r), angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12,
                          range_max=3.5)
    n = SimpleNamespace(
        scan=msg, corr_ultimo_scan=None, corr_cuenta=0, corr_cada=1, corr_max_w=0.08, w_anterior=0.0, estado='AVANZAR',
        corr_modo=modo_mapa, corr_y_max=10.2, pasillo_modo='activo', pasillo_zona=[-0.55, 9.57, 13.90, 17.15],
        pasillo_alcance=2.0, pasillo_paredes=[EXTINTOR], alcance_min_obstaculos=0.20, lidar_offset_x=-0.032,
        lidar_yaw_offset=0.0, corr_ventana=0.15, corr_sig_medida=0.03, corr_sig_xy=0.03, corr_min_puntos=20,
        corr_max_xy=0.03, escribir_log=lambda x, y, th, ev, *a: log.append(ev), fijar_pose=lambda *p: log.append(p))
    n.corregir_con_paredes = lambda x, y, th: Fase1Node.corregir_con_paredes(n, x, y, th)
    assert lx < 9.57
    return n, log


def test_en_el_pasillo_corrige_aunque_el_mapa_este_apagado():
    # Antes de este cambio, con map_corr_mode "apagado" (o con Y > map_corr_y_max) no se corregía nada en el pasillo
    n, log = nodo_falso(3.34, 15.50)
    x, y, th = Fase1Node.corregir_con_mapa(n, 3.34, 15.50, 0.0)
    assert x == 3.34 and th == 0.0
    assert y == pytest.approx(15.50 + 0.03)          # +0.10 medido, limitado a map_corr_max_step
    assert any(isinstance(e, str) and e.startswith('CORR_PASILLO dx=+0.000 dy=+0.09') for e in log)


def test_fuera_del_pasillo_no_usa_las_paredes():
    # En la sala (Y < 13.90) no: allí corrige el mapa (si está activo) y ajuste_puerta
    n, log = nodo_falso(5.0, 10.5)
    assert Fase1Node.corregir_con_mapa(n, 5.0, 10.5, 0.0) == (5.0, 10.5, 0.0)
    assert log == []


def test_sombra_solo_registra():
    n, log = nodo_falso(3.34, 15.50)
    n.pasillo_modo = 'sombra'
    assert Fase1Node.corregir_con_mapa(n, 3.34, 15.50, 0.0) == (3.34, 15.50, 0.0)
    assert len(log) == 1 and log[0].startswith('CORR_PASILLO_SOMBRA')
