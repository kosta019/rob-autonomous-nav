# Tests sin robot de la relocalización en Q con la esquina (ajuste_esquina) y del reintento de la relocalización en la
# puerta (reloc_reintentable), y de la llegada a Q en la máquina de estados. Geometría del 06/11: esquina de Q medida con
# cinta (pared X en 9.570 delante, pared Y en 13.997 a la derecha), robot real ~0.18 m antes en X, ~0.09 m más en Y y
# ~8.6° girado a la derecha de lo que cree.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_reloc_q.py -v
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import Fase1Node, ajuste_esquina  # noqa: E402

XW, YW = 9.570, 13.997
PAR = dict(banda=0.35, alcance=1.0, max_rms=0.02, min_puntos=15, max_corr=0.40, max_ang=math.radians(12),
           max_dif_ang=math.radians(5))


def esquina_vista(real, creida, giro_pared_x=0.0, ruido=0.0, semilla=0):
    # Puntos de las dos paredes vistos desde la pose real y pasados al mundo con la pose creída.
    # giro_pared_x: la pared X no es perpendicular a la Y (rad).
    rng = np.random.default_rng(semilla)
    yy = np.arange(real[1] - 0.9, real[1] + 0.9, 0.02)
    wx = np.c_[XW + (yy - real[1]) * math.tan(giro_pared_x), yy]
    xx = np.arange(real[0] - 0.05, real[0] + 0.75, 0.02)
    wy = np.c_[xx, np.full_like(xx, YW)]
    w = np.r_[wx, wy] + rng.normal(0, ruido, (len(wx) + len(wy), 2))
    # mundo -> robot real -> mundo con la pose creída
    c, s = math.cos(real[2]), math.sin(real[2])
    lx, ly = (w[:, 0] - real[0]) * c + (w[:, 1] - real[1]) * s, -(w[:, 0] - real[0]) * s + (w[:, 1] - real[1]) * c
    c, s = math.cos(creida[2]), math.sin(creida[2])
    return creida[0] + lx * c - ly * s, creida[1] + lx * s + ly * c


def test_corrige_x_y_y_rumbo():
    real, creida = (8.79, 14.59, math.radians(-8.6)), (8.97, 14.497, 0.0)
    px, py = esquina_vista(real, creida, ruido=0.005)
    dx, dy, dth, motivo, d = ajuste_esquina(px, py, creida[0], creida[1], XW, YW, **PAR)
    assert motivo == ''
    assert creida[0] + dx == pytest.approx(real[0], abs=0.01)
    assert creida[1] + dy == pytest.approx(real[1], abs=0.01)
    assert dth == pytest.approx(real[2], abs=math.radians(0.3))


def test_rumbo_solo_de_la_pared_y():
    # Pared X girada 3° (como en Q): el rumbo sale de la Y; con las dos, quedaría a medio camino
    real, creida = (8.79, 14.59, math.radians(-8.6)), (8.97, 14.497, 0.0)
    px, py = esquina_vista(real, creida, giro_pared_x=math.radians(3.0))
    dth_y = ajuste_esquina(px, py, creida[0], creida[1], XW, YW, **PAR)[2]
    dth_las_dos = ajuste_esquina(px, py, creida[0], creida[1], XW, YW, **PAR, eje_rumbo='')[2]
    assert dth_y == pytest.approx(real[2], abs=math.radians(0.3))
    assert abs(dth_las_dos - real[2]) > math.radians(1.0)


def test_repetido_desde_la_corregida_da_cero():
    real, creida = (8.79, 14.59, math.radians(-8.6)), (8.97, 14.497, 0.0)
    px, py = esquina_vista(real, creida)
    dx, dy, dth, _, _ = ajuste_esquina(px, py, creida[0], creida[1], XW, YW, **PAR)
    nueva = (creida[0] + dx, creida[1] + dy, dth)
    px2, py2 = esquina_vista(real, nueva)
    dx2, dy2, dth2, motivo, _ = ajuste_esquina(px2, py2, nueva[0], nueva[1], XW, YW, **PAR)
    assert motivo == '' and abs(dx2) < 0.005 and abs(dy2) < 0.005 and abs(dth2) < math.radians(0.1)


def test_paredes_muy_distintas_no_corrige():
    real, creida = (8.79, 14.59, 0.0), (8.97, 14.497, 0.0)
    px, py = esquina_vista(real, creida, giro_pared_x=math.radians(8.0))
    assert ajuste_esquina(px, py, creida[0], creida[1], XW, YW, **PAR)[3] == 'angulos'


def test_sin_pared_y_no_corrige():
    real, creida = (8.79, 14.59, 0.0), (8.97, 14.497, 0.0)
    px, py = esquina_vista(real, creida)
    lejos = py > YW + 0.5
    assert ajuste_esquina(px[lejos], py[lejos], creida[0], creida[1], XW, YW, **PAR)[3] == 'sin_pared_y'


# ---------- Reintento de la relocalización en la puerta ----------

def nodo_puerta(total=False, reintentos=0):
    return SimpleNamespace(reloc_puerta_total=total, reloc_reintentos=reintentos, reloc_reintento_max=10,
                           reloc_reintento_dist=1.0, reloc_antes_de='Door', i_objetivo=0,
                           objetivos=[('E-Door_2', 5.915, 11.455, 0.07), ('Door', 6.490, 11.682, 0.02)])


def test_sin_camino_junto_a_la_puerta_reintenta():
    # 06/11 14:36: RELOC_PARCIAL y SIN_CAMINO en (5.885, 11.28)
    assert Fase1Node.reloc_reintentable(nodo_puerta(), 5.885, 11.28)


def test_no_reintenta_si_ya_hubo_total_o_lejos_o_agotado():
    assert not Fase1Node.reloc_reintentable(nodo_puerta(total=True), 5.885, 11.28)
    assert not Fase1Node.reloc_reintentable(nodo_puerta(reintentos=10), 5.885, 11.28)
    assert not Fase1Node.reloc_reintentable(nodo_puerta(), 5.10, 7.00)   # E-Door_1: lejos de la puerta


def test_no_reintenta_pasada_la_puerta():
    n = nodo_puerta()
    n.objetivos = [('Door-Q_1', 7.415, 12.355, 0.07), ('Q', 8.970, 14.497, 0.02)]
    assert not Fase1Node.reloc_reintentable(n, 6.6, 11.8)


# ---------- Llegada a Q: relocaliza reloc_q_times veces antes de darlo por alcanzado ----------

def nodo_q():
    n = SimpleNamespace(
        estado='AVANZAR', i_objetivo=0, objetivos=[('Q', 8.970, 14.497, 0.02), ('final', 8.970, 14.497, 0.02)],
        reloc_q_en='Q', reloc_q_veces=2, reloc_q_hechas=0, reloc_n=10, reloc_antes_de='Door', sustitutos={},
        pose_final=[8.970, 14.497, 0.0], tol_final_yaw=0.03, k_angular=1.5, max_w=0.4, k_lineal=0.5, max_v=0.12,
        angulo_girar=0.17, angulo_reorientar=0.52, eventos=[])
    n.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n.escribir_log = lambda x, y, th, ev, *a: n.eventos.append(ev)
    n.empezar_reloc = lambda tipo: Fase1Node.empezar_reloc(n, tipo)
    return n


def test_en_q_relocaliza_dos_veces_y_luego_sigue():
    n = nodo_q()
    assert Fase1Node.maquina_estados(n, 8.965, 14.497, 0.0) == (0.0, 0.0)
    assert (n.estado, n.reloc_tipo, n.i_objetivo) == ('RELOC', 'Q', 0)
    n.estado = 'GIRAR'                      # lo que deja relocalizar_q
    Fase1Node.maquina_estados(n, 8.965, 14.497, 0.0)
    assert (n.estado, n.i_objetivo, n.reloc_q_hechas) == ('RELOC', 0, 2)
    n.estado = 'GIRAR'
    Fase1Node.maquina_estados(n, 8.965, 14.497, 0.0)
    assert n.i_objetivo == 1 and 'WAYPOINT Q' in n.eventos
    assert n.eventos[:2] == ['LLEGADA_RELOC_Q 1', 'LLEGADA_RELOC_Q 2']


# ---------- Rodeo largo: mejor esperar ----------

def test_rodeo_por_las_mesas_es_largo():
    # 06/11 15:10:38 (bag): quedaban 5.20 m hasta Door; el replan sin las mesas en el mapa daba 10.07 m
    from autonomous_nav.fase1_node import rodeo_largo
    assert rodeo_largo(10.07, 5.20, 1.5, 1.0)
    assert not rodeo_largo(5.19, 5.20, 1.5, 1.0)


def test_esquivar_en_el_pasillo_no_es_largo():
    # Rodear una silla añade ~0.5 m; con poco camino por delante, el margen fijo deja esquivar
    from autonomous_nav.fase1_node import longitud_camino, rodeo_largo
    antes = longitud_camino([(5.0, 5.2), (5.08, 6.12)])
    nuevo = longitud_camino([(5.0, 5.2), (4.70, 5.80), (5.08, 6.12)])
    assert not rodeo_largo(nuevo, antes, 1.5, 1.0)


def test_q_tapado_relocaliza_en_el_sustituto():
    n = nodo_q()
    n.objetivos = [('Q~', 8.80, 14.60, 0.07), ('final', 8.970, 14.497, 0.02)]
    n.sustitutos = {'Q~': ('Q', 8.970, 14.497)}
    Fase1Node.maquina_estados(n, 8.80, 14.60, 0.0)
    assert (n.estado, n.reloc_tipo) == ('RELOC', 'Q')
