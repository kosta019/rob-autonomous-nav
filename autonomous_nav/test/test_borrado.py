# Test sin robot de la regla de borrado de obstáculos de fase1_node (actualizar_obstaculos, parte b).
# No simula el robot: llama al método real con un objeto que solo tiene los atributos que usa y con
# escaneos LaserScan hechos a mano. El robot está en (0, 0) mirando a +X y el obstáculo recordado,
# delante, sobre los rayos 359, 0 y 1.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_borrado.py -v
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import LaserScan  # noqa: E402

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import Fase1Node, rayos_libres  # noqa: E402

N = 360
MEMORIA = 8.0


def nodo_falso(dist_obstaculo=1.0):
    # Solo lo que usa actualizar_obstaculos. Holgura 0: ninguna lectura cuenta como obstáculo nuevo,
    # así el test mira solo el borrado del obstáculo que ya estaba en memoria.
    n = SimpleNamespace(
        w_anterior=0.0, giro_max_obstaculos=0.2, alcance_obstaculos=2.5, alcance_min_obstaculos=0.20,
        lidar_offset_x=0.0,
        meta={'resolucion': 0.05, 'origen_x': -5.0, 'origen_y': -5.0}, holgura=np.zeros((200, 200)),
        margen_estatico=0.20, separacion_log=0.20, obstaculos_log=[], memoria=MEMORIA,
        margen_borrado=0.10, escaneos_borrado=3, dist_min_cero=0.5, scan_borrado=None,
        escribir_log=lambda *args, **kwargs: None)
    n.obstaculos = {(0, 0): [dist_obstaculo, 0.0, 0.0, 0]}   # visto por última vez en t = 0
    return n


def scan(valor_delante):
    # Los rayos que apuntan al obstáculo (359, 0, 1) dan valor_delante; el resto, una pared a 3 m
    m = LaserScan()
    m.angle_min = 0.0
    m.angle_increment = 2.0 * math.pi / N
    m.range_min = 0.12
    m.range_max = 3.5
    r = [3.0] * N
    for i in (359, 0, 1):
        r[i] = valor_delante
    m.ranges = r
    return m


def pasar(n, valores, ahora=1.0):
    # Un scan NUEVO por cada valor. Devuelve, tras cada uno, si el obstáculo sigue en memoria.
    sigue = []
    for v in valores:
        n.scan = scan(v)
        Fase1Node.actualizar_obstaculos(n, 0.0, 0.0, 0.0, ahora)
        sigue.append((0, 0) in n.obstaculos)
    return sigue


def test_ceros_con_obstaculo_lejos_se_borra_al_tercer_scan():
    # LDS real: 0.0 = el rayo no choca con nada hasta 3.5 m
    assert pasar(nodo_falso(), [0.0, 0.0, 0.0]) == [True, True, False]


def test_inf_se_borra_al_tercer_scan():
    # Gazebo: inf = el rayo no choca con nada
    assert pasar(nodo_falso(), [math.inf] * 3) == [True, True, False]


def test_distancia_mayor_se_borra_al_tercer_scan():
    assert pasar(nodo_falso(), [2.0] * 3) == [True, True, False]


def test_lectura_normal_no_se_borra():
    # El rayo choca a la distancia del obstáculo: sigue ahí
    assert all(pasar(nodo_falso(), [1.0] * 6))


def test_cero_intermitente_reinicia_la_cuenta():
    # Ropa negra: 0.0 sueltos mezclados con lecturas normales. Solo se borra con 3 libres SEGUIDOS.
    assert pasar(nodo_falso(), [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]) == [True, True, True, True, True, False]


def test_cero_con_obstaculo_cerca_no_borra():
    # A 0.4 m un 0.0 puede ser "más cerca que range_min": no se borra
    assert all(pasar(nodo_falso(dist_obstaculo=0.4), [0.0] * 6))


def test_el_mismo_scan_cuenta_una_vez():
    # El timer (10 Hz) ve cada scan (5 Hz) dos veces: no debe contar doble
    n = nodo_falso()
    n.scan = scan(0.0)
    for _ in range(6):
        Fase1Node.actualizar_obstaculos(n, 0.0, 0.0, 0.0, 1.0)
    assert (0, 0) in n.obstaculos
    assert pasar(n, [0.0, 0.0]) == [True, False]


def test_olvido_por_tiempo():
    # Red de seguridad: pasado obstacle_memory_time sin verlo, se olvida aunque los rayos no lo borren
    assert pasar(nodo_falso(), [1.0], ahora=MEMORIA - 0.5) == [True]
    assert pasar(nodo_falso(), [1.0], ahora=MEMORIA + 0.5) == [False]


def test_lecturas_muy_cercanas_no_entran_en_memoria():
    # obstacle_min_range (0.20): un eco a 0.15 m (ventana sucia, propio robot) no se recuerda; a 0.30 m sí.
    # Holgura alta en todo el mapa: cualquier lectura más lejos que el filtro sería un obstáculo nuevo.
    n = nodo_falso()
    n.obstaculos = {}
    n.holgura = np.full((200, 200), 1.0)
    for distancia, debe_entrar in ((0.15, False), (0.30, True)):
        n.obstaculos = {}
        n.scan = scan(3.0)                      # el resto, a 3 m: más lejos que obstacle_max_range, no cuenta
        r = list(n.scan.ranges)
        r[90] = distancia                       # a la izquierda del robot
        n.scan.ranges = r
        Fase1Node.actualizar_obstaculos(n, 0.0, 0.0, 0.0, 1.0)
        assert bool(n.obstaculos) == debe_entrar, distancia


def test_rayos_libres_lecturas_sin_dato():
    # nan y lecturas por debajo de range_min (finitas pero cortas) no son "libre"
    assert not rayos_libres([math.nan, 0.0, 0.0], 1.0, 0.10, 0.5)
    assert not rayos_libres([0.05, 0.0, 0.0], 1.0, 0.10, 0.5)
    assert rayos_libres([0.0, math.inf, 1.2], 1.0, 0.10, 0.5)
    assert not rayos_libres([0.0, 0.0, 1.05], 1.0, 0.10, 0.5)   # 1.05 < d + margen: no está claro que se haya ido
