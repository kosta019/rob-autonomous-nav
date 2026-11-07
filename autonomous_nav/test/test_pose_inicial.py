# Tests sin robot de la pose de salida con el LiDAR (estado POSE_INICIAL): revisar_pose_inicial decide qué se aplica de
# lo que estima comprobar_pose, y pose_inicial_lidar lo hace con el robot parado antes de moverse. Geometría conocida (una
# sala cuadrada); no simula el robot.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_pose_inicial.py -v
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav import comprobar_pose as cp  # noqa: E402
from autonomous_nav.fase1_node import Fase1Node, revisar_pose_inicial  # noqa: E402

LIM = dict(min_puntos=30, max_rms=0.02, max_corr=0.15, max_ang=math.radians(10.0))
DADA = (1.0, 1.5, 0.0)


def cara(eje, n=60, rms=0.005):
    return {'eje': eje, 'n': n, 'rms': rms}


def test_con_paredes_de_los_dos_ejes_aplica_todo():
    x, y, th, motivo = revisar_pose_inicial(DADA, (1.03, 1.48, 0.05), [cara('X'), cara('Y')], 'Y', **LIM)
    assert motivo == '' and (x, y, th) == (1.03, 1.48, 0.05)


def test_sin_paredes_x_la_x_es_la_dada():
    # Salida B o C: los paneles tapan la pared del fondo. Rumbo e Y de las paredes; X la de la cruz
    x, y, th, motivo = revisar_pose_inicial(DADA, (1.08, 1.48, 0.05), [cara('Y', n=90)], 'Y', **LIM)
    assert motivo == '' and (x, y, th) == (1.0, 1.48, 0.05)


def test_sin_paredes_del_eje_del_rumbo_no_vale():
    assert revisar_pose_inicial(DADA, (1.0, 1.5, 0.05), [cara('X', n=90)], 'Y', **LIM)[3] == 'sin_paredes_rumbo'
    assert revisar_pose_inicial(DADA, (1.0, 1.5, 0.05), [], '', **LIM)[3] == 'sin_paredes_rumbo'


def test_pocos_puntos_residuo_o_corrección_grande_no_vale():
    assert revisar_pose_inicial(DADA, (1.0, 1.5, 0.05), [cara('Y', n=20)], 'Y', **LIM)[3] == 'pocos_puntos'
    assert revisar_pose_inicial(DADA, (1.0, 1.5, 0.05), [cara('Y', rms=0.022)], 'Y', **LIM)[3] == 'residuo'
    assert revisar_pose_inicial(DADA, (1.0, 1.5, math.radians(12)), [cara('Y')], 'Y', **LIM)[3] == 'grande'
    x, y, th, motivo = revisar_pose_inicial(DADA, (1.0, 1.7, 0.0), [cara('Y')], 'Y', **LIM)
    assert motivo == 'grande' and (x, y, th) == DADA      # rechazada: la pose dada sin tocar


# Sala de 4 x 4 m: caras x = 0, x = 4 (eje 0) e y = 0, y = 4 (eje 1), como las da comprobar_pose.cargar_caras
CARAS = {'eje': np.array([0, 0, 1, 1]), 'c': np.array([0.0, 4.0, 0.0, 4.0]),
         'lo': np.zeros(4), 'hi': np.full(4, 4.0)}
OFF = -0.032


def scan_real(pose):
    # Lo que mediría el LiDAR desde la pose real: 360 rayos trazados contra la sala
    x, y, th = pose
    a = np.radians(np.arange(360))
    r, _ = cp.trazar((x + OFF * math.cos(th), y + OFF * math.sin(th)), th + a, CARAS)
    return SimpleNamespace(ranges=list(r), angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12,
                           range_max=3.5)


def nodo_falso(real, yaws):
    log, fijada = [], []
    n = SimpleNamespace(
        scan=None, pi_ultimo_scan=None, pi_scans=[], pi_n=25, pi_wz=[0.0] * 50, pi_yaws=list(yaws), caras=CARAS,
        pi_eje_rumbo='Y', pi_min_puntos=30, pi_max_rms=0.02, pi_max_corr=0.15, pi_max_ang=math.radians(10.0),
        lidar_offset_x=OFF, lidar_yaw_offset=0.0, alcance_min_obstaculos=0.20, obstaculos={(1, 1): None},
        estado='POSE_INICIAL', estado_tras_pose='GIRAR',
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None),
        escribir_log=lambda x, y, th, ev, *a: log.append(ev), fijar_pose=lambda *p: fijada.append(p))
    return n, log, fijada


def pasar_scans(n, real):
    for _ in range(25):
        n.scan = scan_real(real)       # un mensaje nuevo cada vez, como el LiDAR
        Fase1Node.pose_inicial_lidar(n, *DADA)


def test_corrige_la_pose_de_salida_y_sigue():
    # El robot está 3° girado y 4 cm desplazado respecto a initial_pose: el nodo lo corrige antes de moverse
    real = (1.04, 1.47, math.radians(3.0))
    n, log, fijada = nodo_falso(real, [0.5] * 50)
    pasar_scans(n, real)
    assert len(fijada) == 1
    x, y, th = fijada[0]
    assert x == pytest.approx(real[0], abs=0.01) and y == pytest.approx(real[1], abs=0.01)
    assert th == pytest.approx(real[2], abs=math.radians(0.2))
    assert n.estado == 'GIRAR' and n.obstaculos == {}
    assert log[-1].startswith('POSE_INICIAL dx=+0.04')


def test_giroscopo_que_deriva_no_se_mueve():
    # Con el robot quieto, el rumbo de /odom cambia 1°: la OpenCR se calibró moviéndose (05/11). No sale
    real = (1.04, 1.47, math.radians(3.0))
    n, log, fijada = nodo_falso(real, np.linspace(0.0, math.radians(1.0), 50))
    pasar_scans(n, real)
    assert fijada == [] and n.estado == 'POSE_INICIAL'
    assert log[-1].startswith('POSE_INICIAL_RECHAZADA giroscopo')
    assert n.pi_scans == []            # vuelve a empezar con scans nuevos
