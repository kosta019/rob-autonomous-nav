# Tests sin robot de la fase III: el centro de la cuadrada con 3 o 4 patas, el punto delante de la estación, la medida de
# las patas parado (MEDIR3), la entrada (ENTRAR), la parada de emergencia que no para por las patas conocidas y la
# comprobación del aparcado (VERIFICAR3). Scans sintéticos con geometría conocida (patas = cilindros de 5 cm); no simula
# el robot.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_fase3.py -v
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav import pilares as pil  # noqa: E402
from autonomous_nav.fase1_node import Fase1Node  # noqa: E402

OFF = -0.032
C = (7.17, 16.51)                     # cruz de R (cinta)
PATAS = [(C[0] + dx, C[1] + dy) for dx in (-0.22, 0.22) for dy in (-0.22, 0.22)]


def test_centro_con_cuatro_y_con_tres_patas():
    assert pil.centro_cuadrada(PATAS) == pytest.approx((C[0], C[1], 4))
    # Una tapada: con las otras tres (triángulo rectángulo) el centro es el punto medio de la hipotenusa
    assert pil.centro_cuadrada(PATAS[:3]) == pytest.approx((C[0], C[1], 3))
    # Tres patas que no son de un cuadrado de 44 cm: nada
    assert pil.centro_cuadrada([(0.0, 0.0), (0.44, 0.0), (0.22, 0.38)]) is None


def scan(pose, patas=PATAS, fondo=2.5):
    # 360 rayos desde el LiDAR del robot en 'pose' contra las patas (cilindros de radio 2.5 cm); detrás, fondo a 2.5 m
    x, y, th = pose
    lx, ly = x + OFF * math.cos(th), y + OFF * math.sin(th)
    r = np.full(360, fondo)
    for i in range(360):
        u = (math.cos(th + math.radians(i)), math.sin(th + math.radians(i)))
        for px, py in patas:
            b = (px - lx) * u[0] + (py - ly) * u[1]
            c2 = (px - lx) ** 2 + (py - ly) ** 2 - b * b
            if b > 0 and c2 < 0.025 ** 2:
                r[i] = min(r[i], b - math.sqrt(0.025 ** 2 - c2))
    return SimpleNamespace(ranges=list(r), angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12,
                           range_max=3.5)


def nodo(estado='MEDIR3'):
    log = []
    n = SimpleNamespace(
        estado=estado, fase='III', scan=None, f3_scans=[], f3_ultimo_scan=None, f3_n=20, f3_intentos=0,
        f3_intentos_max=3, f3_max_desvio=0.25, f3_centro=None, f3_patas=[], f3_v=0.05, f3_tol=0.01, f3_dist=0.80,
        f3_radio_pata=0.07, f3_tol_aparcado=0.03, estacion_lado=0.44, lidar_offset_x=OFF, k_angular=1.5, k_lineal=0.5,
        max_w=0.4, angulo_girar=0.17, cono=math.radians(30), stop_distance=0.30, tol_obligatorio=0.02,
        estaciones=[{'tipo': 'cuadrada', 'x': C[0] + 0.05, 'y': C[1] - 0.04, 'ang': 0.0, 'vistas': 9}],
        estacion_confirmar=5, inflado=0.25, plan_final_dist=0.30, fase2_max_v=0.12,
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None),
        escribir_log=lambda x, y, th, ev, *a: log.append((ev, a)))
    for m in ('juntar_patas', 'estacion_cuadrada', 'es_pata'):
        setattr(n, m, (lambda f: lambda *a: f(n, *a))(getattr(Fase1Node, m)))
    return n, log


def test_delante_el_lado_mas_cercano_con_sitio():
    # Robot en P Base. El lado +Y (hacia la pared del extintor) no tiene sitio en el mapa; de los otros, el más cercano
    n, log = nodo('PAUSA')
    meta = {'resolucion': 0.05, 'origen_x': 0.0, 'origen_y': 10.0}
    n.holgura = np.full((200, 200), 1.0)
    n.holgura[:, int((17.0 - 10.0) / 0.05):] = 0.0          # Y > 17.0: pared
    n.meta = meta
    n.planificar_final = lambda *a: log.append(('PLAN', a))
    Fase1Node.empezar_fase3(n, 3.34, 15.70, 0.4)
    px, py = C[0] + 0.05 - 0.80, C[1] - 0.04                 # lado -X (hacia P Base)
    assert n.objetivos == [('final', pytest.approx(px), pytest.approx(py), 0.02)]
    assert n.pose_final[2] == pytest.approx(0.0, abs=1e-9)   # mirando a +X, al centro
    assert n.estado == 'GIRAR' and log[0][0] == 'FASE III INICIO' and log[1][0] == 'PLAN'


def test_sin_estacion_no_hace_nada():
    n, log = nodo('PAUSA')
    n.estaciones = []
    Fase1Node.empezar_fase3(n, 3.34, 15.70, 0.0)
    assert n.estado == 'FIN' and log == [('FASE_III_SIN_ESTACION', ())]


def medir(n, pose):
    for _ in range(20):
        n.scan = scan(pose)
        Fase1Node.medir3(n, *pose)


def test_medir3_da_el_centro_relativo_a_las_patas():
    # Delante del lado -X, mirando al centro. La estación de la fase II estaba 6 cm desviada: manda la medida de cerca
    pose = (C[0] - 0.80, C[1], 0.0)
    n, log = nodo()
    medir(n, pose)
    assert n.estado == 'ENTRAR'
    assert n.f3_centro == pytest.approx(C, abs=0.01)
    assert len(n.f3_patas) == 4
    assert log[-1][0] == 'FASE3_CENTRO patas=4'


def test_medir3_sin_patas_reintenta_y_abandona():
    pose = (C[0] - 0.80, C[1], 0.0)
    n, log = nodo()
    for intento in range(3):
        for _ in range(20):
            n.scan = scan(pose, patas=[])
            Fase1Node.medir3(n, *pose)
    assert n.estado == 'FIN' and n.f3_intentos == 3
    assert [ev for ev, _ in log][-1] == 'FASE_III_ABANDONADA'


def test_entrar_avanza_recto_y_para_en_el_centro():
    n, log = nodo('ENTRAR')
    n.f3_centro = C
    v, w = Fase1Node.entrar(n, C[0] - 0.80, C[1], 0.0)
    assert v == pytest.approx(0.05) and w == pytest.approx(0.0)
    # Entre las patas con el rumbo 10° mal: corrige avanzando, no gira sobre sí mismo
    v, w = Fase1Node.entrar(n, C[0] - 0.30, C[1], math.radians(10))
    assert v > 0.0 and w < 0.0
    assert Fase1Node.entrar(n, C[0] - 0.005, C[1], 0.0) == (0.0, 0.0)
    assert n.estado == 'VERIFICAR3' and log[-1][0] == 'FASE3_DENTRO'


def test_parada_de_emergencia_ignora_las_patas_pero_no_lo_demas():
    # Con el cono real (±30°, 0.30 m) y el robot centrado, las patas quedan fuera del cono (a ~41-46°): la exclusión
    # protege si entra desviado. Para probarla, un cono de ±60°, con las patas delanteras dentro (a ~0.30 m y ±46°)
    n, _ = nodo('ENTRAR')
    n.f3_patas = PATAS
    n.cono, n.stop_distance = math.radians(60), 0.35
    pose = (C[0] - 0.40, C[1], 0.0)
    n.scan = scan(pose)
    assert Fase1Node.obstaculo_delante(n, *pose) is None
    n.scan = scan(pose, patas=PATAS + [(C[0] - 0.15, C[1])])   # algo en medio (no es una pata): para
    assert Fase1Node.obstaculo_delante(n, *pose) is not None
    n.estado, n.scan = 'AVANZAR', scan(pose)                  # fuera de ENTRAR, las patas sí paran
    assert Fase1Node.obstaculo_delante(n, *pose) is not None


@pytest.mark.parametrize('desvio, fuera', [(0.02, False), (0.05, True)])
def test_verificar3_mide_el_aparcado(desvio, fuera):
    n, log = nodo('VERIFICAR3')
    pose = (C[0] - desvio, C[1], 0.0)
    for _ in range(20):
        n.scan = scan(pose)
        Fase1Node.verificar3(n, *pose)
    assert n.estado == 'FIN'
    eventos = [ev for ev, _ in log]
    assert eventos[0] == 'FASE III COMPLETADA patas=4'
    assert log[0][1][0] == pytest.approx(desvio, abs=0.01)
    assert ('APARCADO_FUERA_DE_TOLERANCIA' in eventos) == fuera


def test_de_la_fase2_a_la_3_y_de_orientar_a_medir():
    log = []
    reloj = SimpleNamespace(t=100.0)
    n = SimpleNamespace(estado='ORIENTAR', fase='II', pose_final=[3.34, 15.70, 0.2], tol_final_yaw=0.03, k_angular=1.5,
                        max_w=0.4, fase2_activa=True, fase3_activa=True, fase2_espera=3.0, t_pausa=None, f3_scans=[1],
                        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=reloj.t * 1e9)),
                        get_logger=lambda: SimpleNamespace(info=lambda *a: None),
                        escribir_log=lambda x, y, th, ev, *a: log.append(ev))
    n.empezar_fase3 = lambda x, y, th: log.append('empezar_fase3')
    Fase1Node.maquina_estados(n, 3.34, 15.70, 0.2)
    assert n.estado == 'PAUSA' and log == ['FASE II COMPLETADA']
    reloj.t += 3.5
    Fase1Node.maquina_estados(n, 3.34, 15.70, 0.2)
    assert log[-1] == 'empezar_fase3'
    # Fase III: al orientarse delante de la estación no se acaba la fase: se miden las patas
    n.fase, n.estado = 'III', 'ORIENTAR'
    Fase1Node.maquina_estados(n, 3.34, 15.70, 0.2)
    assert n.estado == 'MEDIR3' and log[-1] == 'FASE3_DELANTE' and n.f3_scans == []
