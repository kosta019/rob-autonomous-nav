# Tests de la fase II en el mismo lanzamiento: al acabar la fase I (ORIENTAR) se para phase2_start_delay s (PAUSA) y
# empieza la II con la ruta de cobertura y P Base; current_phase pasa a "II". Sin phase2_enabled, acaba en FIN.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_fase2.py -v
import os
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import Fase1Node  # noqa: E402

Q = (8.97, 14.497, 0.0)


def nodo_falso(activa=True):
    log = []
    reloj = SimpleNamespace(t=100.0)
    n = SimpleNamespace(
        estado='ORIENTAR', fase='I', pose_final=[8.97, 14.497, 0.0], tol_final_yaw=0.05, k_angular=1.0, max_w=1.0,
        fase2_activa=activa, fase2_espera=3.0, t_pausa=None, fase2_nombres=['P1_e', 'P1_o'], fase2_x=[8.8, 0.1],
        fase2_y=[14.35, 14.35], fase2_final=[3.34, 15.70, 0.0], fase2_max_v=0.12, max_v=0.20, tol_waypoint=0.10,
        tol_obligatorio=0.05, objetivos=[('final', 8.97, 14.497, 0.05)], i_objetivo=1, anclas=['E', 'Door'],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=reloj.t * 1e9)),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None),
        escribir_log=lambda x, y, th, ev, *a: log.append((n.fase, ev)),
        obligatorios=['E', 'Door', 'Q'], plan_final_dist=0.30,
        planificar_final=lambda x, y, th, nombre: log.append((n.fase, 'PLAN_' + nombre)))
    n.empezar_fase2 = lambda x, y, th: Fase1Node.empezar_fase2(n, x, y, th)
    return n, log, reloj


def test_sin_fase2_acaba_en_fin():
    n, log, _ = nodo_falso(activa=False)
    assert Fase1Node.maquina_estados(n, *Q) == (0.0, 0.0)
    assert n.estado == 'FIN' and log == [('I', 'FASE I COMPLETADA')]


def test_fase1_pausa_y_fase2():
    n, log, reloj = nodo_falso()
    Fase1Node.maquina_estados(n, *Q)
    assert n.estado == 'PAUSA' and log == [('I', 'FASE I COMPLETADA')]
    reloj.t += 2.0                          # antes de phase2_start_delay: sigue parado
    assert Fase1Node.maquina_estados(n, *Q) == (0.0, 0.0)
    assert n.estado == 'PAUSA'
    reloj.t += 1.5
    Fase1Node.maquina_estados(n, *Q)
    assert n.fase == 'II' and n.estado == 'GIRAR'
    # Desde Q, P1_e está a 0.23 m: no se planifica (las pasadas van rectas, como se diseñaron)
    assert log[1:] == [('II', 'FASE II INICIO')]
    assert [o[0] for o in n.objetivos] == ['P1_e', 'P1_o', 'final'] and n.i_objetivo == 0
    assert n.objetivos[-1][1:3] == (3.34, 15.70)
    assert n.pose_final == [3.34, 15.70, 0.0]
    assert n.anclas == ['P1_e', 'P1_o']     # un replan no se salta una pasada
    assert n.max_v == 0.12
    assert n.obligatorios == ['P1_e']


def test_fase2_lejos_del_primer_punto_planifica():
    # Pose final de la fase I en el vestíbulo (no en Q): hasta P1_e se planifica, que en línea recta cruzaría paredes
    n, log, reloj = nodo_falso()
    n.estado, n.fase, n.t_pausa = 'PAUSA', 'I', 0.0
    Fase1Node.maquina_estados(n, 7.5, 12.6, 0.0)
    assert log == [('II', 'FASE II INICIO'), ('II', 'PLAN_FASE2')]


def test_fase2_completada_acaba_en_fin():
    n, log, _ = nodo_falso()
    n.fase = 'II'
    n.pose_final = [3.34, 15.70, 0.0]
    Fase1Node.maquina_estados(n, 3.34, 15.70, 0.0)
    assert n.estado == 'FIN' and log == [('II', 'FASE II COMPLETADA')]
