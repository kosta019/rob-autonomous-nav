# Tests de la fase II en el mismo lanzamiento: al acabar la fase I (ORIENTAR) se para phase2_start_delay s (PAUSA) y
# empieza la II con la ruta de cobertura y P Base; current_phase pasa a "II". Sin phase2_enabled, acaba en FIN.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_fase2.py -v
import math
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
        fase2_activa=activa, fase3_activa=False, fase2_espera=3.0, t_pausa=None, fase2_nombres=['P1_e', 'P1_o'], fase2_x=[8.8, 0.1],
        fase2_y=[14.35, 14.35], fase2_final=[3.34, 15.70, 0.0], fase2_max_v=0.12, max_v=0.20, tol_waypoint=0.10,
        tol_obligatorio=0.05, objetivos=[('final', 8.97, 14.497, 0.05)], i_objetivo=1, anclas=['E', 'Door'],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=reloj.t * 1e9)),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None),
        escribir_log=lambda x, y, th, ev, *a: log.append((n.fase, ev)),
        obligatorios=['E', 'Door', 'Q'], plan_final_dist=0.30,
        planificar_final=lambda x, y, th, nombre='FINAL': log.append((n.fase, 'PLAN_' + nombre)))
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


# ---------- Fin de la cobertura (07/11): segunda cobertura si no hay estación cuadrada, y θ en P Base hacia ella ----------

def nodo_cobertura(estaciones):
    # Fase II, yendo al último punto de la cobertura (P2_e); después, P Base
    n, log, reloj = nodo_falso()
    n.fase, n.estado, n.cobertura = 'II', 'AVANZAR', 1
    n.objetivos = [('P2_e', 8.80, 15.55, 0.10), ('final', 3.34, 15.70, 0.05)]
    n.i_objetivo, n.pose_final, n.anclas, n.obligatorios = 0, [3.34, 15.70, 0.0], ['P2_e'], ['P1_e']
    n.sustitutos, n.reloc_q_en, n.reloc_antes_de = {}, '', 'Door'
    n.reloc_p_hechas, n.reloc_p_veces = 0, 0
    n.v_paso = 0.0
    n.estaciones, n.estacion_confirmar = estaciones, 5
    n.fase2b_activa, n.fase2b_nombres = True, ['S1_e', 'S1_o']
    n.fase2b_x, n.fase2b_y = [8.80, 0.10], [14.65, 14.65]
    n.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n.estacion_cuadrada = lambda: Fase1Node.estacion_cuadrada(n)
    n.fin_de_cobertura = lambda x, y, th: Fase1Node.fin_de_cobertura(n, x, y, th)
    return n, log


def test_sin_estacion_segunda_cobertura_y_despues_no_encontrada():
    n, log = nodo_cobertura([{'tipo': 'triangular', 'x': 2.1, 'y': 15.3, 'vistas': 9}])
    Fase1Node.maquina_estados(n, 8.80, 15.55, 0.0)          # llega a P2_e
    assert [o[0] for o in n.objetivos[n.i_objetivo:]] == ['S1_e', 'S1_o', 'final']
    # S1_e está a 0.9 m de P2_e: se planifica hasta él
    assert log[-2:] == [('II', 'SEGUNDA_COBERTURA'), ('II', 'PLAN_COBERTURA2')]
    assert n.anclas == ['P2_e', 'S1_e', 'S1_o'] and n.obligatorios == ['S1_e']
    n.i_objetivo = 2                                        # [P2_e, S1_e, S1_o, final]: yendo a S1_o
    n.estado = 'AVANZAR'
    Fase1Node.maquina_estados(n, 0.10, 14.65, 3.14)         # llega a S1_o: tampoco hay estación cuadrada
    assert log[-2:] == [('II', 'ESTACION_NO_ENCONTRADA'), ('II', 'PLAN_FINAL')]   # P Base, lejos: se planifica
    assert n.objetivos[n.i_objetivo][0] == 'final' and n.pose_final[2] == 0.0


def test_con_estacion_va_a_p_base_y_mira_hacia_ella():
    n, log = nodo_cobertura([{'tipo': 'cuadrada', 'x': 7.17, 'y': 16.51, 'vistas': 7}])
    Fase1Node.maquina_estados(n, 8.80, 15.55, 0.0)          # llega a P2_e: sin segunda cobertura
    assert [o[0] for o in n.objetivos[n.i_objetivo:]] == ['final']
    assert all(ev != 'SEGUNDA_COBERTURA' for _, ev in log)
    n.estado = 'AVANZAR'
    Fase1Node.maquina_estados(n, 3.34, 15.70, math.pi)      # llega a P Base
    assert n.estado == 'ORIENTAR'
    assert n.pose_final[2] == pytest.approx(math.atan2(16.51 - 15.70, 7.17 - 3.34))
    assert log[-1][1].startswith('THETA_HACIA_ESTACION +11.9')     # atan2(0.81, 3.83)


def test_en_p_base_se_relocaliza_mirando_a_x_y_luego_acaba():
    # Con reloc_pbase_times = 1: al llegar gira a +X, se relocaliza parado y, a la vuelta, ya sigue como siempre
    n, log = nodo_cobertura([{'tipo': 'cuadrada', 'x': 7.17, 'y': 16.51, 'vistas': 7}])
    n.reloc_p_veces, n.reloc_n = 1, 10
    n.reloc_tipo, n.reloc_scans_lista = '', []
    n.empezar_reloc = lambda tipo: Fase1Node.empezar_reloc(n, tipo)
    n.objetivos, n.i_objetivo, n.estado = [('final', 3.34, 15.70, 0.05)], 0, 'AVANZAR'
    Fase1Node.maquina_estados(n, 3.34, 15.70, math.pi)      # llega a P Base
    assert (n.estado, n.reloc_p_hechas, n.i_objetivo) == ('ORIENTAR_P', 1, 0)
    assert log[-1][1] == 'LLEGADA_RELOC_P 1'
    _, w = Fase1Node.maquina_estados(n, 3.34, 15.70, 1.0)   # todavía girado: gira hacia rumbo 0
    assert w < 0.0 and n.estado == 'ORIENTAR_P'
    Fase1Node.maquina_estados(n, 3.34, 15.70, 0.0)          # ya mira a +X: relocaliza
    assert (n.estado, n.reloc_tipo) == ('RELOC', 'P')
    n.estado = 'AVANZAR'                                    # tras la relocalización vuelve a P Base
    Fase1Node.maquina_estados(n, 3.34, 15.70, 0.0)
    assert n.estado == 'ORIENTAR' and n.i_objetivo == 1


def test_estacion_sin_confirmar_no_cuenta():
    # Vista en menos de station_confirm_scans scans: como si no estuviera
    n, log = nodo_cobertura([{'tipo': 'cuadrada', 'x': 7.17, 'y': 16.51, 'vistas': 2}])
    Fase1Node.maquina_estados(n, 8.80, 15.55, 0.0)
    assert ('II', 'SEGUNDA_COBERTURA') in log
