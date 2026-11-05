# Test sin robot del aviso de ventana sucia de comprobar_pose (fraccion_con_ecos).
# Los scans son como los que guarda comprobar_pose: arrays con nan donde no hay dato.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_comprobar_pose.py -v
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.comprobar_pose import ECOS_MAX, fraccion_con_ecos  # noqa: E402

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
