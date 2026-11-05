# Test sin robot de punto_sustituto (fase1_node): el punto libre más cercano a un obligatorio tapado.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_tapado.py -v
import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav.fase1_node import punto_sustituto  # noqa: E402

META = {'resolucion': 0.05, 'origen_x': 0.0, 'origen_y': 0.0}
E = (1.025, 1.025)                      # centro de una celda


def holgura_con(obstaculo, margen=0.10):
    # Rejilla de 2.5 x 2.5 m libre (holgura 1 m) con un obstáculo puntual inflado 'margen'
    n = 50
    c = (np.arange(n) + 0.5) * META['resolucion']
    d = np.hypot(c[:, None] - obstaculo[0], c[None, :] - obstaculo[1]) - margen
    return np.minimum(1.0, np.maximum(d, 0.0))


def test_obstaculo_a_029_m_da_un_punto_libre_cerca():
    # Como E en la ruta 4: algo a 0.29 m. El sustituto está libre (holgura > 0.25) y a menos de 0.40 m de E,
    # del lado contrario al obstáculo.
    obst = (E[0] + 0.29, E[1])
    h = holgura_con(obst)
    s = punto_sustituto(h, META, E[0], E[1], 0.25, 0.40)
    assert s is not None
    assert math.dist(s, E) <= 0.40
    assert math.dist(s, obst) - 0.10 > 0.25
    assert s[0] < E[0]


def test_si_el_punto_esta_libre_devuelve_el_mismo():
    h = holgura_con((2.2, 2.2))
    s = punto_sustituto(h, META, E[0], E[1], 0.25, 0.40)
    assert math.dist(s, E) < 0.01


def test_sin_sitio_libre_a_menos_de_040():
    h = holgura_con(E, margen=0.30)     # el obstáculo encima del punto: nada libre en 0.40 m con inflado 0.25
    assert punto_sustituto(h, META, E[0], E[1], 0.25, 0.40) is None
