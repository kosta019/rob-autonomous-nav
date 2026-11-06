# Tests del detector de pilares (autonomous_nav/pilares.py), sin ROS.
# Con scans REALES del 06/11 (test/datos/scans_estaciones.npz, sacados con scripts/extraer_scans_test.py):
#   R_parado: a 0.55 m de la estación R, mirándola (bag 17:10). R medida con cinta: (7.17, 16.51), paralela a las paredes.
#   T_rodeo: pasando junto a la estación triangular T (bag 16:13).
#   pasillo_Q: los primeros 60 s de la fase II desde Q, sin estaciones a la vista (bag 16:31).
# Uso:  python3 -m pytest autonomous_nav/test/test_pilares.py -v
import math
import os
import sys

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from autonomous_nav import pilares as pil  # noqa: E402

DATOS = np.load(os.path.join(RAIZ, 'test', 'datos', 'scans_estaciones.npz'))
# Los scans del 06/11: 360 rayos de 1° desde 0, range_min 0.12, range_max 3.5
A_MIN, A_INC, R_MIN, R_MAX = 0.0, math.radians(1.0), 0.12, 3.5


def detectar(r):
    return pil.pilares_scan(r, A_MIN, A_INC, R_MIN, R_MAX)


def estaciones(tramo, min_det=3):
    # Recorre un tramo como lo haría el nodo y devuelve la estación encontrada tras cada scan (o None)
    grupos, salida = [], []
    for r, (t, x, y, th) in zip(DATOS[tramo + '_scans'], DATOS[tramo + '_poses']):
        pil.agrupar(grupos, pil.a_mundo(detectar(r), (x, y, th)), t)
        pil.olvidar(grupos, t)
        salida.append(pil.buscar_estacion([(g[0], g[1]) for g in grupos if g[2] >= min_det]))
    return salida


def scan_sintetico(fondo=1.6):
    # 360 rayos a una pared lejana
    return np.full(360, fondo)


def test_cuatro_patas_en_un_scan_real():
    # Las 4 patas de R en un solo scan (vistas en bruto: 0.70 m a 8°, 0.88 a 37°, 1.14 a 4°, 1.23 a 24°).
    # El prototipo sacaba 1-2 (en este scan, 2): junto a 3 de ellas hay un rayo a 0.0 (sin dato) que tomaba por
    # "algo más cerca".
    # Con una pata fina el LDS es ruidoso (la de 1.2 m da de 1.03 a 1.32 según el scan) y en cada scan salen de 2 a 4:
    # por eso se promedian varios scans en el mundo (agrupar).
    esperadas = [(0.70, 8), (0.88, 37), (1.14, 4), (1.23, 24)]
    vistos = detectar(DATOS['R_parado_scans'][22])
    assert len(vistos) == 4
    for d, ang in esperadas:
        ex, ey = (d + 0.025) * math.cos(math.radians(ang)), (d + 0.025) * math.sin(math.radians(ang))
        assert min(math.hypot(x - ex, y - ey) for x, y, _ in vistos) < 0.10
    # En todos los scans parado, lo que sale a menos de 1.5 m está en la dirección de una de las 4 patas
    for r in DATOS['R_parado_scans']:
        for x, y, _ in detectar(r):
            if math.hypot(x, y) < 1.5:
                assert min(abs(math.degrees(math.atan2(y, x)) - ang) for _, ang in esperadas) <= 3


def test_rayo_sin_dato_junto_a_la_pata():
    r = scan_sintetico()
    r[10:13] = 0.70
    r[13] = 0.0     # el rayo que roza el borde
    r[9] = np.inf
    vistos = detectar(r)
    assert len(vistos) == 1
    assert vistos[0][2] == 3


def test_trozo_de_pared_entre_huecos_no_es_pata():
    # Pared a 1.0 m con huecos sin dato: los dos rayos sueltos tienen la pared al lado, a la misma distancia
    r = scan_sintetico(1.0)
    r[20] = r[23] = 0.0
    assert detectar(r) == []


def test_pata_pegada_a_la_pared_no_se_ve():
    # Si lo de detrás está a menos de `salto` (10 cm), no se distingue de la pared: no se inventa una pata
    r = scan_sintetico(1.0)
    r[10:13] = 0.95
    assert detectar(r) == []


def test_estacion_R_real():
    vistas = [e for e in estaciones('R_parado') if e is not None]
    assert len(vistas) > 20
    tipo, cx, cy, ang, patas = vistas[-1]
    assert tipo == 'cuadrada'
    assert math.hypot(cx - 7.17, cy - 16.51) < 0.03      # cinta
    assert abs(math.degrees(ang)) < 3                   # paralela a las paredes
    assert len(patas) == 4


def test_estacion_T_real_es_triangular():
    vistas = [e for e in estaciones('T_rodeo') if e is not None]
    assert len(vistas) > 10
    assert all(e[0] == 'triangular' for e in vistas)


def test_pasillo_sin_estaciones():
    # Esquinas de las paredes junto a Q salen como "pilares", pero no forman ninguna estación
    assert all(e is None for e in estaciones('pasillo_Q'))


def cuadrado(cx, cy, ang, lado=0.44):
    c, s = math.cos(ang), math.sin(ang)
    return [(cx + (u * c - v * s) * lado / 2, cy + (u * s + v * c) * lado / 2) for u, v in ((1, 1), (-1, 1), (-1, -1),
                                                                                         (1, -1))]


def test_cuadrado_girado_entre_otros_pilares():
    patas = cuadrado(3.0, 15.0, math.radians(30))
    falsos = [(3.5, 15.6), (2.4, 14.9), (3.1, 14.2), (8.37, 14.04)]
    tipo, cx, cy, ang, _ = pil.buscar_estacion(falsos[:2] + patas + falsos[2:])
    assert tipo == 'cuadrada'
    assert math.hypot(cx - 3.0, cy - 15.0) < 1e-6
    assert abs(math.degrees(ang) - 30) < 1e-6


def test_angulo_del_cuadrado_entre_menos_45_y_45():
    for grados in (-44, -2, 0, 2, 44, 88, 92):
        ang = pil.buscar_estacion(cuadrado(0, 0, math.radians(grados)))[3]
        esperado = (grados + 45) % 90 - 45
        assert abs(math.degrees(ang) - esperado) < 1e-6


def test_tres_patas_de_un_cuadrado_no_son_triangulo():
    # Con una pata tapada, el cuadrado da lados L, L y L·√2: no es la estación triangular
    assert pil.buscar_estacion(cuadrado(0, 0, 0.3)[:3]) is None


def test_triangulo_equilatero():
    L = 0.44
    patas = [(1.0, 1.0), (1.0 + L, 1.0), (1.0 + L / 2, 1.0 + L * math.sqrt(3) / 2)]
    tipo, cx, cy, _, _ = pil.buscar_estacion(patas)
    assert tipo == 'triangular'
    assert math.hypot(cx - (1.0 + L / 2), cy - (1.0 + L * math.sqrt(3) / 6)) < 1e-6


def test_agrupar_y_olvidar():
    grupos = []
    pil.agrupar(grupos, [(1.0, 1.0), (2.0, 2.0)], t=0.0)
    pil.agrupar(grupos, [(1.02, 1.0)], t=1.0)
    assert len(grupos) == 2 and grupos[0][2] == 2
    pil.olvidar(grupos, t=3.5, olvido=3.0)     # (2, 2) no se ve desde t = 0
    assert len(grupos) == 1 and abs(grupos[0][0] - 1.01) < 1e-9


def test_a_mundo():
    # Robot en (1, 2) mirando a +Y; el LiDAR 3.2 cm detrás: un punto 1 m delante del LiDAR queda en (1, 2.968)
    (x, y), = pil.a_mundo([(1.0, 0.0, 3)], (1.0, 2.0, math.pi / 2))
    assert abs(x - 1.0) < 1e-9 and abs(y - 2.968) < 1e-9
