# Detector de pilares (patas de las estaciones de carga) con el LiDAR, para la fase II.
# Solo numpy: lo usa fase1_node y se puede probar sin ROS (test/test_pilares.py).
#
# 1) pilares_scan: en un scan, una pata (cilindro de 5 cm) es un segmento de pocos rayos seguidos, estrecho, con lo que
#    hay a los lados MÁS LEJOS (está delante del fondo). Segmentación del scan por saltos de distancia (teoría: extracción
#    de características del láser).
# 2) agrupar: las detecciones de muchos scans, ya en el mundo, se juntan en grupos (un grupo = un pilar).
# 3) buscar_estacion: entre los pilares, 4 en cuadrado de lado L (la estación buena) o 3 en triángulo equilátero (la
#    que hay que descartar).
import itertools
import math

import numpy as np

RADIO_PATA = 0.025   # pata de 5 cm de diámetro


def pilares_scan(r, angle_min, angle_inc, range_min, range_max, alcance=1.6, salto=0.10, ancho_max=0.09,
                 max_rayos=8, hueco=2):
    # Devuelve los centros de las patas vistas en un scan, en el frame del LiDAR: lista de (x, y, n_rayos).
    # alcance: más lejos, una pata da 0-1 rayos y no se distingue (a 1.2 m da 2-3).
    # hueco: rayos sin dato que se saltan para buscar el vecino de cada lado (junto al borde de una pata suele salir
    # un 0.0: el rayo roza el cilindro y no vuelve).
    r = np.array(r, dtype=float)
    n = len(r)
    sin_dato = ~np.isfinite(r) | (r < range_min) | (r > range_max)
    r[sin_dato] = np.nan
    a = angle_min + np.arange(n) * angle_inc

    # Segmentos: rayos con dato seguidos, sin saltos de más de `salto`
    segmentos = []
    actual = []
    for i in range(n):
        if np.isnan(r[i]):
            if actual:
                segmentos.append(actual)
            actual = []
        elif actual and abs(r[i] - r[actual[-1]]) >= salto:
            segmentos.append(actual)
            actual = [i]
        else:
            actual.append(i)
    if actual:
        segmentos.append(actual)

    salida = []
    for s in segmentos:
        rm = float(np.mean(r[s]))
        if len(s) > max_rayos or rm > alcance:
            continue
        x = r[s] * np.cos(a[s])
        y = r[s] * np.sin(a[s])
        if math.hypot(x[-1] - x[0], y[-1] - y[0]) > ancho_max:
            continue
        # Vecino de cada lado: el primer rayo con dato, saltando como mucho `hueco` sin dato
        fondo_izq = vecino(r, s[0], -1, hueco)
        fondo_der = vecino(r, s[-1], +1, hueco)
        # Si un vecino está a la misma distancia (o más cerca), el segmento es un trozo de pared o de otra cosa
        if fondo_izq is not None and fondo_izq < rm + salto:
            continue
        if fondo_der is not None and fondo_der < rm + salto:
            continue
        # Al menos un lado con fondo visible: un trozo de pared entre dos huecos sin dato no cuenta
        if fondo_izq is None and fondo_der is None:
            continue
        # Centro de la pata: el punto medio visto, un radio más lejos en la misma dirección
        cx, cy = float(np.mean(x)), float(np.mean(y))
        d = math.hypot(cx, cy)
        salida.append((cx * (d + RADIO_PATA) / d, cy * (d + RADIO_PATA) / d, len(s)))
    return salida


def vecino(r, i, paso, hueco):
    # Distancia del primer rayo con dato a partir de i (sin contarlo), hacia `paso` (-1 o +1). None si no hay.
    n = len(r)
    for k in range(1, hueco + 2):
        v = r[(i + paso * k) % n]
        if not np.isnan(v):
            return float(v)
    return None


def a_mundo(puntos, pose, lidar_x=-0.032):
    # Del frame del LiDAR al mundo. pose = (x, y, yaw) del centro del robot; el LiDAR está lidar_x por delante (negativo:
    # detrás) en el eje del robot, sin girar.
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    lx, ly = x + lidar_x * c, y + lidar_x * s
    return [(lx + px * c - py * s, ly + px * s + py * c) for px, py, *_ in puntos]


def agrupar(grupos, puntos, t, radio=0.08, memoria=20):
    # Junta cada punto (en el mundo) con el grupo más cercano a menos de `radio`, o crea uno nuevo.
    # Cada grupo es una lista [x, y, n, t_ultimo]. La media pesa las últimas `memoria` detecciones: si la pose deriva,
    # el grupo la sigue en vez de quedarse en un borrón.
    for px, py in puntos:
        mejor, dmin = None, radio
        for g in grupos:
            d = math.hypot(g[0] - px, g[1] - py)
            if d < dmin:
                mejor, dmin = g, d
        if mejor is None:
            grupos.append([px, py, 1, t])
        else:
            mejor[2] += 1
            mejor[3] = t
            k = 1.0 / min(mejor[2], memoria)
            mejor[0] += k * (px - mejor[0])
            mejor[1] += k * (py - mejor[1])
    return grupos


def olvidar(grupos, t, olvido=3.0):
    # Quita los grupos que no se ven desde hace más de `olvido` s. Con la pose derivando, juntar mucho rato mezcla las
    # patas; la estación se busca con lo visto hace poco.
    grupos[:] = [g for g in grupos if t - g[3] <= olvido]
    return grupos


def buscar_estacion(pilares, lado=0.44, tol=0.05):
    # pilares: lista de (x, y). Busca 4 en cuadrado (lados `lado`, diagonales lado·√2) y, si no, 3 en triángulo
    # equilátero de lado `lado`. Devuelve (tipo, centro_x, centro_y, angulo, patas) o None.
    # tipo: 'cuadrada' (la estación buena) o 'triangular' (la que hay que descartar). Tres patas de un cuadrado
    # (lados L, L, L·√2) no son un triángulo equilátero: no se confunden.
    # angulo: cuadrada, el de sus lados en [-45°, 45°) (girado 90° es el mismo cuadrado); triangular, el de una punta
    # vista desde el centro, en [-60°, 60°).
    diag = lado * math.sqrt(2)
    mejor = None
    for n_patas in (4, 3):
        for i, p0 in enumerate(pilares):
            # Solo combinaciones de pilares cercanos entre sí (con muchos grupos, probarlas todas sería lento)
            cerca = [p for p in pilares[i + 1:] if math.dist(p0, p) < diag + tol]
            for resto in itertools.combinations(cerca, n_patas - 1):
                patas = (p0,) + resto
                d = sorted(math.dist(p, q) for p, q in itertools.combinations(patas, 2))
                if n_patas == 4:
                    err = max([abs(v - lado) for v in d[:4]] + [abs(v - diag) for v in d[4:]])
                else:
                    err = max(abs(v - lado) for v in d)
                if err < tol and (mejor is None or err < mejor[0]):
                    mejor = (err, patas)
        if mejor is not None:
            break
    if mejor is None:
        return None
    patas = mejor[1]
    cx = sum(p[0] for p in patas) / len(patas)
    cy = sum(p[1] for p in patas) / len(patas)
    p0 = patas[0]
    if len(patas) == 4:
        # Un lado: de la primera pata a la más cercana a ella (la diagonal es más larga)
        q = min(patas[1:], key=lambda p: math.dist(p0, p))
        a = math.atan2(q[1] - p0[1], q[0] - p0[0])
        return 'cuadrada', cx, cy, (a + math.pi / 4) % (math.pi / 2) - math.pi / 4, list(patas)
    a = math.atan2(p0[1] - cy, p0[0] - cx)
    return 'triangular', cx, cy, (a + math.pi / 3) % (2 * math.pi / 3) - math.pi / 3, list(patas)
