# Detector de pilares (patas de las estaciones de carga) con el LiDAR. Solo usa numpy, así que se puede probar sin ROS.
# Primero busca patas en un scan (segmentos estrechos de pocos rayos con fondo más lejos a los lados), luego junta las
# detecciones de varios scans en grupos (un grupo es un pilar) y al final busca 4 pilares en cuadrado o 3 en triángulo.
import itertools
import math

import numpy as np

RADIO_PATA = 0.025   # pata de 5 cm de diámetro


def pilares_scan(r, angle_min, angle_inc, range_min, range_max, alcance=1.6, salto=0.10, ancho_max=0.09,
                 max_rayos=8, hueco=2):
    # Devuelve los centros de las patas vistas en un scan, en el frame del LiDAR: lista de (x, y, n_rayos).
    # alcance: más lejos, una pata da 0-1 rayos y no se distingue.
    # hueco: rayos sin dato que se saltan al buscar el vecino de cada lado (al borde de una pata suele salir un 0.0).
    r = np.array(r, dtype=float)
    n = len(r)
    sin_dato = ~np.isfinite(r) | (r < range_min) | (r > range_max)
    r[sin_dato] = np.nan
    a = angle_min + np.arange(n) * angle_inc

    # Segmentos de rayos con dato seguidos, sin saltos de más de `salto`
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
    # Distancia del primer rayo con dato a partir de i (sin contarlo), hacia `paso` (-1 o +1), o None si no hay.
    n = len(r)
    for k in range(1, hueco + 2):
        v = r[(i + paso * k) % n]
        if not np.isnan(v):
            return float(v)
    return None


def a_mundo(puntos, pose, lidar_x=-0.032):
    # Pasa puntos del frame del LiDAR al mundo. pose = (x, y, yaw) del robot; el LiDAR está lidar_x por delante.
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    lx, ly = x + lidar_x * c, y + lidar_x * s
    return [(lx + px * c - py * s, ly + px * s + py * c) for px, py, *_ in puntos]


def agrupar(grupos, puntos, t, radio=0.08, memoria=20):
    # Junta cada punto (en el mundo) con el grupo más cercano a menos de `radio`, o crea uno nuevo.
    # Cada grupo es [x, y, n, t_ultimo]. La media pesa las últimas `memoria` detecciones, para seguir la deriva.
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
    # Quita los grupos que no se ven desde hace más de `olvido` s, porque con la pose derivando se mezclan patas.
    grupos[:] = [g for g in grupos if t - g[3] <= olvido]
    return grupos


def centro_cuadrada(pilares, lado=0.44, tol=0.03):
    # Centro de la estación cuadrada vista desde cerca, aunque una pata quede tapada. Con 4 patas usa buscar_estacion;
    # con 3 forman un triángulo rectángulo y el centro es el punto medio de la hipotenusa.
    # Devuelve (centro_x, centro_y, n_patas, angulo) o None, con el ángulo de los lados en [-45°, 45°).
    est = buscar_estacion(pilares, lado, tol)
    if est is not None and est[0] == 'cuadrada':
        return est[1], est[2], 4, est[3]
    diag = lado * math.sqrt(2)
    mejor = None
    for patas in itertools.combinations(pilares, 3):
        d = sorted((math.dist(p, q), p, q) for p, q in itertools.combinations(patas, 2))
        err = max(abs(d[0][0] - lado), abs(d[1][0] - lado), abs(d[2][0] - diag))
        if err < tol and (mejor is None or err < mejor[0]):
            p, q = d[2][1], d[2][2]                      # extremos de la hipotenusa
            v = next(r for r in patas if r is not p and r is not q)   # la del ángulo recto
            a = math.atan2(p[1] - v[1], p[0] - v[0])     # un lado
            mejor = (err, (p[0] + q[0]) / 2, (p[1] + q[1]) / 2, (a + math.pi / 4) % (math.pi / 2) - math.pi / 4)
    return None if mejor is None else (mejor[1], mejor[2], 3, mejor[3])


def buscar_estacion(pilares, lado=0.44, tol=0.05):
    # pilares: lista de (x, y). Busca 4 en cuadrado (lado `lado`) y, si no, 3 en triángulo equilátero.
    # Devuelve (tipo, centro_x, centro_y, angulo, patas) o None. El tipo es 'cuadrada' (la buena) o 'triangular'.
    # El ángulo es el de los lados en [-45°, 45°) si es cuadrada y el de una punta en [-60°, 60°) si es triangular.
    diag = lado * math.sqrt(2)
    mejor = None
    for n_patas in (4, 3):
        for i, p0 in enumerate(pilares):
            # Solo combinaciones de pilares cercanos entre sí, para no probarlas todas
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
