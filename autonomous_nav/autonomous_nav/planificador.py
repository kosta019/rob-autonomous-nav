# Planificador global: Distance Transform con costes. Es un Dijkstra lanzado desde el objetivo sobre un mapa de
# costes; el camino sale bajando desde el inicio hacia el vecino con menor distancia.
# Solo usa numpy y la librería estándar, y también lo usa fase1_node para replanificar.
# Rejilla: la fila i va con X y la columna j con Y, como en el mapa (fila = 100·X + 71).
# 'meta' es un diccionario con la resolución (m/celda) y el origen: el (X, Y) del borde de la celda (0, 0).
import heapq
import math

import numpy as np

# 8 vecinos: (di, dj, longitud del paso en celdas)
VECINOS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
           (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def mundo_a_celda(x, y, meta):
    return (int(math.floor((x - meta['origen_x']) / meta['resolucion'])),
            int(math.floor((y - meta['origen_y']) / meta['resolucion'])))


def celda_a_mundo(i, j, meta):
    # Centro de la celda
    return (meta['origen_x'] + (i + 0.5) * meta['resolucion'],
            meta['origen_y'] + (j + 0.5) * meta['resolucion'])


def holgura_por_bloques(holgura_fina, factor):
    # Holgura de cada celda gruesa: el mínimo de sus factor x factor píxeles (lo más conservador).
    # Si el mapa no es múltiplo de factor, el borde se rellena con 0 (ocupado).
    filas, cols = holgura_fina.shape
    F = -(-filas // factor) * factor
    C = -(-cols // factor) * factor
    relleno = np.zeros((F, C))
    relleno[:filas, :cols] = holgura_fina
    return relleno.reshape(F // factor, factor, C // factor, factor).min(axis=(1, 3))


def holgura_con_obstaculos(holgura, meta, puntos, margen, alcance):
    # Holgura estática + obstáculos nuevos (puntos (x, y) del LiDAR que no están en el mapa).
    # Cada punto cuenta como un obstáculo 'margen' metros más grande. Solo se recalculan las celdas
    # a menos de 'alcance' de cada punto: más lejos, el obstáculo ya no cambia el coste.
    h = holgura.copy()
    res = meta['resolucion']
    r = int(math.ceil((alcance + margen) / res))
    for x, y in puntos:
        i, j = mundo_a_celda(x, y, meta)
        i0, i1 = max(0, i - r), min(h.shape[0], i + r + 1)
        j0, j1 = max(0, j - r), min(h.shape[1], j + r + 1)
        if i0 >= i1 or j0 >= j1:
            continue
        cx = meta['origen_x'] + (np.arange(i0, i1) + 0.5) * res      # centros de las celdas de la ventana
        cy = meta['origen_y'] + (np.arange(j0, j1) + 0.5) * res
        d = np.hypot(cx[:, None] - x, cy[None, :] - y) - margen
        h[i0:i1, j0:j1] = np.minimum(h[i0:i1, j0:j1], np.maximum(d, 0.0))
    return h


def mapa_de_costes(holgura, inflado, holgura_objetivo, ganancia):
    # Coste de pisar cada celda: 1 lejos de los obstáculos, hasta 1 + ganancia pegado al borde inflado,
    # e infinito dentro del inflado. Así el camino se aleja de las paredes cuando hay sitio.
    t = np.clip((holgura_objetivo - holgura) / (holgura_objetivo - inflado), 0.0, 1.0)
    coste = 1.0 + ganancia * t
    coste[holgura <= inflado] = np.inf
    return coste


def celda_libre_mas_cercana(coste, celda):
    # Si la celda cae dentro del inflado (o fuera del mapa), devuelve la celda libre más cercana, o None si no hay.
    filas, cols = coste.shape
    i, j = celda
    if 0 <= i < filas and 0 <= j < cols and math.isfinite(coste[i, j]):
        return celda
    libres = np.argwhere(np.isfinite(coste))
    if len(libres) == 0:
        return None
    d2 = (libres[:, 0] - i) ** 2 + (libres[:, 1] - j) ** 2
    k = int(np.argmin(d2))
    return int(libres[k, 0]), int(libres[k, 1])


def distance_transform(coste, objetivo, resolucion, inicio=None):
    # Dijkstra desde el objetivo: dist[c] = coste acumulado desde la celda c hasta el objetivo.
    # Cada paso cuesta el coste de la celda vecina por la longitud del paso.
    # Si se da el inicio, se para al alcanzarlo, porque el resto del mapa no hace falta.
    filas, cols = coste.shape
    c = coste.tolist()                      # listas de Python: en este bucle son mucho más rápidas que numpy
    dist = [[math.inf] * cols for _ in range(filas)]
    hecho = [[False] * cols for _ in range(filas)]
    oi, oj = objetivo
    dist[oi][oj] = 0.0
    cola = [(0.0, oi, oj)]
    while cola:
        d, i, j = heapq.heappop(cola)
        if hecho[i][j]:
            continue
        hecho[i][j] = True
        if inicio is not None and (i, j) == inicio:
            break
        for di, dj, longitud in VECINOS:
            a = i + di
            b = j + dj
            if 0 <= a < filas and 0 <= b < cols and not hecho[a][b] and c[a][b] != math.inf:
                nd = d + c[a][b] * resolucion * longitud
                if nd < dist[a][b]:
                    dist[a][b] = nd
                    heapq.heappush(cola, (nd, a, b))
    return np.array(dist)


def extraer_camino(dist, inicio):
    # Baja desde el inicio hacia el vecino con menor distancia hasta el objetivo (dist = 0).
    # Devuelve None si desde el inicio no se llega.
    filas, cols = dist.shape
    i, j = inicio
    if not math.isfinite(dist[i, j]):
        return None
    camino = [(i, j)]
    while dist[i, j] > 0.0:
        mejor = None
        for di, dj, _ in VECINOS:
            a = i + di
            b = j + dj
            if 0 <= a < filas and 0 <= b < cols and dist[a, b] < dist[i, j]:
                if mejor is None or dist[a, b] < dist[mejor]:
                    mejor = (a, b)
        i, j = mejor
        camino.append(mejor)
    return camino


def planificar_tramo(coste, meta, p, q):
    # Camino de celdas de p a q (en el mundo), como lista de puntos (x, y), o None si no hay camino.
    # El primer y el último punto son p y q exactos, aunque p caiga dentro del inflado.
    inicio = celda_libre_mas_cercana(coste, mundo_a_celda(p[0], p[1], meta))
    objetivo = celda_libre_mas_cercana(coste, mundo_a_celda(q[0], q[1], meta))
    if inicio is None or objetivo is None:
        return None
    dist = distance_transform(coste, objetivo, meta['resolucion'], inicio)
    camino = extraer_camino(dist, inicio)
    if camino is None:
        return None
    puntos = [celda_a_mundo(i, j, meta) for i, j in camino]
    if len(puntos) == 1:
        return [tuple(p), tuple(q)]
    puntos[0] = tuple(p)
    puntos[-1] = tuple(q)
    return puntos


def holgura_en(holgura, meta, x, y):
    i, j = mundo_a_celda(x, y, meta)
    i = min(max(i, 0), holgura.shape[0] - 1)
    j = min(max(j, 0), holgura.shape[1] - 1)
    return float(holgura[i, j])


def holgura_segmento(holgura, meta, p, q, paso):
    # Holgura mínima del segmento p-q, muestreada cada 'paso' metros
    n = max(2, int(math.ceil(math.hypot(q[0] - p[0], q[1] - p[1]) / paso)) + 1)
    xs = np.linspace(p[0], q[0], n)
    ys = np.linspace(p[1], q[1], n)
    i = np.clip(np.floor((xs - meta['origen_x']) / meta['resolucion']).astype(int), 0, holgura.shape[0] - 1)
    j = np.clip(np.floor((ys - meta['origen_y']) / meta['resolucion']).astype(int), 0, holgura.shape[1] - 1)
    return float(holgura[i, j].min())


def simplificar(puntos, holgura, meta, paso, holgura_max, margen):
    # Reduce la escalera de celdas a pocos waypoints. Desde el punto i salta al j más lejano cuyo segmento i-j tenga
    # holgura >= min(holgura_max, holgura del camino entre i y j) - margen: el atajo no se acerca más a las paredes.
    h = np.array([holgura_en(holgura, meta, x, y) for x, y in puntos])
    resultado = [puntos[0]]
    i = 0
    while i < len(puntos) - 1:
        h_camino = np.minimum.accumulate(h[i:])   # h_camino[k] = mínimo del camino entre i e i + k
        j = len(puntos) - 1
        while j > i + 1:
            exigida = min(holgura_max, h_camino[j - i]) - margen
            if holgura_segmento(holgura, meta, puntos[i], puntos[j], paso) >= exigida:
                break
            j -= 1
        resultado.append(puntos[j])
        i = j
    return resultado


def longitud(puntos):
    return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(puntos, puntos[1:]))
