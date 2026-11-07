#!/usr/bin/env python3
# Script offline (en el portátil): planifica la ruta de la fase I sobre el mapa.
# Usa Pillow, scipy y matplotlib, que no se usan en los nodos; los ficheros que genera se suben al repositorio.
# Uso: python3 autonomous_nav/scripts/planificar_fase1.py
# Lee config/fase1.yaml y genera:
#   - config/fase1_ruta.yaml         waypoints para fase1_node (se pasa como segundo --params-file)
#   - maps/holgura_5cm.npy + .yaml   holgura estática de la rejilla de 5 cm, para detectar obstáculos nuevos
#   - ../docs/ruta_fase1_plan.png    figura de la ruta para el informe
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # carpeta del paquete autonomous_nav/
sys.path.insert(0, RAIZ)
from autonomous_nav import planificador as pl  # noqa: E402


def cargar_parametros():
    with open(os.path.join(RAIZ, 'config', 'fase1.yaml')) as f:
        return yaml.safe_load(f)['fase1_node']['ros__parameters']


def mapa_ocupacion(p, con_zonas=True):
    # Mapa de 1 cm (True = ocupado) sin las motas sueltas y, si con_zonas, con las zonas prohibidas pintadas.
    imagen = np.array(Image.open(os.path.join(RAIZ, p['map_image'])).convert('L'))
    ocupado = imagen < 128
    for fila, col in zip(p['map_remove_rows'], p['map_remove_cols']):
        ocupado[fila, col] = False
    if not con_zonas:
        return ocupado
    meta = meta_1cm(p)
    res = meta['resolucion']
    for x0, x1, y0, y1 in zip(p['nogo_x_min'], p['nogo_x_max'], p['nogo_y_min'], p['nogo_y_max']):
        # Píxeles cuyo centro cae dentro del rectángulo
        f0 = max(0, math.ceil((x0 - meta['origen_x']) / res - 0.5))
        f1 = min(ocupado.shape[0] - 1, math.floor((x1 - meta['origen_x']) / res - 0.5))
        c0 = max(0, math.ceil((y0 - meta['origen_y']) / res - 0.5))
        c1 = min(ocupado.shape[1] - 1, math.floor((y1 - meta['origen_y']) / res - 0.5))
        ocupado[f0:f1 + 1, c0:c1 + 1] = True
    return ocupado


def meta_1cm(p):
    res = p['map_resolution']
    return {'resolucion': res, 'origen_x': -p['map_origin_row'] * res, 'origen_y': -p['map_origin_col'] * res}


def calcular_holguras(p):
    # Holgura (distancia al ocupado más cercano, en m) a 1 cm y en la rejilla de grid_cell
    ocupado = mapa_ocupacion(p)
    meta1 = meta_1cm(p)
    h1 = distance_transform_edt(~ocupado) * meta1['resolucion']
    factor = int(round(p['grid_cell'] / meta1['resolucion']))
    h5 = pl.holgura_por_bloques(h1, factor)
    meta5 = {'resolucion': p['grid_cell'], 'origen_x': meta1['origen_x'], 'origen_y': meta1['origen_y']}
    return ocupado, h1, meta1, h5, meta5


def planificar(p, h1, meta1, h5, meta5, secuencia, inflado):
    # secuencia: lista de (nombre, x, y). Devuelve los tramos como (nombre_a, nombre_b, puntos simplificados, longitud,
    # holgura mínima, segundos), o None si un tramo no tiene camino.
    coste = pl.mapa_de_costes(h5, inflado, p['clearance_target'], p['clearance_cost_gain'])
    tramos = []
    for (na, xa, ya), (nb, xb, yb) in zip(secuencia, secuencia[1:]):
        t0 = time.perf_counter()
        celdas = pl.planificar_tramo(coste, meta5, (xa, ya), (xb, yb))
        if celdas is None:
            return None
        puntos = pl.simplificar(celdas, h1, meta1, p['simplify_step'],
                                p['simplify_clearance_cap'], p['simplify_margin'])
        segundos = time.perf_counter() - t0
        h_min = min(pl.holgura_segmento(h1, meta1, a, b, p['simplify_step']) for a, b in zip(puntos, puntos[1:]))
        tramos.append((na, nb, puntos, pl.longitud(puntos), h_min, segundos))
    return tramos


def secuencia_fase1(p):
    # Puntos de planning_sequence y, si la pose final no coincide con el último, un tramo más hasta ella
    puntos = dict(zip(p['point_names'], zip(p['point_x'], p['point_y'])))
    secuencia = [(n, puntos[n][0], puntos[n][1]) for n in p['planning_sequence']]
    xi, yi = p['initial_pose'][0], p['initial_pose'][1]
    if math.hypot(xi - secuencia[0][1], yi - secuencia[0][2]) > 0.05:
        print(f'AVISO: initial_pose ({xi}, {yi}) no coincide con {secuencia[0][0]}, '
              f'el primer punto de planning_sequence')
    xf, yf = p['final_pose'][0], p['final_pose'][1]
    if math.hypot(xf - secuencia[-1][1], yf - secuencia[-1][2]) > p['tol_mandatory']:
        secuencia.append(('final', xf, yf))
    return secuencia


def lista_waypoints(tramos):
    # Waypoints para fase1_node: sin el punto de salida, con los intermedios nombrados "A-B_1", "A-B_2"...
    # Sin el punto 'final': el nodo lo añade él solo a partir de final_pose.
    nombres, xs, ys = [], [], []
    for na, nb, puntos, _, _, _ in tramos:
        for k, (x, y) in enumerate(puntos[1:-1], start=1):
            nombres.append(f'{na}-{nb}_{k}')
            xs.append(x)
            ys.append(y)
        if nb != 'final':
            nombres.append(nb)
            xs.append(puntos[-1][0])
            ys.append(puntos[-1][1])
    return nombres, xs, ys


def escribir_ruta_yaml(ruta, tramos, inflado):
    nombres, xs, ys = lista_waypoints(tramos)
    total = sum(t[3] for t in tramos)
    h_min = min(t[4] for t in tramos)
    with open(ruta, 'w') as f:
        f.write(f"# GENERADO por scripts/planificar_fase1.py el {datetime.now():%Y-%m-%d %H:%M}. No editar a mano:\n")
        f.write("# cambia config/fase1.yaml y vuelve a ejecutar el script.\n")
        f.write(f"# Inflado {inflado:.2f} m. Longitud {total:.2f} m. Holgura mínima {h_min:.2f} m.\n")
        f.write("fase1_node:\n  ros__parameters:\n")
        f.write('    waypoint_names: [' + ', '.join(f'"{n}"' for n in nombres) + ']\n')
        f.write('    waypoints_x:    [' + ', '.join(f'{x:.3f}' for x in xs) + ']\n')
        f.write('    waypoints_y:    [' + ', '.join(f'{y:.3f}' for y in ys) + ']\n')


def escribir_holgura(ruta_npy, h5, meta5, p):
    np.save(ruta_npy, h5.astype(np.float32))
    with open(ruta_npy.replace('.npy', '.yaml'), 'w') as f:
        f.write(f"# GENERADO por scripts/planificar_fase1.py el {datetime.now():%Y-%m-%d %H:%M}.\n")
        f.write("# Holgura estática (m) de cada celda: el mínimo de sus píxeles de 1 cm, con las zonas prohibidas.\n")
        f.write("# Celda (i, j): i con X, j con Y. X del borde superior de la celda i = origen_x + i * resolucion.\n")
        f.write(f"resolucion: {meta5['resolucion']}\norigen_x: {meta5['origen_x']}\norigen_y: {meta5['origen_y']}\n")
        f.write(f"filas: {h5.shape[0]}\ncolumnas: {h5.shape[1]}\n")
        f.write('zonas_prohibidas: [' + ', '.join(f'"{n}"' for n in p['nogo_names']) + ']\n')


def dibujar(ruta_png, h5, meta1, meta5, p, tramos, inflado):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch, Rectangle

    ocupado = mapa_ocupacion(p, con_zonas=False)   # las zonas prohibidas se dibujan aparte, encima
    filas, cols = ocupado.shape
    # Ejes como en la figura del enunciado: Y hacia la derecha y X hacia abajo
    ext1 = [meta1['origen_y'], meta1['origen_y'] + cols * meta1['resolucion'],
            meta1['origen_x'] + filas * meta1['resolucion'], meta1['origen_x']]
    ext5 = [meta5['origen_y'], meta5['origen_y'] + h5.shape[1] * meta5['resolucion'],
            meta5['origen_x'] + h5.shape[0] * meta5['resolucion'], meta5['origen_x']]
    fig, ax = plt.subplots(figsize=(15, 9.5))
    ax.imshow(np.ma.masked_where(h5 > inflado, h5 <= inflado), cmap=ListedColormap(['#d9d9d9']), extent=ext5)
    ax.imshow(np.ma.masked_where(~ocupado, ocupado), cmap=ListedColormap(['#444444']), extent=ext1)
    for x0, x1, y0, y1 in zip(p['nogo_x_min'], p['nogo_x_max'], p['nogo_y_min'], p['nogo_y_max']):
        ax.add_patch(Rectangle((y0, x0), y1 - y0, x1 - x0, facecolor='#f5c98b', edgecolor='#c98a2e',
                               hatch='xx', alpha=0.8))
    for na, nb, puntos, _, _, _ in tramos:
        ax.plot([q[1] for q in puntos], [q[0] for q in puntos], '-', color='tab:blue', lw=2)
        ax.plot([q[1] for q in puntos[1:-1]], [q[0] for q in puntos[1:-1]], 'o', color='tab:blue', ms=5)
    secuencia = [(t[0], t[2][0]) for t in tramos] + [(tramos[-1][1], tramos[-1][2][-1])]
    for nombre, (x, y) in secuencia:
        obligatorio = nombre in p['mandatory_waypoints']
        ax.plot(y, x, 's' if obligatorio else 'o', color='tab:orange' if obligatorio else 'tab:green', ms=8)
        ax.annotate(nombre, (y, x), textcoords='offset points', xytext=(6, 6), fontsize=10)
    ax.legend(handles=[Patch(color='#444444', label='ocupado (mapa)'),
                       Patch(color='#d9d9d9', label=f'inflado {inflado:.2f} m (rejilla 5 cm)'),
                       Patch(facecolor='#f5c98b', edgecolor='#c98a2e', hatch='xx', label='zona prohibida (estimada)'),
                       plt.Line2D([], [], color='tab:blue', marker='o', ms=5, label='ruta planificada'),
                       plt.Line2D([], [], color='tab:orange', marker='s', ls='', label='punto obligatorio')],
              loc='lower left')
    total = sum(t[3] for t in tramos)
    h_min = min(t[4] for t in tramos)
    ax.set_title(f'Fase I · Distance Transform con costes · inflado {inflado:.2f} m · '
                 f'{total:.1f} m · holgura mínima {h_min:.2f} m')
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('X (m)')
    ax.set_xlim(ext1[0], ext1[1])
    ax.set_ylim(ext1[2], ext1[3])
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ruta_png, dpi=110)
    plt.close(fig)


def main():
    p = cargar_parametros()
    inflado = p['inflation_radius']
    t0 = time.perf_counter()
    ocupado, h1, meta1, h5, meta5 = calcular_holguras(p)
    print(f'Holguras: mapa {ocupado.shape} a 1 cm, rejilla {h5.shape} a {meta5["resolucion"]} m '
          f'({time.perf_counter() - t0:.2f} s)')

    puntos = dict(zip(p['point_names'], zip(p['point_x'], p['point_y'])))
    print('\nHolgura a 1 cm en los puntos del enunciado:')
    print('  ' + '  '.join(f'{n} {pl.holgura_en(h1, meta1, x, y):.2f}' for n, (x, y) in puntos.items()))

    secuencia = secuencia_fase1(p)
    tramos = planificar(p, h1, meta1, h5, meta5, secuencia, inflado)
    if tramos is None:
        sys.exit(f'ERROR: no hay camino con inflado {inflado} m')
    print(f'\nRuta con inflado {inflado:.2f} m:')
    for na, nb, pts, lon, hm, seg in tramos:
        intermedios = ' '.join(f'({x:.2f}, {y:.2f})' for x, y in pts[1:-1]) or '-'
        print(f'  {na:>5} -> {nb:<5} {lon:5.2f} m  holgura mín {hm:.2f} m  {seg:.2f} s  intermedios: {intermedios}')
    print(f'  TOTAL {sum(t[3] for t in tramos):.2f} m, holgura mínima {min(t[4] for t in tramos):.2f} m')

    escribir_ruta_yaml(os.path.join(RAIZ, 'config', 'fase1_ruta.yaml'), tramos, inflado)
    escribir_holgura(os.path.join(RAIZ, 'maps', 'holgura_5cm.npy'), h5, meta5, p)
    ruta_png = os.path.join(os.path.dirname(RAIZ), 'docs', 'ruta_fase1_plan.png')
    dibujar(ruta_png, h5, meta1, meta5, p, tramos, inflado)
    print('\nEscrito: config/fase1_ruta.yaml, maps/holgura_5cm.npy, maps/holgura_5cm.yaml, ../docs/ruta_fase1_plan.png')


if __name__ == '__main__':
    main()
