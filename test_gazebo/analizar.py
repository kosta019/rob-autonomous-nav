#!/usr/bin/env python3
# OFFLINE: analiza una prueba en el mundo del laboratorio a partir de la pose REAL de Gazebo (real.csv).
# Uso:  python3 test_gazebo/analizar.py test_gazebo/resultados/<nombre> [config/fase1_ruta.yaml ...]
# Mide lo que el /odom no puede decir: si el robot se acerca a una pared o a una caja lo bastante para
# chocar, si pisa una zona prohibida y la precisión real en los puntos obligatorios y la pose final.
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

DIR = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(DIR, '..', 'autonomous_nav')
sys.path.insert(0, os.path.join(PKG, 'scripts'))
import planificar_fase1 as pf  # noqa: E402

RADIO_ROBOT = 0.105   # m, radio circunscrito del Burger (CLAUDE.md)
LADO_CAJA = 0.2       # m, las cajas de prueba.sh


def main():
    res_dir = sys.argv[1]
    p = pf.cargar_parametros()
    for extra in sys.argv[2:]:   # los mismos .yaml que se pasaron al nodo
        with open(os.path.join(PKG, extra)) as f:
            p.update(yaml.safe_load(f)['fase1_node']['ros__parameters'])

    real = np.array([[float(v) for v in fila[1:]] for fila in csv.reader(open(os.path.join(res_dir, 'real.csv')))
                     if fila and fila[0] != 't_reloj'])
    xs, ys, yaws = real[:, 0], real[:, 1], real[:, 2]
    print(f'{len(real)} muestras de la pose real; recorrido {np.sum(np.hypot(np.diff(xs), np.diff(ys))):.2f} m')

    # Paredes: holgura del mapa de 1 cm SIN las zonas prohibidas (esas no son paredes)
    ocupado = pf.mapa_ocupacion(p, con_zonas=False)
    meta = pf.meta_1cm(p)
    h = distance_transform_edt(~ocupado) * meta['resolucion']
    filas = np.clip(np.floor((xs - meta['origen_x']) / meta['resolucion']).astype(int), 0, h.shape[0] - 1)
    cols = np.clip(np.floor((ys - meta['origen_y']) / meta['resolucion']).astype(int), 0, h.shape[1] - 1)
    hp = h[filas, cols]
    k = int(np.argmin(hp))
    print(f'Pared más cercana: {hp[k]:.3f} m del centro en ({xs[k]:.2f}, {ys[k]:.2f}) '
          f'-> {hp[k] - RADIO_ROBOT:.3f} m desde el borde del robot '
          f'{"CHOQUE" if hp[k] < RADIO_ROBOT else "sin choque"}')

    # Cajas
    log_cajas = os.path.join(res_dir, 'cajas.log')
    if os.path.exists(log_cajas):
        for m in re.finditer(r'(caja\d+) en \(([-\d.]+), ([-\d.]+)\)', open(log_cajas).read()):
            cx, cy = float(m.group(2)), float(m.group(3))
            dx = np.maximum(np.abs(xs - cx) - LADO_CAJA / 2, 0)
            dy = np.maximum(np.abs(ys - cy) - LADO_CAJA / 2, 0)
            d = np.hypot(dx, dy)
            k = int(np.argmin(d))
            print(f'{m.group(1)} ({cx}, {cy}): {d[k]:.3f} m del centro a su cara -> '
                  f'{d[k] - RADIO_ROBOT:.3f} m desde el borde {"CHOQUE" if d[k] < RADIO_ROBOT else "sin choque"}')

    # Zonas prohibidas: ¿el robot (su círculo) las pisa?
    for n, x0, x1, y0, y1 in zip(p['nogo_names'], p['nogo_x_min'], p['nogo_x_max'], p['nogo_y_min'], p['nogo_y_max']):
        dx = np.maximum(np.maximum(x0 - xs, xs - x1), 0)
        dy = np.maximum(np.maximum(y0 - ys, ys - y1), 0)
        d = np.hypot(dx, dy)
        print(f'Zona {n}: distancia mínima del centro {d.min():.3f} m '
              f'{"PISADA" if d.min() < RADIO_ROBOT else "no pisada"}')

    # Precisión real: lo más cerca que pasa de cada punto obligatorio, y la pose final
    puntos = dict(zip(p['waypoint_names'], zip(p['waypoints_x'], p['waypoints_y'])))
    for n in p['mandatory_waypoints']:
        if n in puntos:
            d = np.hypot(xs - puntos[n][0], ys - puntos[n][1])
            print(f'{n}: pasa a {d.min():.3f} m (pose real)')
    xf, yf, thf = p['final_pose']
    print(f'Pose final real ({xs[-1]:.3f}, {ys[-1]:.3f}, {yaws[-1]:.3f}): error {math.hypot(xs[-1] - xf, ys[-1] - yf):.3f} m, '
          f'{abs(math.atan2(math.sin(yaws[-1] - thf), math.cos(yaws[-1] - thf))):.3f} rad')

    # Eventos del Mission Log
    eventos = {}
    for fichero in glob.glob(os.path.join(res_dir, 'mission_log_*.csv')):
        for fila in csv.DictReader(open(fichero)):
            if fila['event']:
                clave = fila['event'].split(' ')[0]
                eventos[clave] = eventos.get(clave, 0) + 1
    print('Eventos del log:', ', '.join(f'{k} x{v}' for k, v in eventos.items()))


if __name__ == '__main__':
    main()
