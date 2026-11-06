# Pasa el detector de pilares (autonomous_nav/pilares.py) por un bag grabado, con la pose del Mission Log de esa prueba.
# Script OFFLINE (portátil, sin ROS): lee el MCAP con leer_mcap.py.
# Uso:  python3 pilares_bag.py <bag.mcap> <mission_log.csv>
# Imprime cada vez que cambia lo que se ve (tipo de estación o centro movido > 5 cm).
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from autonomous_nav import pilares as pil  # noqa: E402
from leer_mcap import leer  # noqa: E402

MIN_DET = 3      # detecciones para que un grupo cuente como pilar


def pose_log(fichero):
    # Tiempos y poses (x, y, yaw desenrollado) del Mission Log, para interpolar
    filas = [f for f in csv.DictReader(open(fichero)) if f['robot_x']]
    t = np.array([float(f['timestamp']) for f in filas])
    x = np.array([float(f['robot_x']) for f in filas])
    y = np.array([float(f['robot_y']) for f in filas])
    th = np.unwrap([float(f['robot_yaw']) for f in filas])
    return t, x, y, th


def main():
    bag, log = sys.argv[1], sys.argv[2]
    T, X, Y, TH = pose_log(log)
    grupos = []
    t0 = None
    ultimo = None
    vistas = {}
    for _, t, m in leer(bag, ['/scan']):
        if t < T[0] or t > T[-1]:
            continue
        t0 = t if t0 is None else t0
        pose = (np.interp(t, T, X), np.interp(t, T, Y), np.interp(t, T, TH))
        vistos = pil.pilares_scan(m['ranges'], m['angle_min'], m['angle_increment'], m['range_min'], m['range_max'])
        pil.agrupar(grupos, pil.a_mundo(vistos, pose), t)
        pil.olvidar(grupos, t)
        est = pil.buscar_estacion([(g[0], g[1]) for g in grupos if g[2] >= MIN_DET])
        clave = None if est is None else (est[0], round(est[1], 1), round(est[2], 1))
        if est:
            vistas.setdefault(est[0], []).append(est[1:4])
        if clave != ultimo:
            ultimo = clave
            txt = 'nada' if est is None else (f'{est[0]} ({est[1]:.3f}, {est[2]:.3f}) {math.degrees(est[3]):5.1f}°'
                                              f'  robot ({pose[0]:.2f}, {pose[1]:.2f})')
            print(f'{t - t0:7.1f} s  {txt}')
    for tipo, v in vistas.items():
        v = np.array(v)
        print(f'{tipo}: {len(v)} scans, centro mediano ({np.median(v[:, 0]):.3f}, {np.median(v[:, 1]):.3f}), '
              f'ángulo mediano {math.degrees(np.median(v[:, 2])):.1f}°')


if __name__ == '__main__':
    main()
