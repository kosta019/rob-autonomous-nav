# Saca de los bags del 06/11 los scans (y la pose del Mission Log) que usa test/test_pilares.py, a test/datos/.
# Script OFFLINE, solo hace falta volver a ejecutarlo si se cambian los tramos. Necesita los bags descomprimidos en bags/.
# Uso (desde la raíz del repo):  python3 autonomous_nav/scripts/extraer_scans_test.py
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from leer_mcap import leer  # noqa: E402
from pilares_bag import pose_log  # noqa: E402

D = 'test_real/datos_20261106/'
# nombre: (bag, mission log, desde, hasta) en s desde el primer scan dentro del log
TRAMOS = {
    'R_parado': ('fase1_20261106_171005', 'mission_log_aproximacion_R.csv', 40, 50),   # a 0.55 m de R, mirándola
    'T_rodeo': ('fase1_20261106_161359', 'mission_log_prueba_estaciones.csv', 160, 172),  # pasando junto a T
    'pasillo_Q': ('fase1_20261106_163111', 'mission_log_ensayo_fase2_pasillo.csv', 0, 60),  # de Q, sin estaciones
}

salida = {}
for nombre, (bag, log, t1, t2) in TRAMOS.items():
    T, X, Y, TH = pose_log(D + log)
    scans, poses, t0 = [], [], None
    for _, t, m in leer(f'bags/{bag}/{bag}_0.mcap', ['/scan']):
        if t < T[0] or t > T[-1]:
            continue
        t0 = t if t0 is None else t0
        if t1 <= t - t0 <= t2:
            scans.append(m['ranges'])
            poses.append((t, np.interp(t, T, X), np.interp(t, T, Y), np.interp(t, T, TH)))
    salida[nombre + '_scans'] = np.array(scans, dtype=np.float32)
    salida[nombre + '_poses'] = np.array(poses)
    print(nombre, len(scans), 'scans')
# Todos los scans del 06/11: angle_min 0, 360 rayos de 1°, range_min 0.12, range_max 3.5
np.savez_compressed('autonomous_nav/test/datos/scans_estaciones.npz', **salida)
