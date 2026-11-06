# Mide el retraso del sello de tiempo de /scan respecto a /odom con los giros en la sala de un bag (script OFFLINE).
# Cada scan tomado girando se proyecta al mundo con la pose del instante (sello - delta) y se mide lo lejos que caen sus
# puntos de las paredes del mapa (mapa de distancias de ajuste_mapa). El delta con los puntos más pegados a las paredes
# es el retraso (positivo: el scan es de antes de lo que dice su sello). Girando a w rad/s, un retraso delta gira el
# scan w·delta respecto a la pose.
# Pose: la del Mission Log (corregida con el mapa) en la fila más cercana, más lo que se mueve /odom desde ella.
# Uso:  python3 retraso_scan.py <bag.mcap> <mission_log.csv> <t_max (s desde el inicio del log)>
import csv
import math
import os
import sys

import numpy as np
import yaml

from leer_mcap import leer

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W_MIN = 0.3        # rad/s: solo scans girando al menos así
VENTANA = 0.30     # m: puntos más lejos de una pared no cuentan (mesas, sillas, personas)
LIDAR_X = -0.032


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


mapa = np.load(os.path.join(RAIZ, 'maps', 'distancias_2cm.npy'))
meta = yaml.safe_load(open(os.path.join(RAIZ, 'maps', 'distancias_2cm.yaml')))
filas = [f for f in csv.DictReader(open(sys.argv[2])) if f['robot_x']]
TL = np.array([float(f['timestamp']) for f in filas])
PL = np.array([(float(f['robot_x']), float(f['robot_y']), float(f['robot_yaw'])) for f in filas])
t_max = TL[0] + float(sys.argv[3])

scans, odom = [], []
for topic, t, m in leer(sys.argv[1], ['/scan', '/odom']):
    if topic == '/scan':
        scans.append((m['header']['stamp'], np.array(m['ranges']), m['angle_min'], m['angle_increment'], m))
    else:
        odom.append((m['header']['stamp'], m['position'][0], m['position'][1], yaw(m['orientation'])))
odom = np.array(odom)
odom[:, 3] = np.unwrap(odom[:, 3])


def pose_odom(t):
    return [np.interp(t, odom[:, 0], odom[:, k]) for k in (1, 2, 3)]


def pose(t):
    # Fila del log más cercana y el movimiento de /odom desde ella (en los ejes del robot)
    k = int(np.argmin(np.abs(TL - t)))
    x0, y0, th0 = PL[k]
    a, b = pose_odom(TL[k]), pose_odom(t)
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    u, v = c * dx + s * dy, -s * dx + c * dy          # avance en los ejes del robot en TL[k]
    c0, s0 = math.cos(th0), math.sin(th0)
    return x0 + c0 * u - s0 * v, y0 + s0 * u + c0 * v, th0 + (b[2] - a[2])


def error_medio(t, r, a, delta):
    x, y, th = pose(t - delta)
    ok = np.isfinite(r) & (r > 0.12) & (r < 3.5)
    lx, ly = x + LIDAR_X * math.cos(th), y + LIDAR_X * math.sin(th)
    px, py = lx + r[ok] * np.cos(th + a[ok]), ly + r[ok] * np.sin(th + a[ok])
    fi = np.floor((px - meta['origen_x']) / meta['resolucion']).astype(int)
    co = np.floor((py - meta['origen_y']) / meta['resolucion']).astype(int)
    dentro = (fi >= 0) & (fi < mapa.shape[1]) & (co >= 0) & (co < mapa.shape[2])
    d = mapa[0, fi[dentro], co[dentro]]
    return d[d < VENTANA]


girando, rectos = [], []
for t, r, amin, inc, _ in scans:
    if not (TL[0] <= t <= t_max):
        continue
    w = (pose_odom(t + 0.1)[2] - pose_odom(t - 0.1)[2]) / 0.2
    a = amin + np.arange(len(r)) * inc
    (girando if abs(w) > W_MIN else rectos).append((t, r, a))
print(f'{len(girando)} scans girando (> {W_MIN} rad/s), {len(rectos)} sin girar')
for delta in np.arange(-0.10, 0.31, 0.05):
    for nombre, grupo in (('girando', girando), ('sin girar', rectos[::5])):
        d = np.concatenate([error_medio(t, r, a, delta) for t, r, a in grupo])
        print(f'retraso {delta:+.2f} s, {nombre:9s}: distancia media a la pared {100 * d.mean():.2f} cm ({len(d)} puntos)')
