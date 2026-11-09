# Figura del mapa entregable (misión de las 16:39 del 09/11) para la memoria. Script OFFLINE (matplotlib, Pillow).
# Necesita, de la rama arreglo-fase3-lado, test_real/datos_20261109/: mapa_sin_paradas.pgm, el Mission Log y el bag descomprimido.
# Uso: python3 docs/figura_mapa_entrega.py <carpeta con esos ficheros> <salida.png>
# El .pgm está en el frame map del SLAM; aquí se dibuja en el sistema del enunciado con la pose de salida del Mission Log.
import csv
import glob
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, 'autonomous_nav', 'scripts'))
sys.path.insert(0, os.path.join(RAIZ, 'autonomous_nav'))
from evaluar_slam import PoseSlam, componer, inversa  # noqa: E402

DATOS, SALIDA = sys.argv[1], sys.argv[2]
RES, OX, OY = 0.05, -11.011, -39.047          # mapa_sin_paradas.yaml
slam = PoseSlam(glob.glob(os.path.join(DATOS, 'bag', '*', '*.mcap'))[0])
filas = list(csv.DictReader(open(os.path.join(DATOS, 'mission_log_bot11_sin_paradas.csv'))))
ini = next(f for f in filas if f['event'].startswith('POSE_INICIAL'))
pose = (float(ini['robot_x']), float(ini['robot_y']), float(ini['robot_yaw']))
x0, y0, th = componer(pose, inversa(slam(float(ini['timestamp']))))   # mundo <- map
c, s = math.cos(th), math.sin(th)


def patas(evento):
    # Las patas del Mission Log, llevadas al map con la pose relativa al robot en ese instante, y de ahí al mundo
    res = []
    for f in filas:
        if f['event'] != evento:
            continue
        rob = (float(f['robot_x']), float(f['robot_y']), float(f['robot_yaw']))
        rel = componer(inversa(rob), (float(f['obstacle_x']), float(f['obstacle_y']), 0.0))
        mx, my = componer(slam(float(f['timestamp'])), rel)[:2]
        res.append((x0 + c * mx - s * my, y0 + s * mx + c * my))
    return np.array(res)


im = np.array(Image.open(os.path.join(DATOS, 'mapa_sin_paradas.pgm')))[::-1]     # fila 0 = y mínima
H, W = im.shape
MX, MY = np.meshgrid(OX + np.arange(W + 1) * RES, OY + np.arange(H + 1) * RES)
WX, WY = x0 + c * MX - s * MY, y0 + s * MX + c * MY
gris = np.where(im == 0, 0.0, np.where(im >= 250, 1.0, 0.82))
of = np.array(Image.open(os.path.join(RAIZ, 'autonomous_nav', 'maps', 'mapa_1cm.png')).convert('L')) < 128
borde = of & ~(np.roll(of, 1, 0) & np.roll(of, -1, 0) & np.roll(of, 1, 1) & np.roll(of, -1, 1))  # solo el contorno
fi, co = np.nonzero(borde)
OFX, OFY = (fi + 0.5 - 71) / 100, (co + 0.5 - 2) / 100
pc = patas('PILAR')

fig, (a, b) = plt.subplots(1, 2, figsize=(11, 5.2), gridspec_kw={'width_ratios': [2.3, 1]})
a.pcolormesh(WY, WX, gris, cmap='gray', vmin=0, vmax=1, shading='flat')
a.plot(OFY[::3], OFX[::3], '.', color='tab:orange', ms=0.8, alpha=0.8)
a.plot(pc[:, 1], pc[:, 0], 'o', mfc='none', mec='tab:red', ms=6)
a.set_xlim(-0.6, 19.5)
a.set_ylim(11.0, -1.5)
a.set_title('Mapa entregable, en el sistema del enunciado (naranja: plano oficial)', fontsize=10)
cx, cy = pc.mean(axis=0)
b.pcolormesh(WY, WX, gris, cmap='gray', vmin=0, vmax=1, shading='flat')
b.plot(pc[:, 1], pc[:, 0], 'o', mfc='none', mec='tab:red', ms=13, mew=1.5, label='patas detectadas')
b.set_xlim(cy - 0.75, cy + 0.75)
b.set_ylim(cx + 0.75, cx - 0.75)
b.set_title('Detalle: estación cuadrada', fontsize=10)
b.legend(loc='lower right', fontsize=8)
for e in (a, b):
    e.set_aspect('equal')
    e.set_xlabel('Y (m)')
    e.set_ylabel('X (m)')
fig.tight_layout()
fig.savefig(SALIDA, dpi=150)
