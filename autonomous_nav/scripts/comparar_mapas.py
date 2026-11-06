# Dibuja el último /map de uno o varios bags de SLAM, pasado al sistema del mundo con la pose de salida del Mission Log,
# encima del mapa oficial (maps/mapa_1cm.png), para ver a simple vista cuál se parece más. Script OFFLINE (matplotlib).
# Uso:  python3 comparar_mapas.py <log fase I> <salida.png> <título>=<bag> [<título>=<bag> ...]
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from evaluar_slam import PoseSlam, componer, filas_log, inversa  # noqa: E402
from leer_mcap import leer  # noqa: E402

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def mapa_oficial():
    from PIL import Image
    img = np.array(Image.open(os.path.join(RAIZ, 'maps', 'mapa_1cm.png')).convert('L'))
    fila, col = np.nonzero(img < 128)
    return (fila + 0.5 - 71) / 100, (col + 0.5 - 2) / 100       # georreferencia de CLAUDE.md


def ocupadas_mundo(bag, salida, t0):
    m = None
    for _, _, x in leer(bag, ['/map']):
        m = x
    slam = PoseSlam(bag)
    mundo_map = componer(salida, inversa(slam(t0)))
    fila, col = np.nonzero(m['data'] >= 65)
    mx = m['origin'][0] + (col + 0.5) * m['resolution']
    my = m['origin'][1] + (fila + 0.5) * m['resolution']
    x0, y0, th = mundo_map
    c, s = math.cos(th), math.sin(th)
    return x0 + c * mx - s * my, y0 + s * mx + c * my


def main():
    f1 = filas_log(sys.argv[1])
    t0 = float(f1[0]['timestamp'])
    salida = (float(f1[0]['robot_x']), float(f1[0]['robot_y']), float(f1[0]['robot_yaw']))
    ox, oy = mapa_oficial()
    casos = [a.split('=', 1) for a in sys.argv[3:]]
    fig, ejes = plt.subplots(1, len(casos), figsize=(6 * len(casos), 6.5), squeeze=False)
    for eje, (titulo, bag) in zip(ejes[0], casos):
        wx, wy = ocupadas_mundo(bag, salida, t0)
        eje.plot(oy[::7], ox[::7], ',', color='0.6')
        eje.plot(wy, wx, '.', color='tab:red', markersize=1.5)
        eje.set_title(titulo)
        eje.set_aspect('equal')
        eje.invert_yaxis()           # como en la figura del enunciado: X hacia abajo, Y hacia la derecha
        eje.set_xlabel('Y (m)')
        eje.set_ylabel('X (m)')
    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=80)


if __name__ == '__main__':
    main()
