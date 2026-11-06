# Evalúa la pose de un SLAM (map->odom grabado en un bag) contra referencias medidas, en el sistema del mundo.
# Script OFFLINE (portátil, sin ROS). La transformación mundo<-map se fija con la pose de salida del Mission Log.
# Referencias:
#   1) la pose de la fase I en la sala hasta la puerta (corregida con el mapa y la relocalización: buena a ~5 cm);
#   2) la pose en Q tras la RELOC_Q (esquina medida con cinta);
#   3) la pose final (P Base): el robot real quedó a 15-20 cm de la cruz (3.34, 15.70);
#   4) el centro de la estación R visto por el detector de pilares con la pose del SLAM, frente a la cinta (7.17, 16.51).
# Uso:  python3 evaluar_slam.py <bag con /tf del SLAM (y odom->base_footprint)> <bag con /scan> <log fase I> <log fase II>
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from autonomous_nav import pilares as pil  # noqa: E402
from leer_mcap import leer  # noqa: E402

R_CINTA = (7.17, 16.51)
P_BASE = (3.34, 15.70)
T_PUERTA = 108.0     # s desde la salida: tras la relocalización en la puerta (ensayo del 06/11, 16:31)


def componer(a, b):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], t + b[2])


def inversa(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseSlam:
    # Pose map<-base_footprint en cualquier instante (map->odom interpolado y odom->base más cercano)
    def __init__(self, bag):
        mo, ob = [], []
        for _, _, m in leer(bag, ['/tf']):
            for tr in m:
                fila = (tr['stamp'], tr['translation'][0], tr['translation'][1], yaw(tr['rotation']))
                if tr['parent'] == 'map' and tr['child'] == 'odom':
                    mo.append(fila)
                elif tr['parent'] == 'odom' and tr['child'] == 'base_footprint':
                    ob.append(fila)
        self.mo = np.array(sorted(mo))
        self.ob = np.array(sorted(ob))
        self.mo_th = np.unwrap(self.mo[:, 3])

    def __call__(self, t):
        i = int(np.clip(np.searchsorted(self.ob[:, 0], t), 1, len(self.ob) - 1))
        o = self.ob[i] if abs(self.ob[i, 0] - t) < abs(self.ob[i - 1, 0] - t) else self.ob[i - 1]
        m = (np.interp(t, self.mo[:, 0], self.mo[:, 1]), np.interp(t, self.mo[:, 0], self.mo[:, 2]),
             np.interp(t, self.mo[:, 0], self.mo_th))
        return componer(m, (o[1], o[2], o[3]))


def filas_log(fichero):
    return [f for f in csv.DictReader(open(fichero)) if f['robot_x']]


def main():
    bag_slam, bag_scan, log1, log2 = sys.argv[1:5]
    slam = PoseSlam(bag_slam)
    f1, f2 = filas_log(log1), filas_log(log2)
    t0 = float(f1[0]['timestamp'])
    salida = (float(f1[0]['robot_x']), float(f1[0]['robot_y']), float(f1[0]['robot_yaw']))
    mundo_map = componer(salida, inversa(slam(t0)))

    def pose(t):
        return componer(mundo_map, slam(t))

    def error(f):
        t = float(f['timestamp'])
        x, y, th = pose(t)
        return math.hypot(x - float(f['robot_x']), y - float(f['robot_y'])), math.degrees(norm(th - float(f['robot_yaw'])))

    # 1) Sala hasta la puerta
    e = np.array([error(f) for f in f1 if float(f['timestamp']) - t0 <= T_PUERTA])
    print(f'1) sala (hasta la puerta): posición media {e[:, 0].mean():.2f} m, máx {e[:, 0].max():.2f}; '
          f'rumbo medio {np.abs(e[:, 1]).mean():.1f}°, máx {np.abs(e[:, 1]).max():.1f}°')
    # 2) Q tras la RELOC_Q (última fila de la fase I)
    d, a = error(f1[-1])
    print(f'2) Q tras RELOC_Q: {d:.2f} m, {a:+.1f}°')
    # 3) P Base (última fila de la fase II)
    x, y, th = pose(float(f2[-1]['timestamp']))
    print(f'3) P Base: SLAM ({x:.2f}, {y:.2f}) a {math.hypot(x - P_BASE[0], y - P_BASE[1]):.2f} m de la cruz '
          f'(el robot real quedó a 0.15-0.20 m)')
    # 4) Estación R con el detector: con la pose del SLAM y, para comparar, con la del Mission Log (odometría del nodo)
    T2 = np.array([float(f['timestamp']) for f in f2])
    X2, Y2 = np.array([float(f['robot_x']) for f in f2]), np.array([float(f['robot_y']) for f in f2])
    TH2 = np.unwrap([float(f['robot_yaw']) for f in f2])

    def pose_log(t):
        return np.interp(t, T2, X2), np.interp(t, T2, Y2), np.interp(t, T2, TH2)

    for nombre, fpose in (('SLAM', pose), ('odometría (log)', pose_log)):
        grupos, centros = [], []
        for _, t, m in leer(bag_scan, ['/scan']):
            if t < T2[0] or t > T2[-1]:
                continue
            vistos = pil.pilares_scan(m['ranges'], m['angle_min'], m['angle_increment'], m['range_min'],
                                      m['range_max'])
            pil.agrupar(grupos, pil.a_mundo(vistos, fpose(t)), t)
            pil.olvidar(grupos, t)
            est = pil.buscar_estacion([(g[0], g[1]) for g in grupos if g[2] >= 3])
            if est and est[0] == 'cuadrada':
                centros.append((est[1], est[2]))
        if not centros:
            print(f'4) estación R con {nombre}: no se ve')
            continue
        c = np.array(centros)
        dist = np.hypot(c[:, 0] - R_CINTA[0], c[:, 1] - R_CINTA[1])
        print(f'4) estación R con {nombre}: {len(c)} scans, centro mediano ({np.median(c[:, 0]):.2f}, '
              f'{np.median(c[:, 1]):.2f}), a la cinta: mediana {np.median(dist):.2f} m, máx {dist.max():.2f}')


if __name__ == '__main__':
    main()
