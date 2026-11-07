# Evalúa la corrección en el pasillo con sus paredes medidas (ajuste_paredes de fase1_node) con un bag grabado en el
# robot: rehace la pose del nodo (odometría de /odom, como real.yaml) desde la pose de salida del Mission Log, sin y con
# la corrección, y la compara con referencias medidas con cinta:
#   - el centro de la estación R visto por el detector de pilares con esa pose, frente a la cinta (7.17, 16.51);
#   - si el bag acaba en P Base: dónde estaba de verdad el robot (el real quedó a 0.15-0.20 m de la cruz el 06/11).
# No simula el robot: usa los /odom y /scan grabados. Las paredes y los parámetros, de config/fase1.yaml + real.yaml.
# Necesita rclpy (importa fase1_node): en el PC del laboratorio, o con Docker desde la raíz del repo:
#   docker run --rm -v "$PWD":/ws/src/rob-autonomous-nav:ro rob-jazzy bash -c 'source /opt/ros/jazzy/setup.bash &&
#     cd /ws/src/rob-autonomous-nav/autonomous_nav && python3 scripts/evaluar_pasillo.py <bag.mcap> <mission_log.csv>'
import csv
import math
import os
import sys

import numpy as np
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(AQUI))
sys.path.insert(0, AQUI)
from autonomous_nav import pilares as pil  # noqa: E402
from autonomous_nav.fase1_node import ajuste_paredes, limitar, normalizar  # noqa: E402
from leer_mcap import leer  # noqa: E402

R_CINTA = (7.17, 16.51)
P_BASE = (3.34, 15.70)
LIDAR_X = -0.032

# Parámetros: los de fase1.yaml con los de real.yaml encima (como en el robot)
p = yaml.safe_load(open(os.path.join(AQUI, '..', 'config', 'fase1.yaml')))['fase1_node']['ros__parameters']
p.update(yaml.safe_load(open(os.path.join(AQUI, '..', 'config', 'real.yaml')))['fase1_node']['ros__parameters'])
PAREDES = list(zip(p['corridor_wall_axis'], p['corridor_wall_value'], p['corridor_wall_from'], p['corridor_wall_to']))


def recorrer(odoms, scans, pose0, corregir):
    # Como el nodo: lo integrado desde la salida (x_rel, y_rel, th_rel) y la pose de salida, que mueve fijar_pose
    xi, yi, thi = pose0
    xr = yr = thr = 0.0
    tray = []
    k = cuenta = 0
    w = 0.0
    for j in range(len(odoms)):
        t, x, y, th = odoms[j]
        if j > 0:
            ta, xa, ya, tha = odoms[j - 1]
            ds = (x - xa) * math.cos(tha) + (y - ya) * math.sin(tha)
            dth = normalizar(th - tha)
            xr += ds * math.cos(thr + 0.5 * dth)
            yr += ds * math.sin(thr + 0.5 * dth)
            thr += dth
            w = dth / (t - ta) if t > ta else w
        X = xi + math.cos(thi) * xr - math.sin(thi) * yr
        Y = yi + math.sin(thi) * xr + math.cos(thi) * yr
        TH = normalizar(thi + thr)
        while k < len(scans) and scans[k][0] <= t:
            m = scans[k][1]
            k += 1
            cuenta += 1
            x0, x1, y0, y1 = p['corridor_corr_zone']
            if (not corregir or cuenta % p['map_corr_every_scans'] or abs(w) > p['map_corr_max_turn_rate']
                    or not (x0 <= X <= x1 and y0 <= Y <= y1)):
                continue
            r = np.array(m['ranges'], float)
            a = m['angle_min'] + np.arange(len(r)) * m['angle_increment']
            ok = (np.isfinite(r) & (r >= max(m['range_min'], p['obstacle_min_range']))
                  & (r <= min(m['range_max'], p['corridor_corr_max_range'])))
            ok[1::2] = False
            lx, ly = X + LIDAR_X * math.cos(TH), Y + LIDAR_X * math.sin(TH)
            px, py = lx + r[ok] * np.cos(TH + a[ok]), ly + r[ok] * np.sin(TH + a[ok])
            dx, dy, nx, ny = ajuste_paredes(px, py, PAREDES, p['map_corr_window'], p['map_corr_sigma_range'],
                                            p['map_corr_sigma_xy'], p['map_corr_min_points'])
            X += limitar(dx, p['map_corr_max_step'])
            Y += limitar(dy, p['map_corr_max_step'])
            # fijar_pose: mover la pose de salida para que la pose sea (X, Y, TH)
            thi = normalizar(TH - thr)
            xi = X - (math.cos(thi) * xr - math.sin(thi) * yr)
            yi = Y - (math.sin(thi) * xr + math.cos(thi) * yr)
        tray.append((t, X, Y, TH))
    return np.array(tray)


def pose_en(tray, t):
    return (np.interp(t, tray[:, 0], tray[:, 1]), np.interp(t, tray[:, 0], tray[:, 2]),
            np.interp(t, tray[:, 0], np.unwrap(tray[:, 3])))


def estacion_r(tray, scans):
    # Centro de la estación cuadrada con el detector de pilares (como evaluar_slam.py), scan a scan
    grupos, centros = [], []
    for t, m in scans:
        vistos = pil.pilares_scan(m['ranges'], m['angle_min'], m['angle_increment'], m['range_min'], m['range_max'])
        pil.agrupar(grupos, pil.a_mundo(vistos, pose_en(tray, t)), t)
        pil.olvidar(grupos, t)
        est = pil.buscar_estacion([(g[0], g[1]) for g in grupos if g[2] >= 3])
        if est and est[0] == 'cuadrada':
            centros.append((est[1], est[2]))
    return np.array(centros)


def main():
    bag, log = sys.argv[1:3]
    filas = [f for f in csv.DictReader(open(log)) if f['robot_x']]
    t0, t1 = float(filas[0]['timestamp']), float(filas[-1]['timestamp'])
    pose0 = tuple(float(filas[0][k]) for k in ('robot_x', 'robot_y', 'robot_yaw'))
    t_pbase = next((float(f['timestamp']) for f in filas if f['event'] == 'WAYPOINT PBase'), None)
    odoms, scans = [], []
    for topic, t, m in leer(bag, ['/odom', '/scan']):
        if t0 <= t <= t1:
            if topic == '/odom':
                x, y, z, w = m['orientation']
                odoms.append((t, m['position'][0], m['position'][1],
                              math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
            else:
                scans.append((t, m))
    print(f'{len(odoms)} /odom y {len(scans)} /scan en {t1 - t0:.0f} s; salida {pose0}; paredes {PAREDES}')
    for nombre, corregir in (('sin corrección', False), ('con las paredes del pasillo', True)):
        tray = recorrer(odoms, scans, pose0, corregir)
        c = estacion_r(tray, scans)
        texto = 'R no se ve'
        if len(c):
            d = np.hypot(c[:, 0] - R_CINTA[0], c[:, 1] - R_CINTA[1])
            texto = (f'R en ({np.median(c[:, 0]):.2f}, {np.median(c[:, 1]):.2f}), a la cinta {100 * np.median(d):.1f} cm '
                     f'(mediana de {len(c)} scans)')
        if t_pbase is not None:
            x, y, _ = pose_en(tray, t_pbase)
            texto += f'; al llegar a P Base, robot en ({x:.2f}, {y:.2f}): a {100 * math.hypot(x - P_BASE[0], y - P_BASE[1]):.0f} cm de la cruz'
        print(f'{nombre}: {texto}')


if __name__ == '__main__':
    main()
