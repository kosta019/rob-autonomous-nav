# Con el arreglo: desde E~ (alcanzado), ¿hay camino hasta Door rodeando la silla? Obstáculos: scans del bag con el
# robot parado en (4.982, 5.68), como en bucle_E.py.
import math, sys
import numpy as np, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import os
PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'autonomous_nav')
sys.path.insert(0, PKG)
from autonomous_nav import planificador as pl
P = {}
for f in ('fase1.yaml', 'fase1_ruta.yaml', 'real.yaml'):
    P.update(yaml.safe_load(open(f'{PKG}/config/{f}'))['fase1_node']['ros__parameters'])
hol = np.load(f'{PKG}/{P["static_clearance_file"]}')
m = yaml.safe_load(open(f'{PKG}/{P["static_clearance_file"]}'.replace('.npy', '.yaml')))
meta = {'resolucion': m['resolucion'], 'origen_x': m['origen_x'], 'origen_y': m['origen_y']}
x, y, th = 4.982, 5.68, 1.795
t0, t1 = 1793967442.0, 1793967450.0          # últimos 8 s parado (memoria de 8 s)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
obst = {}
lx, ly = x + P['lidar_offset_x'] * math.cos(th), y + P['lidar_offset_x'] * math.sin(th)
while r.has_next():
    _, data, t = r.read_next()
    if not (t0 < t * 1e-9 < t1): continue
    s = deserialize_message(data, LaserScan)
    rr = np.array(s.ranges, dtype=float); a = s.angle_min + np.arange(len(rr)) * s.angle_increment
    ok = np.isfinite(rr) & (rr >= max(s.range_min, P['obstacle_min_range'])) & (rr <= min(s.range_max, P['obstacle_max_range']))
    for ox, oy in zip(lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])):
        i, j = pl.mundo_a_celda(ox, oy, meta)
        if 0 <= i < hol.shape[0] and 0 <= j < hol.shape[1] and hol[i, j] > P['obstacle_static_margin']:
            obst[(i, j)] = (ox, oy)
O = list(obst.values())
h = pl.holgura_con_obstaculos(hol, meta, O, P['obstacle_plan_margin'], P['clearance_target'])
ini = (4.965, 5.755)
door = (6.490, 11.682)
for inf in (P['inflation_radius'], P['inflation_radius_retry'], P['inflation_radius_min']):
    celdas = pl.planificar_tramo(pl.mapa_de_costes(h, inf, P['clearance_target'], P['clearance_cost_gain']), meta, ini, door)
    if celdas is None:
        print(f'inflado {inf}: sin camino'); continue
    pts = pl.simplificar(celdas, h, meta, P['simplify_step'], P['simplify_clearance_cap'], P['simplify_margin'])
    Oa = np.array(O)
    cerca = min(np.hypot(Oa[:, 0] - cx, Oa[:, 1] - cy).min() for cx, cy in [pl.celda_a_mundo(i, j, meta) for i, j in celdas]) if False else None
    print(f'inflado {inf}: camino, {pl.longitud(pts):.2f} m; primeros waypoints {[(round(p[0], 2), round(p[1], 2)) for p in pts[:5]]}')
    # distancia mínima del camino simplificado a los obstáculos nuevos
    dmin = 9
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        tt = np.linspace(0, 1, int(math.hypot(bx - ax, by - ay) / 0.02) + 2)
        smp = np.column_stack([ax + tt * (bx - ax), ay + tt * (by - ay)])
        dmin = min(dmin, np.hypot(smp[:, None, 0] - Oa[None, :, 0], smp[:, None, 1] - Oa[None, :, 1]).min())
    print(f'   pasa a {dmin:.3f} m del obstáculo nuevo más cercano (bloqueo si < 0.25)')
    break
