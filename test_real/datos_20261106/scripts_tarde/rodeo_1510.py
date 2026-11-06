# 06/11 15:10:38: replan desde (5.125, 6.943) hasta Door con los obstáculos del LiDAR (mediana de 5 scans), mapa viejo y nuevo
import math, sys, os, csv
import numpy as np, yaml, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
PKG = os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav'); sys.path.insert(0, PKG)
from autonomous_nav import planificador as pl
from autonomous_nav.fase1_node import longitud_camino, rodeo_largo
from autonomous_nav import comprobar_pose as cp
P = {}
for f in ('fase1.yaml', 'fase1_ruta.yaml', 'real.yaml'):
    P.update(yaml.safe_load(open(f'{PKG}/config/{f}'))['fase1_node']['ros__parameters'])
m = yaml.safe_load(open(f'{PKG}/maps/holgura_5cm.yaml')); meta = {k: m[k] for k in ('resolucion', 'origen_x', 'origen_y')}
fil = list(csv.DictReader(open(sys.argv[2])))
ev = [a for a in fil if a['event'].startswith('REPLANIFICACION')][1]     # la de las 15:10:38
t, x, y, th = float(ev['timestamp']), float(ev['robot_x']), float(ev['robot_y']), float(ev['robot_yaw'])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan'])); S = []
while r.has_next():
    _, d, tt = r.read_next()
    if t - 1.2 < tt * 1e-9 < t:
        s = deserialize_message(d, LaserScan); s0 = s; rr = np.array(s.ranges, float)
        rr[~np.isfinite(rr) | (rr < 0.12) | (rr > P['obstacle_max_range'])] = np.nan; S.append(rr)
rr = cp.mediana_de_scans(S); a = s0.angle_min + np.arange(len(rr)) * s0.angle_increment; ok = ~np.isnan(rr)
lx, ly = x + P['lidar_offset_x'] * math.cos(th), y + P['lidar_offset_x'] * math.sin(th)
px, py = lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])
resto = [(o, xx, yy) for o, xx, yy in zip(P['waypoint_names'], P['waypoints_x'], P['waypoints_y'])]
iD = [o[0] for o in resto].index('Door'); door = resto[iD]
# lo que quedaba con la ruta vieja: E-Door_1 viejo (5.115, 7.005) -> E-Door_2 (5.915, 11.455) -> Door
antes = longitud_camino([(x, y), (5.115, 7.005), (5.915, 11.455), (door[1], door[2])])
print(f'pose ({x:.3f}, {y:.3f}), {len(S)} scans; quedaban {antes:.2f} m hasta Door')
for nombre, fich in (('VIEJO', sys.argv[3]), ('NUEVO', f'{PKG}/maps/holgura_5cm.npy')):
    hol = np.load(fich)
    obst = []
    for ox, oy in zip(px, py):
        i, j = pl.mundo_a_celda(ox, oy, meta)
        if 0 <= i < hol.shape[0] and 0 <= j < hol.shape[1] and hol[i, j] > P['obstacle_static_margin']:
            obst.append((ox, oy))
    h = pl.holgura_con_obstaculos(hol, meta, obst, P['obstacle_plan_margin'], P['clearance_target'])
    for inf in (P['inflation_radius'], P['inflation_radius_retry'], P['inflation_radius_min']):
        c = pl.planificar_tramo(pl.mapa_de_costes(h, inf, P['clearance_target'], P['clearance_cost_gain']), meta, (x, y), (door[1], door[2]))
        if c is None:
            print(f'  {nombre} inflado {inf:.2f}: sin camino'); continue
        pts = pl.simplificar(c, h, meta, P['simplify_step'], P['simplify_clearance_cap'], P['simplify_margin'])
        L = longitud_camino(pts)
        print(f'  {nombre} inflado {inf:.2f}: {L:.2f} m {"RODEO_LARGO (espera)" if rodeo_largo(L, antes, P["replan_max_ratio"], P["replan_max_extra"]) else "aceptado"}'
              f'  ({len(obst)} obstáculos)  X mín {min(p[0] for p in pts):.2f} Y mín {min(p[1] for p in pts):.2f}')
        break
