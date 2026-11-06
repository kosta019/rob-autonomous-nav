# Repite, con los scans del bag y las poses del Mission Log, la memoria de obstáculos del nodo y sus decisiones
# (replanificar con punto sustituto y comprobar bloqueo) en cada evento del bucle junto a E (06/11 13:16-13:17).
import csv, math, sys, time
import numpy as np, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import os
PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'autonomous_nav')
sys.path.insert(0, PKG)
from autonomous_nav import planificador as pl
from autonomous_nav.fase1_node import punto_sustituto, rayos_libres

P = {}
for f in ('fase1.yaml', 'fase1_ruta.yaml', 'real.yaml'):
    P.update(yaml.safe_load(open(f'{PKG}/config/{f}'))['fase1_node']['ros__parameters'])
hol = np.load(f'{PKG}/{P["static_clearance_file"]}')
m = yaml.safe_load(open(f'{PKG}/{P["static_clearance_file"]}'.replace('.npy', '.yaml')))
meta = {'resolucion': m['resolucion'], 'origen_x': m['origen_x'], 'origen_y': m['origen_y']}
INF = [P['inflation_radius'], P['inflation_radius_retry'], P['inflation_radius_min']]
ruta = list(zip(P['waypoint_names'], P['waypoints_x'], P['waypoints_y']))
iE = [w[0] for w in ruta].index('E')
E = ruta[iE]
despues = [(w[1], w[2], w[0]) for w in ruta[iE + 1:]]

filas = list(csv.DictReader(open(sys.argv[1])))
T = np.array([float(a['timestamp']) for a in filas])
X = np.array([float(a['robot_x']) for a in filas]); Y = np.array([float(a['robot_y']) for a in filas])
TH = np.unwrap(np.array([float(a['robot_yaw']) for a in filas]))
eventos = [(float(a['timestamp']), a['event']) for a in filas
           if a['event'].startswith(('BLOQUEO', 'SIN_CAMINO', 'REPLANIFICACION'))]
t_ini, t_fin = eventos[0][0] - 10.0, eventos[-1][0] + 0.5

def pose(t):
    return np.interp(t, T, X), np.interp(t, T, Y), np.interp(t, T, TH)

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[2], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
scans = []
while r.has_next():
    _, data, t = r.read_next()
    if t_ini < t * 1e-9 < t_fin:
        scans.append((t * 1e-9, deserialize_message(data, LaserScan)))
scans.sort(key=lambda s: s[0])

mem = {}
def actualizar(t, s):
    x, y, th = pose(t)
    w = (pose(t + 0.25)[2] - pose(t - 0.25)[2]) / 0.5
    if abs(w) > P['obstacle_max_turn_rate']:
        return
    rr = np.array(s.ranges, dtype=float); n = len(rr)
    a = s.angle_min + np.arange(n) * s.angle_increment
    lx, ly = x + P['lidar_offset_x'] * math.cos(th), y + P['lidar_offset_x'] * math.sin(th)
    ok = np.isfinite(rr) & (rr >= max(s.range_min, P['obstacle_min_range'])) & (rr <= min(s.range_max, P['obstacle_max_range']))
    for ox, oy in zip(lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])):
        i, j = pl.mundo_a_celda(ox, oy, meta)
        if 0 <= i < hol.shape[0] and 0 <= j < hol.shape[1] and hol[i, j] > P['obstacle_static_margin']:
            mem[(i, j)] = [ox, oy, t, 0]
    for c, ob in list(mem.items()):
        if t - ob[2] > P['obstacle_memory_time']:
            del mem[c]; continue
        d = math.hypot(ob[0] - lx, ob[1] - ly)
        if not (s.range_min < d < s.range_max):
            continue
        k = int(round(((math.atan2(ob[1] - ly, ob[0] - lx) - th - s.angle_min) % (2 * math.pi)) / s.angle_increment))
        if rayos_libres([rr[(k + dk) % n] for dk in (-1, 0, 1)], d, P['obstacle_clear_margin'], P['obstacle_clear_zero_min_dist']):
            ob[3] += 1
            if ob[3] >= P['obstacle_clear_scans']:
                del mem[c]
        else:
            ob[3] = 0

def bloqueo(x, y, puntos):
    O = np.array([(v[0], v[1]) for v in mem.values()])
    restante, peor = P['block_lookahead'], None
    for (ax, ay, na), (bx, by, nb) in zip([(x, y, 'robot')] + puntos, puntos):
        largo = math.hypot(bx - ax, by - ay)
        if largo == 0: continue
        usado = min(largo, restante)
        tt = np.linspace(0, usado / largo, int(math.ceil(usado / 0.05)) + 1)
        smp = np.column_stack([ax + tt * (bx - ax), ay + tt * (by - ay)])
        d = np.hypot(smp[:, None, 0] - O[None, :, 0], smp[:, None, 1] - O[None, :, 1])
        k = np.unravel_index(d.argmin(), d.shape)
        if peor is None or d.min() < peor[0]:
            peor = (d.min(), f'{na}->{nb}', O[k[1], 0], O[k[1], 1], usado)
        restante -= usado
        if restante <= 0: break
    return peor

sust = None          # (x, y) del punto sustituto, fijo desde que se elige (como en el nodo)
plan = None
i_s = 0
hora = lambda t: time.strftime('%H:%M:%S', time.localtime(t))
for te, ev in eventos:
    while i_s < len(scans) and scans[i_s][0] <= te:
        actualizar(*scans[i_s]); i_s += 1
    x, y, _ = pose(te)
    O = [(v[0], v[1]) for v in mem.values()]
    h = pl.holgura_con_obstaculos(hol, meta, O, P['obstacle_plan_margin'], P['clearance_target'])
    if ev.startswith('REPLANIFICACION'):
        primera = sust is None
        hecho = None
        for inf in INF:
            if primera:
                if pl.holgura_en(h, meta, E[1], E[2]) <= inf:
                    s = punto_sustituto(h, meta, E[1], E[2], inf, P['mandatory_cover_radius'])
                    if s is None: continue
                else:
                    s = (E[1], E[2])
            else:
                s = sust
                if pl.holgura_en(h, meta, s[0], s[1]) <= inf: continue
            celdas = pl.planificar_tramo(pl.mapa_de_costes(h, inf, P['clearance_target'], P['clearance_cost_gain']), meta, (x, y), s)
            if celdas is None: continue
            hecho = (inf, s, pl.simplificar(celdas, h, meta, P['simplify_step'], P['simplify_clearance_cap'], P['simplify_margin']))
            break
        if hecho is None:
            print(f'{hora(te)} {ev:28s} | reconstrucción: SIN camino'); continue
        inf, s, pts = hecho
        if primera: sust = s
        plan = [(p[0], p[1], 'replan') for p in pts[1:-1]] + [(s[0], s[1], 'E~')] + despues
        print(f'{hora(te)} {ev:28s} | reconstr.: inflado {inf:.2f}, E~ ({s[0]:.3f}, {s[1]:.3f}) '
              f'holgura E~ {pl.holgura_en(h, meta, s[0], s[1]):.3f}, robot a {math.hypot(s[0]-x, s[1]-y):.2f} m de E~')
    elif ev == 'BLOQUEO' and plan is not None:
        d, tramo, ox, oy, usado = bloqueo(x, y, plan)
        print(f'{hora(te)} {ev:28s} | más cerca: {d:.3f} m en el tramo {tramo} ({usado:.2f} m mirados), obstáculo ({ox:.3f}, {oy:.3f})'
              f' {len(mem)} celdas en memoria')
    else:
        print(f'{hora(te)} {ev:28s} | holgura en E {pl.holgura_en(h, meta, E[1], E[2]):.3f}, {len(mem)} celdas en memoria')
