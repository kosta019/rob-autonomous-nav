# Offline: segundo a segundo en ESPERAR, lecturas "nuevas" (holgura del mapa > 0.20) a < 0.35 m de la ruta pendiente
# (los próximos 1.6 m) y a < 0.28 m de E (con eso la replanificación no puede llegar a E).
import math, sys, yaml, os
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
sh = get_package_share_directory('autonomous_nav')
hol = np.load(os.path.join(sh, 'maps/holgura_5cm.npy'))
meta = yaml.safe_load(open(os.path.join(sh, 'maps/holgura_5cm.yaml')))
x, y, th, off = 5.122, 5.114, 1.603, -0.032
E = (5.082, 6.122); F = (5.965, 11.655)
lx, ly = x + off * math.cos(th), y + off * math.sin(th)
L1 = math.hypot(E[0]-x, E[1]-y); rest = 1.6 - L1
ux, uy = (F[0]-E[0]) / math.hypot(F[0]-E[0], F[1]-E[1]), (F[1]-E[1]) / math.hypot(F[0]-E[0], F[1]-E[1])
ruta = [(x, y), E, (E[0] + rest*ux, E[1] + rest*uy)]
def dseg(px, py):
    d = np.full(px.shape, np.inf)
    for (ax, ay), (bx, by) in zip(ruta, ruta[1:]):
        vx, vy = bx-ax, by-ay
        t = np.clip(((px-ax)*vx + (py-ay)*vy) / (vx*vx + vy*vy), 0, 1)
        d = np.minimum(d, np.hypot(px-(ax+t*vx), py-(ay+t*vy)))
    return d
t0 = datetime.strptime('2026-11-05 14:58:05', '%Y-%m-%d %H:%M:%S').timestamp()
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
porseg = {}
while r.has_next():
    _, data, t = r.read_next()
    s = t * 1e-9 - t0
    if s < 0: continue
    if s > 68: break
    m = deserialize_message(data, LaserScan)
    a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
    rr = np.array(m.ranges)
    ok = np.isfinite(rr) & (rr >= m.range_min) & (rr <= 2.5)
    px, py = lx + rr[ok]*np.cos(th + a[ok]), ly + rr[ok]*np.sin(th + a[ok])
    fi = np.floor((px - meta['origen_x']) / meta['resolucion']).astype(int)
    co = np.floor((py - meta['origen_y']) / meta['resolucion']).astype(int)
    nuevo = hol[np.clip(fi, 0, hol.shape[0]-1), np.clip(co, 0, hol.shape[1]-1)] > 0.20
    d = dseg(px, py); dE = np.hypot(px - E[0], py - E[1])
    k = int(s)
    c = porseg.setdefault(k, [0, 0, None])
    c[0] += int(np.sum(nuevo & (d < 0.35))); c[1] += int(np.sum(nuevo & (dE < 0.28)))
    sel = nuevo & (d < 0.35)
    if sel.any() and c[2] is None: c[2] = (round(float(np.mean(px[sel])), 2), round(float(np.mean(py[sel])), 2))
for k in sorted(porseg):
    hh = datetime.fromtimestamp(t0 + k).strftime('%H:%M:%S')
    print(hh, f'cerca de la ruta: {porseg[k][0]:3d}   cerca de E: {porseg[k][1]:3d}', porseg[k][2] or '')
