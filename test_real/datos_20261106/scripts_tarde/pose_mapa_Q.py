# Pose del robot en Q según el mapa: ajuste_mapa iterado sin confianza en la odometría, con los scans del final (parado)
import math, sys, os, csv
import numpy as np, yaml, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
PKG = os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav'); sys.path.insert(0, PKG)
from autonomous_nav.fase1_node import ajuste_mapa
P = {}
for f in ('fase1.yaml', 'fase1_ruta.yaml', 'real.yaml'):
    P.update(yaml.safe_load(open(f'{PKG}/config/{f}'))['fase1_node']['ros__parameters'])
mapa = np.load(f'{PKG}/{P["map_corr_file"]}'); meta = yaml.safe_load(open(f'{PKG}/{P["map_corr_file"]}'.replace('.npy', '.yaml')))
mcap, log = sys.argv[1], sys.argv[2]
fil = list(csv.DictReader(open(log)))
fin = [a for a in fil if a['event'] == 'FASE I COMPLETADA'][0]
t0 = float(fin['timestamp']) + 1.0
x0, y0 = float(fin['robot_x']), float(fin['robot_y'])
# rumbo: el de la fila más cercana a cada scan (tras FIN solo cambia si lo giran a mano)
T = np.array([float(a['timestamp']) for a in fil]); TH = np.array([float(a['robot_yaw']) for a in fil])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=mcap, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
scans = []
while r.has_next():
    _, data, t = r.read_next(); t *= 1e-9
    if t0 < t < t0 + 6.0:
        scans.append((t, deserialize_message(data, LaserScan)))
print(len(scans), 'scans; creída', x0, y0, f'{math.degrees(TH[np.searchsorted(T, t0)]):.2f}°')
res = []
for t, s in scans:
    th0 = TH[min(np.searchsorted(T, t), len(TH) - 1)]
    rr = np.array(s.ranges, float); a = s.angle_min + np.arange(len(rr)) * s.angle_increment
    ok = np.isfinite(rr) & (rr > s.range_min) & (rr < min(s.range_max, 5.0))
    x, y, th = x0, y0, th0
    for _ in range(40):
        dx, dy, dth, n = ajuste_mapa(rr[ok], a[ok], x, y, th, P['lidar_offset_x'], mapa, meta, float(sys.argv[3]) if len(sys.argv) > 3 else 0.30,
                                     P['map_corr_sigma_range'], 1.0, math.radians(20), 15)
        x, y, th = x + dx, y + dy, th + dth
    res.append((x - x0, y - y0, math.degrees(th - th0), n))
R = np.array(res)
print('pose según el mapa - creída: dx %.3f dy %.3f dth %.2f°  (desv. %.3f %.3f %.2f)  n %d' % (*np.median(R[:, :3], 0), *R[:, :3].std(0), np.median(R[:, 3])))
