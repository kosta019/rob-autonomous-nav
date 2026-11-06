# ajuste_esquina con los scans del final de cada ruta (robot parado en Q): mediana de 10 scans como el nodo
import math, sys, os, csv
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav'))
from autonomous_nav.fase1_node import ajuste_esquina
from autonomous_nav import comprobar_pose as cp
OFF = -0.032
XW, YW = 8.970 + 0.60, 14.497 - 0.50
mcap, log = sys.argv[1], sys.argv[2]
fin = [a for a in csv.DictReader(open(log)) if a['event'] == 'FASE I COMPLETADA'][0]
t0 = float(fin['timestamp']) + 0.5
x, y, th = float(fin['robot_x']), float(fin['robot_y']), float(fin['robot_yaw'])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=mcap, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
S = []
while r.has_next():
    _, d, t = r.read_next()
    if t0 < t * 1e-9 < t0 + 2.2:
        s = deserialize_message(d, LaserScan); s0 = s
        rr = np.array(s.ranges, float); rr[~np.isfinite(rr) | (rr < s.range_min) | (rr > s.range_max)] = np.nan; S.append(rr)
rr = cp.mediana_de_scans(S[:10]); a = s0.angle_min + np.arange(len(rr)) * s0.angle_increment
def aj(x, y, th):
    lx, ly = x + OFF * math.cos(th), y + OFF * math.sin(th); ok = ~np.isnan(rr)
    return ajuste_esquina(lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok]), x, y, XW, YW,
                          0.35, 1.0, 0.02, 15, 0.40, math.radians(12), math.radians(5), 'Y')
dx, dy, dth, m, d = aj(x, y, th)
f = lambda v: 'None' if v is None else f'{v:.3f}'
print(f'{len(S)} scans; creída ({x:.3f}, {y:.3f}, {math.degrees(th):.2f}°) -> [{m or "OK"}] dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}°'
      f"  nx {d['nx']} ny {d['ny']} rms {f(d['rms_x'])}/{f(d['rms_y'])} dth_x {math.degrees(d['dth_x'] or 0):+.2f} dth_y {math.degrees(d['dth_y'] or 0):+.2f}")
nx, ny, nth = x + dx, y + dy, th + dth
dx2, dy2, dth2, m2, _ = aj(nx, ny, nth)
print(f'   repetido desde la corregida: [{m2 or "OK"}] dx={dx2:+.3f} dy={dy2:+.3f} dth={math.degrees(dth2):+.2f}°;'
      f'  real según esto: ({nx:.3f}, {ny:.3f}); Q - real = ({8.970-nx:+.3f}, {14.497-ny:+.3f})')
