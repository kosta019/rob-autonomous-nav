# Solo escucha /scan: lecturas < 0.25 m por ángulo durante N escaneos (y la más cercana en general)
import sys, collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
N = int(sys.argv[1])
datos = []
def cb(m):
    datos.append(m)
rclpy.init()
n = Node('vivo_cortas_claude')
n.create_subscription(LaserScan, '/scan', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(datos) < N:
    rclpy.spin_once(n, timeout_sec=1.0)
c = collections.Counter(); d = collections.defaultdict(list); it = collections.defaultdict(list); mins = []; escaneos_con = 0
for m in datos:
    rr = np.array(m.ranges); a = np.round(np.degrees(m.angle_min + np.arange(len(rr)) * m.angle_increment)).astype(int) % 360
    ok = np.isfinite(rr) & (rr >= m.range_min)
    mins.append((rr[ok].min(), int(a[ok][rr[ok].argmin()])))
    sel = np.where(ok & (rr < 0.25))[0]
    escaneos_con += len(sel) > 0
    for i in sel:
        c[int(a[i])] += 1; d[int(a[i])].append(float(rr[i]))
        if len(m.intensities): it[int(a[i])].append(float(m.intensities[i]))
print(f'{len(datos)} escaneos; con alguna lectura < 0.25 m: {escaneos_con} ({100*escaneos_con/len(datos):.0f} %)')
for k, v in c.most_common(10):
    print(f'  {k:3d}°: {v} veces, {min(d[k]):.3f}-{max(d[k]):.3f} m, intensidad mediana {np.median(it[k]) if it[k] else "-"}')
dm = min(mins)
print(f'lectura más cercana de todas: {dm[0]:.3f} m a {dm[1]}°; mediana del mínimo por escaneo {np.median([x[0] for x in mins]):.3f} m')
