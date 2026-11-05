# Offline: lecturas cortas (< 0.45 m) por bag: cuántas en el rayo de 146°, sus distancias e intensidades,
# y si el robot se movía (|v| o |w| de /odom) en ese momento.
import sys, collections
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rosbag2_py
for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/odom']))
    n = 0; moviendo = False; dist = []; inten = []; mov = collections.Counter(); horas = []; otros = collections.Counter()
    while r.has_next():
        top, data, t = r.read_next()
        if top == '/odom':
            o = deserialize_message(data, Odometry)
            moviendo = abs(o.twist.twist.linear.x) > 0.005 or abs(o.twist.twist.angular.z) > 0.05
            continue
        m = deserialize_message(data, LaserScan); n += 1
        rr = np.array(m.ranges); a = np.round(np.degrees(m.angle_min + np.arange(len(rr)) * m.angle_increment)).astype(int) % 360
        ok = np.isfinite(rr) & (rr >= m.range_min) & (rr < 0.45)
        for i in np.where(ok)[0]:
            if a[i] in (145, 146, 147):
                dist.append(rr[i]); mov[moviendo] += 1; horas.append(t)
                if len(m.intensities): inten.append(m.intensities[i])
            else:
                otros[a[i]] += 1
    nombre = bag.split('/')[-1]
    if dist:
        h0 = datetime.fromtimestamp(min(horas) / 1e9).strftime('%H:%M:%S'); h1 = datetime.fromtimestamp(max(horas) / 1e9).strftime('%H:%M:%S')
        print(f'{nombre}: {n} scans, rayo 145-147°: {len(dist)} veces ({h0}..{h1}), dist {min(dist):.3f}-{max(dist):.3f} m (mediana {np.median(dist):.3f}), '
              f'intensidad {np.median(inten) if inten else "-"}; con robot moviéndose {mov[True]}, parado {mov[False]}')
    else:
        print(f'{nombre}: {n} scans, rayo 145-147°: nunca')
    print('   otros ángulos con lecturas < 0.45 m (los 6 más frecuentes):', otros.most_common(6))
