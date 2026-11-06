# Detector de pilares por scan (prototipo): segmentos del scan separados por saltos, de 2-8 puntos y ancho <= 9 cm,
# con huecos a los lados (algo aislado delante de lo que hay detrás). Centro = media + radio hacia fuera.
import math, sys, csv, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
def pilares(r, a, rmax=1.6, salto=0.10, ancho_max=0.09):
    ok=np.isfinite(r)&(r>0.12)&(r<rmax)
    out=[]; n=len(r); i=0
    idx=np.where(ok)[0]
    if len(idx)==0: return out
    # segmentos de rayos consecutivos con |dr| < salto
    segs=[]; cur=[idx[0]]
    for k in idx[1:]:
        if k==cur[-1]+1 and abs(r[k]-r[cur[-1]])<salto: cur.append(k)
        else: segs.append(cur); cur=[k]
    segs.append(cur)
    for s in segs:
        if not (1<=len(s)<=8): continue
        x=r[s]*np.cos(a[s]); y=r[s]*np.sin(a[s])
        w=math.hypot(x[-1]-x[0], y[-1]-y[0])
        if w>ancho_max: continue
        # aislado: los vecinos a los lados, más lejos (o sin dato)
        izq=r[(s[0]-1)%n]; der=r[(s[-1]+1)%n]; rm=np.mean(r[s])
        if not ((not np.isfinite(izq) or izq>rm+salto) and (not np.isfinite(der) or der>rm+salto)): continue
        cx,cy=np.mean(x),np.mean(y); d=math.hypot(cx,cy)
        cx,cy=cx*(d+0.025)/d, cy*(d+0.025)/d       # del borde visto al centro de la pata (radio 2.5 cm)
        out.append((cx,cy,len(s),w))
    return out
if __name__=='__main__':
    mcap,log=sys.argv[1],sys.argv[2]
    f=[a for a in csv.DictReader(open(log)) if a['robot_x']]
    T=np.array([float(a['timestamp']) for a in f]); X=np.array([float(a['robot_x']) for a in f]); Y=np.array([float(a['robot_y']) for a in f]); TH=np.unwrap([float(a['robot_yaw']) for a in f])
    rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=mcap,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/scan'])); det=[]
    while rd.has_next():
        _,d,t=rd.read_next(); t*=1e-9
        if t<T[0] or t>T[-1]: continue
        s=deserialize_message(d,LaserScan); r=np.array(s.ranges,float); a=s.angle_min+np.arange(len(r))*s.angle_increment
        x,y,th=np.interp(t,T,X),np.interp(t,T,Y),np.interp(t,T,TH)
        lx,ly=x-0.032*math.cos(th),y-0.032*math.sin(th)
        for cx,cy,n,w in pilares(r,a):
            det.append((t, lx+cx*math.cos(th)-cy*math.sin(th), ly+cx*math.sin(th)+cy*math.cos(th), n, w))
    D=np.array(det); print(len(D),'detecciones')
    # agrupar en el mundo (5 cm)
    grupos=[]
    for t,x,y,n,w in D:
        for g in grupos:
            if math.hypot(g['x']-x,g['y']-y)<0.07: g['p'].append((x,y)); g['x'],g['y']=np.mean(g['p'],0); break
        else: grupos.append({'x':x,'y':y,'p':[(x,y)]})
    G=sorted([g for g in grupos if len(g['p'])>=5],key=lambda g:-len(g['p']))
    for g in G[:15]: print(f"  pilar ({g['x']:.3f},{g['y']:.3f})  {len(g['p'])} detecciones")
