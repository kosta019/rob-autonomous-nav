# Herramienta de montaje: con el robot parado, estima con el LiDAR su pose real (rumbo y posición) comparando
# el /scan con las paredes del mapa, y dice cuánto se desvía de initial_pose. No publica nada.
# Uso: ros2 run autonomous_nav comprobar_pose config/fase1.yaml [config/real.yaml ...] [--escribir]
# Los .yaml son los mismos que se pasan a fase1_node (cada uno sobrescribe al anterior); de ellos salen
# initial_pose y lidar_offset_x. Las rutas son relativas a la carpeta actual.
# Misma estructura que ref/go_stop.py: una clase Node que se suscribe a /scan, y main().
import math
import os
import sys
import warnings
from datetime import datetime

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

N_SCANS = 25                    # scans que se juntan (se toma la mediana de cada rayo)
COMPUERTAS = [0.30, 0.10, 0.05]  # m: |medido - esperado| máximo para ser inlier, una por iteración
CARA_MIN = 0.30                 # m: se ignoran las caras de pared más cortas
ESQUINA = 0.05                  # m: se descartan los impactos a menos de esto del extremo de una cara
PUNTOS_MIN = 15                 # puntos mínimos de una cara para usarla
INLIERS_MIN = 30                # por debajo, aviso
RMS_MAX = 0.02                  # m, aviso
CORR_MAX_POS = 0.15             # m, aviso
CORR_MAX_ANG = 5.0              # grados, aviso
ECOS_MAX = 0.05                 # fracción de scans con lecturas a menos de obstacle_min_range: aviso (ventana sucia)
GIRO_W_MAX = 0.01               # rad/s: media de |w| de /imu con el robot quieto por encima de esto, aviso (giróscopo)
GIRO_DERIVA_MAX = 0.2           # grados: cambio del rumbo de /odom en la ventana por encima de esto, aviso


def normalizar(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_cuaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ComprobarPose(Node):
    def __init__(self):
        super().__init__('comprobar_pose')
        self.scans = []
        self.info = None   # (angle_min, angle_increment, range_min, range_max)
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)
        # Mientras se juntan los scans se guardan también /imu y /odom, que con el robot quieto no deben marcar giro
        self.wz = []      # velocidad angular z de /imu
        self.yaws = []    # rumbo de /odom
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_b)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_b)

    def scan_callback(self, msg):
        if len(self.scans) >= N_SCANS:
            return
        if self.info is None:
            self.info = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        r = np.array(msg.ranges, dtype=float)
        # Sin dato: 0, inf, nan o fuera de [range_min, range_max]
        r[~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)] = np.nan
        self.scans.append(r)

    def imu_callback(self, msg):
        if len(self.scans) < N_SCANS:
            self.wz.append(msg.angular_velocity.z)

    def odom_callback(self, msg):
        if len(self.scans) < N_SCANS:
            self.yaws.append(yaw_de_cuaternion(msg.pose.pose.orientation))


def aviso_giroscopo(wz, yaws):
    # Con el robot quieto, /imu y el rumbo de /odom (que sale del IMU) no deben marcar giro. Si el robot se encendió
    # moviéndose, el giróscopo se calibra mal y marca un giro falso. Devuelve el texto del aviso, o None si va bien.
    if not wz or not yaws:
        return 'no llega /imu o /odom: no puedo comprobar el giróscopo'
    w_media = float(np.mean(np.abs(wz)))
    deriva = math.degrees(float(np.ptp(np.unwrap(yaws))))   # unwrap: pasar de -180 a 180 no es girar
    if w_media > GIRO_W_MAX or deriva > GIRO_DERIVA_MAX:
        return (f'/imu marca {w_media:.3f} rad/s de media y el rumbo de /odom cambia {deriva:.2f} grados '
                f'con el robot quieto: reinicia la OpenCR con el robot quieto en el suelo')
    return None


def fraccion_con_ecos(scans, dist):
    # Fracción de scans con alguna lectura válida a menos de dist del LiDAR. Con el robot quieto y nada alrededor,
    # solo pueden ser ecos del propio robot (la ventana del LiDAR sucia).
    if not scans:
        return 0.0
    with np.errstate(invalid='ignore'):
        return sum(bool(np.any(s < dist)) for s in scans) / len(scans)


def mediana_de_scans(scans):
    # Mediana de cada rayo. Un rayo es válido si al menos la mitad de los scans lo ven.
    m = np.array(scans)
    validos = np.sum(~np.isnan(m), axis=0) >= len(scans) / 2
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')   # columnas todo nan
        r = np.nanmedian(m, axis=0)
    r[~validos] = np.nan
    return r


def cargar_parametros(rutas):
    # Junta los ros__parameters de fase1_node de cada fichero, en orden
    p = {}
    for ruta in rutas:
        with open(ruta) as f:
            p.update(yaml.safe_load(f)['fase1_node']['ros__parameters'])
    return p


def cargar_caras():
    # Cuatro caras por rectángulo: eje 0 es una cara X (x = c, y en [lo, hi]) y eje 1 una cara Y (y = c, x en [lo, hi])
    ruta = os.path.join(get_package_share_directory('autonomous_nav'), 'maps', 'paredes.yaml')
    with open(ruta) as f:
        w = yaml.safe_load(f)
    x0, x1, y0, y1 = (np.array(w[k]) for k in ('x_min', 'x_max', 'y_min', 'y_max'))
    eje = np.concatenate([np.zeros(2 * len(x0)), np.ones(2 * len(x0))]).astype(int)
    c = np.concatenate([x0, x1, y0, y1])
    lo = np.concatenate([y0, y0, x0, x0])
    hi = np.concatenate([y1, y1, x1, x1])
    largas = (hi - lo) >= CARA_MIN
    return {'eje': eje[largas], 'c': c[largas], 'lo': lo[largas], 'hi': hi[largas]}


def trazar(L, ang, caras):
    # Para cada rayo (ángulo en el mundo, desde L) calcula la distancia esperada y la cara que golpea (-1 si ninguna
    # o si cae a menos de ESQUINA del extremo de la cara).
    dx = np.cos(ang)[:, None]
    dy = np.sin(ang)[:, None]
    x_es_perp = (caras['eje'] == 0)[None, :]
    d_perp = np.where(x_es_perp, dx, dy)     # componente del rayo perpendicular a la cara
    d_along = np.where(x_es_perp, dy, dx)    # componente a lo largo de la cara
    L_perp = np.where(x_es_perp, L[0], L[1])
    L_along = np.where(x_es_perp, L[1], L[0])
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (caras['c'][None, :] - L_perp) / d_perp
        s = L_along + t * d_along            # punto de impacto a lo largo de la cara
    ok = (t > 0) & (s >= caras['lo'][None, :]) & (s <= caras['hi'][None, :])
    t = np.where(ok, t, np.inf)
    k = np.arange(len(ang))
    cara = np.argmin(t, axis=1)
    dist = t[k, cara]
    en_esquina = (s[k, cara] < caras['lo'][cara] + ESQUINA) | (s[k, cara] > caras['hi'][cara] - ESQUINA)
    malo = ~np.isfinite(dist) | en_esquina
    cara[malo] = -1
    dist[malo] = np.nan
    return dist, cara


def ajustar(x, y, th, off, p, cara, ok, caras, eje_rumbo=''):
    # Calcula con los inliers el nuevo rumbo (recta de cada cara) y la nueva posición. p: puntos en el sistema del LiDAR.
    # eje_rumbo: el rumbo sale solo de las caras de ese eje ('Y' son las rectas y = c; '' = todas), porque las paredes
    # de la sala no son perpendiculares entre sí.
    usadas = [f for f in np.unique(cara[ok]) if np.sum(ok & (cara == f)) >= PUNTOS_MIN]
    para_rumbo = [f for f in usadas if not eje_rumbo or 'XY'[int(caras['eje'][f])] == eje_rumbo]
    difs, pesos = [], []
    for f in para_rumbo:
        pts = p[ok & (cara == f)]
        _, vec = np.linalg.eigh(np.cov(pts.T))
        phi = math.atan2(vec[1, 1], vec[0, 1])         # dirección de la recta (autovector mayor) en el LiDAR
        direccion = math.pi / 2 if caras['eje'][f] == 0 else 0.0
        d = normalizar(direccion - phi - th)
        difs.append(d - math.pi * round(d / math.pi))  # la recta no tiene sentido: módulo pi
        pesos.append(len(pts))
    if para_rumbo:
        th = th + float(np.average(difs, weights=pesos))

    # Posición del LiDAR: para cada cara, c - media de los puntos girados al mundo
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    q = p @ R.T
    L = [x + off * math.cos(th), y + off * math.sin(th)]
    valores = {0: [], 1: []}
    for f in usadas:
        e = int(caras['eje'][f])
        sel = ok & (cara == f)
        valores[e].append((caras['c'][f] - np.mean(q[sel, e]), np.sum(sel)))
    for e in (0, 1):
        if valores[e]:
            L[e] = float(np.average([v for v, _ in valores[e]], weights=[n for _, n in valores[e]]))
    x = L[0] - off * math.cos(th)
    y = L[1] - off * math.sin(th)

    # Resumen por cara: eje, c, puntos, residuo RMS, distancia medida y esperada
    resumen = []
    for f in usadas:
        e = int(caras['eje'][f])
        sel = ok & (cara == f)
        res = caras['c'][f] - (L[e] + q[sel, e])
        resumen.append({'eje': 'XY'[e], 'c': float(caras['c'][f]), 'n': int(np.sum(sel)),
                        'rms': float(np.sqrt(np.mean(res ** 2))), 'f': f, 'sel': sel})
    return x, y, th, resumen


def estimar(pose, off, r, info, caras, eje_rumbo=''):
    # Tres iteraciones con una compuerta cada vez más estrecha; cada una traza los rayos desde la pose nueva
    angulo_min, inc, _, _ = info
    a = angulo_min + np.arange(len(r)) * inc            # ángulo de cada rayo en el sistema del LiDAR
    valido = ~np.isnan(r)
    rr = np.where(valido, r, 0.0)
    p = np.stack([rr * np.cos(a), rr * np.sin(a)], axis=1)
    x, y, th = pose
    for compuerta in COMPUERTAS:
        L = (x + off * math.cos(th), y + off * math.sin(th))
        esperada, cara = trazar(L, th + a, caras)
        with np.errstate(invalid='ignore'):
            ok = valido & (cara >= 0) & (np.abs(r - esperada) < compuerta)
        x, y, th, resumen = ajustar(x, y, th, off, p, cara, ok, caras, eje_rumbo)
    # Distancia esperada con la pose final, para el resumen
    L = (x + off * math.cos(th), y + off * math.sin(th))
    esperada, _ = trazar(L, th + a, caras)
    for c in resumen:
        c['medida'] = float(np.mean(r[c['sel']]))
        c['esperada'] = float(np.nanmean(esperada[c['sel']]))
    return x, y, th, resumen


def texto_mov(dx, dy, th):
    # Un desplazamiento (dx, dy) del mundo, dicho también en ejes del robot
    delante = dx * math.cos(th) + dy * math.sin(th)
    izquierda = -dx * math.sin(th) + dy * math.cos(th)
    return (f'{abs(dx) * 100:.1f} cm hacia {"+X" if dx >= 0 else "-X"} y {abs(dy) * 100:.1f} cm hacia '
            f'{"+Y" if dy >= 0 else "-Y"} ({abs(delante) * 100:.1f} cm hacia {"delante" if delante >= 0 else "atrás"}, '
            f'{abs(izquierda) * 100:.1f} cm a su {"izquierda" if izquierda >= 0 else "derecha"})')


def informar(pose_ini, pose, resumen):
    xi, yi, ti = pose_ini
    x, y, th = pose
    dth = math.degrees(normalizar(th - ti))
    dx, dy = x - xi, y - yi
    print('\nCaras de pared usadas:')
    for c in resumen:
        print(f'  eje {c["eje"]}  c = {c["c"]:.3f} m  {c["n"]:3d} puntos  residuo RMS {c["rms"] * 100:.2f} cm  '
              f'distancia del LiDAR medida {c["medida"]:.3f} m, esperada {c["esperada"]:.3f} m')
    print(f'\nPose inicial (initial_pose): {xi:.3f} {yi:.3f} {ti:.4f}')
    print(f'Pose estimada: x = {x:.3f} m, y = {y:.3f} m, theta = {th:.4f} rad ({math.degrees(th):.2f} grados)')
    if abs(dth) < 0.05:
        print('Rumbo: coincide con initial_pose.')
    elif dth > 0:
        print(f'El robot está girado {dth:.1f} grados a la izquierda de initial_pose: '
              f'gíralo {dth:.1f} grados a la derecha.')
    else:
        print(f'El robot está girado {-dth:.1f} grados a la derecha de initial_pose: '
              f'gíralo {-dth:.1f} grados a la izquierda.')
    print(f'Está {texto_mov(dx, dy, ti)} respecto a initial_pose:')
    print(f'  muévelo {texto_mov(-dx, -dy, ti)}.')
    print('Alternativa recomendada: no mover el robot y usar el fichero corregido (--escribir).')
    print(f'POSE_ESTIMADA {x:.4f} {y:.4f} {th:.5f}')

    avisos = []
    if sum(c['n'] for c in resumen) < INLIERS_MIN:
        avisos.append(f'hay menos de {INLIERS_MIN} puntos de pared en total')
    if len({c['eje'] for c in resumen}) < 2:
        avisos.append('solo hay paredes de un eje: la otra coordenada no se ha corregido')
    if any(c['rms'] > RMS_MAX for c in resumen):
        avisos.append(f'alguna cara tiene un residuo RMS mayor de {RMS_MAX * 100:.0f} cm')
    if math.hypot(dx, dy) > CORR_MAX_POS or abs(dth) > CORR_MAX_ANG:
        avisos.append('la corrección es muy grande: revisa initial_pose o el mapa en esta zona')
    for a in avisos:
        print(f'AVISO: {a}')
    return avisos


def escribir_yaml(pose):
    with open('initial_pose_corregida.yaml', 'w') as f:
        f.write(f'# Generado por comprobar_pose el {datetime.now():%Y-%m-%d %H:%M}.\n')
        f.write('# Pásalo como ÚLTIMO --params-file para que sobrescriba initial_pose.\n')
        f.write('fase1_node:\n  ros__parameters:\n')
        f.write(f'    initial_pose: [{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.5f}]\n')
    print('Escrito initial_pose_corregida.yaml')


def main(args=None):
    # Los argumentos que no son de ROS son los .yaml de parámetros (y --escribir)
    resto = remove_ros_args(args=sys.argv)[1:]
    escribir = '--escribir' in resto
    rutas = [a for a in resto if a != '--escribir']
    if not rutas:
        print('Uso: comprobar_pose config/fase1.yaml [config/real.yaml ...] [--escribir]')
        return
    p = cargar_parametros(rutas)
    pose_ini = list(p['initial_pose'])
    off = p['lidar_offset_x']
    caras = cargar_caras()

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ComprobarPose()
    try:
        node.get_logger().info(f'Esperando {N_SCANS} scans de /scan...')
        while rclpy.ok() and len(node.scans) < N_SCANS:
            rclpy.spin_once(node, timeout_sec=1.0)
        if len(node.scans) < N_SCANS:
            return
        r = mediana_de_scans(node.scans)
        x, y, th, resumen = estimar(pose_ini, off, r, node.info, caras, p.get('pose_heading_axis', ''))
        # Las paredes dan el rumbo del LiDAR. Si está montado girado (lidar_yaw_offset), el rumbo del robot es ese
        # menos el desfase. x e y no cambian: el error es menor de 1 mm.
        desfase = p.get('lidar_yaw_offset', 0.0)
        if desfase != 0.0:
            print(f'Rumbo del LiDAR {math.degrees(th):.2f} grados; restando lidar_yaw_offset '
                  f'({math.degrees(desfase):.2f} grados), rumbo del robot {math.degrees(th - desfase):.2f} grados')
            th = th - desfase
        avisos = informar(pose_ini, (x, y, th), resumen)
        ecos = fraccion_con_ecos(node.scans, p['obstacle_min_range'])
        if ecos > ECOS_MAX:
            print(f'AVISO: el {ecos * 100:.0f} % de los scans tiene lecturas a menos de '
                  f"{p['obstacle_min_range']:.2f} m: limpia la ventana del LiDAR (o aparta lo que haya al lado)")
            avisos.append('ecos cercanos')
        giro = aviso_giroscopo(node.wz, node.yaws)
        if giro:
            print(f'AVISO: {giro}')
            avisos.append('giróscopo')
        if escribir and avisos:
            # Con avisos la estimación no es fiable, y es mejor no dejar un fichero que se use sin mirar
            print('No escribo initial_pose_corregida.yaml: hay avisos. Revisa la colocación y repite.')
        elif escribir:
            escribir_yaml((x, y, th))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
