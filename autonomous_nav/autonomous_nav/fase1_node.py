# Fase I: seguir la lista de waypoints con parada de emergencia y Mission Log (paso 1), y
# detectar obstáculos nuevos con el LiDAR y replanificar alrededor de ellos (paso 3).
# Misma estructura que ref/go_stop.py: una clase Node, publicadores y suscriptores en __init__,
# callbacks y main(). La diferencia: el control va en un timer a ritmo fijo (así se detecta
# si un sensor deja de llegar) y no en los callbacks.
import csv
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

from autonomous_nav import comprobar_pose as cp
from autonomous_nav import planificador as pl

BLOQUEO_ACERCARSE = 0.005   # m: dentro del inflado, la ruta bloquea si se acerca al obstáculo más que esto (ruta_bloqueada)


def normalizar(a):
    # Ángulo en (-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_cuaternion(q):
    # Yaw sin tf_transformations (en Jazzy da problemas)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def limitar(valor, maximo):
    return max(-maximo, min(maximo, valor))


def rayos_libres(rayos, d, margen, dist_min_cero):
    # ¿Dicen los rayos que apuntan a un obstáculo recordado (a distancia d del LiDAR) que ya no está?
    # Un rayo es "libre" si da inf, una distancia mayor que d + margen, o 0.0 con el obstáculo lejos.
    # El LDS real da 0.0 cuando el rayo no choca con nada hasta range_max (Gazebo da inf), pero también
    # cuando algo está más cerca que range_min: por eso un 0.0 solo vale si d > dist_min_cero.
    for r in rayos:
        if r == math.inf or (math.isfinite(r) and r > d + margen) or (r == 0.0 and d > dist_min_cero):
            continue
        return False
    return True


def punto_sustituto(holgura, meta, ox, oy, inflado, radio):
    # Punto obligatorio (ox, oy) tapado por un obstáculo: la celda libre (holgura > inflado) más cercana a él, a menos
    # de 'radio'. Devuelve su centro (x, y) o None si no hay ninguna.
    res = meta['resolucion']
    i0, j0 = pl.mundo_a_celda(ox, oy, meta)
    r = int(math.ceil(radio / res))
    mejor = None
    for i in range(max(0, i0 - r), min(holgura.shape[0], i0 + r + 1)):
        for j in range(max(0, j0 - r), min(holgura.shape[1], j0 + r + 1)):
            cx, cy = pl.celda_a_mundo(i, j, meta)
            d = math.hypot(cx - ox, cy - oy)
            if d <= radio and holgura[i, j] > inflado and (mejor is None or d < mejor[0]):
                mejor = (d, cx, cy)
    return None if mejor is None else (mejor[1], mejor[2])


def ajuste_puerta(px, py, rx, ry, x_pared, y_borde, ancho, tol_ancho, banda, largo, max_rms, min_puntos,
                  min_cercano, max_dx, max_dy, max_ang, fondo_marco, y_fin_canaleta=None):
    # Relocalización con la pared de la puerta (lado sala), con el robot parado. px, py: puntos del scan pasados al
    # mundo con la pose CREÍDA; (rx, ry): posición creída del robot. Referencias CALIBRADAS con el robot en una pose
    # medida con cinta: x_pared = X a la que el LiDAR ve la pared (la canaleta) e y_borde = Y del lateral del marco
    # del borde cercano del hueco.
    # 1. Rumbo y X: recta x = a + b*y con el tramo de pared cercano (la hoja de la puerta, en el lado lejano, no entra).
    # 2. Y: el lateral del marco (puntos detrás de la recta justo donde se acaba la pared). El ancho es solo un aviso.
    #    Si desde aquí no se ve el marco, la Y sale del final de la canaleta (y_fin_canaleta): motivo 'canaleta'.
    # Devuelve (dx, dy, dth, nivel, motivo, datos): la corrección a sumar a la pose creída; nivel 'total' (dx, dy,
    # dth), 'parcial' (solo dx y dth: la Y no es fiable) o '' (nada: la recta no es fiable); motivo; y lo medido.
    datos = {'n': 0, 'rms': None, 'x_linea': None, 'y1': None, 'y2': None, 'ancho': None, 'aviso_ancho': False,
             'y_fin': None}
    en_banda = np.abs(px - x_pared) < banda
    sel = en_banda & (py > y_borde - largo - max_dy) & (py < y_borde + max_dy + 0.10)
    x, y = px[sel], py[sel]
    datos['n'] = int(len(x))
    if len(x) < min_puntos:
        return 0.0, 0.0, 0.0, '', 'pocos_puntos', datos

    def mejor_recta(qx, qy):
        # Punto de partida robusto: de todas las rectas x = a + b*y que pasan por dos puntos separados más de 0.3 m en y
        # (y con menos de 6° de pendiente), la que tiene más puntos a < 2 cm. Es la canaleta: lo que más puntos tiene
        # alineados. (Un ajuste por mínimos cuadrados con todo saldría torcido por el escalón de yeso y el marco.)
        mejor = (0, 0.0, 0.0)
        for i in range(len(qy)):
            for j in range(i + 1, len(qy)):
                if abs(qy[j] - qy[i]) < 0.3:
                    continue
                b = (qx[j] - qx[i]) / (qy[j] - qy[i])
                if abs(b) > math.tan(math.radians(6.0)):
                    continue
                a = qx[i] - b * qy[i]
                n = int(np.sum(np.abs(qx - (a + b * qy)) < 0.02))
                if n > mejor[0]:
                    mejor = (n, a, b)
        return mejor

    def tramo_mas_largo(qx, qy, a, b):
        # Puntos a < 2 cm de la recta x = a + b*y, y de ellos el tramo continuo (sin saltos de más de 10 cm en y) más
        # largo: la canaleta. Así no entran el escalón de yeso del final, el marco ni algo que haya en el hueco.
        cerca = np.abs(qx - (a + b * qy)) < 0.02
        cx, cy = qx[cerca], qy[cerca]
        orden = np.argsort(cy)
        cx, cy = cx[orden], cy[orden]
        if len(cy) == 0:
            return cx, cy
        cortes = np.where(np.diff(cy) > 0.10)[0] + 1
        tramos = np.split(np.arange(len(cy)), cortes)
        mejor = max(tramos, key=lambda k: cy[k[-1]] - cy[k[0]])
        return cx[mejor], cy[mejor]

    def marco_y(xl, qx, qy, yfin):
        # Lateral del marco: puntos más de fondo_marco por detrás de la recta (x = xl), justo donde acaba la pared
        m = (qx > xl + fondo_marco) & (qx < xl + banda) & (qy > yfin - 0.05) & (qy < yfin + 0.25)
        return float(np.median(qy[m])) if np.sum(m) >= 3 else None

    # 1. Recta de partida (la de más puntos a < 2 cm); 2. recta definitiva con su tramo continuo más largo
    n, a, b = mejor_recta(x, y)
    if n < min_puntos:
        datos['n'] = n
        return 0.0, 0.0, 0.0, '', 'pocos_puntos', datos
    for _ in range(2):
        x, y = tramo_mas_largo(px[sel], py[sel], a, b)
        if len(x) < min_puntos:
            datos['n'] = int(len(x))
            return 0.0, 0.0, 0.0, '', 'pocos_puntos', datos
        b, a = np.polyfit(y, x, 1)
    datos['n'] = int(len(x))
    datos['rms'] = float(np.sqrt(np.mean((x - (a + b * y)) ** 2)))
    # Rumbo: la recta debería ir exactamente en dirección Y. Se corrige girando los puntos alrededor del robot
    # (así su posición no cambia con el giro); después, la X.
    dth = math.atan(b)
    c, s = math.cos(dth), math.sin(dth)

    def girar(qx, qy):
        return rx + c * (qx - rx) - s * (qy - ry), ry + s * (qx - rx) + c * (qy - ry)

    xr, yr = girar(x, y)
    datos['x_linea'] = float(np.mean(xr))
    dx = x_pared - datos['x_linea']
    if datos['rms'] > max_rms:
        return dx, 0.0, dth, '', 'residuo', datos
    if abs(dx) > max_dx:
        return dx, 0.0, dth, '', 'dx', datos
    if abs(dth) > max_ang:
        return dx, 0.0, dth, '', 'dth', datos
    # 3. Borde cercano: el lateral del marco, justo donde acaba el tramo de pared (la canaleta)
    qx, qy = girar(px[en_banda], py[en_banda])
    yfin = float(np.max(yr))
    datos['y_fin'] = yfin
    y1 = marco_y(datos['x_linea'], qx, qy, yfin)
    if y1 is None:
        # Sin marco: el final de la canaleta. Creíble solo si justo después hay un escalón hacia atrás (el yeso); algo
        # apoyado en la pared cortaría el tramo, pero quedaría por delante de la recta, no por detrás.
        # Basta 1 punto (06/11 14:36): de lado, el escalón da 2-3 rayos y el primero (< 2 cm por detrás) entra en la
        # recta; con >= 2 quedó 1 y no se corrigió la Y (+0.45 m: SIN_CAMINO en la puerta). Detrás de la recta solo
        # puede estar la pared, y el scan ya es la mediana de reloc_scans.
        escalon = ((qx > datos['x_linea'] + 0.02) & (qx < datos['x_linea'] + 0.20)
                   & (qy > yfin) & (qy < yfin + 0.20))
        if y_fin_canaleta is not None and np.sum(escalon) >= 1 and abs(y_fin_canaleta - yfin) <= max_dy:
            return dx, y_fin_canaleta - yfin, dth, 'total', 'canaleta', datos
        return dx, 0.0, dth, 'parcial', 'sin_borde', datos
    datos['y1'] = y1
    # Borde creíble: pared justo antes de y1, y nada sobre la línea de pared en los 0.5*ancho siguientes
    linea = np.abs(qx - datos['x_linea']) < 0.05
    antes = np.sum(linea & (qy > y1 - 0.5) & (qy <= y1))
    ocupado = np.sum(linea & (qy > y1 + 0.02) & (qy < y1 + 0.5 * ancho))
    # Ancho, solo como aviso: hasta el primer punto sobre la línea al otro lado (la bisagra de la hoja)
    lejos = qy[(np.abs(qx - datos['x_linea']) < 0.03) & (qy >= y1 + 0.5 * ancho)]
    if len(lejos):
        datos['y2'] = float(lejos.min())
        datos['ancho'] = datos['y2'] - y1
    datos['aviso_ancho'] = datos['ancho'] is None or abs(datos['ancho'] - ancho) > tol_ancho
    dy = y_borde - y1
    if antes < min_cercano or ocupado > 0:
        return dx, dy, dth, 'parcial', 'borde', datos     # borde poco creíble (tapado, o algo en el hueco)
    if abs(dy) > max_dy:
        return dx, dy, dth, 'parcial', 'dy', datos
    return dx, dy, dth, 'total', '', datos


def ajuste_mapa(r, a, x, y, th, offset, mapa, meta, ventana, sig_medida, sig_xy, sig_th, min_puntos):
    # Corrección de la pose con el mapa: la actualización del EKF (Localization, diapositivas 10-11), con cada punto del
    # LiDAR cerca de una pared como una medida. 'mapa' tiene, por celda, la distancia a la pared y su gradiente.
    # Si el punto está a distancia d de la pared, mover el robot (dx, dy, dth) debería llevarlo a d = 0:
    #     g * (dx, dy) + rotación * dth = -d
    # Se resuelve por mínimos cuadrados, con un término que mantiene (dx, dy, dth) cerca de 0: la confianza en la
    # predicción de la odometría. Así, lo que las paredes no observan (avanzar a lo largo de un pasillo) no se mueve.
    # Solo se usan los puntos a menos de 'ventana' de una pared: mesas, sillas y personas quedan fuera.
    # Devuelve (dx, dy, dth, n), con n los puntos usados; (0, 0, 0, n) si son menos de min_puntos.
    res = meta['resolucion']
    info = np.diag([1.0 / sig_xy ** 2, 1.0 / sig_xy ** 2, 1.0 / sig_th ** 2])
    dx, dy, dth = 0.0, 0.0, 0.0
    n = 0
    for _ in range(3):                  # 3 iteraciones: la normal de cada punto cambia al mover la pose
        rx, ry, rth = x + dx, y + dy, th + dth
        lx = rx + offset * math.cos(rth)
        ly = ry + offset * math.sin(rth)
        px = lx + r * np.cos(rth + a)
        py = ly + r * np.sin(rth + a)
        fi = np.floor((px - meta['origen_x']) / res).astype(int)
        co = np.floor((py - meta['origen_y']) / res).astype(int)
        dentro = (fi >= 0) & (fi < mapa.shape[1]) & (co >= 0) & (co < mapa.shape[2])
        fi, co, px, py = fi[dentro], co[dentro], px[dentro], py[dentro]
        d = mapa[0, fi, co]
        u = d < ventana
        n = int(np.sum(u))
        if n < min_puntos:
            return 0.0, 0.0, 0.0, n
        gx, gy = mapa[1, fi[u], co[u]], mapa[2, fi[u], co[u]]
        J = np.column_stack([gx, gy, -gx * (py[u] - ry) + gy * (px[u] - rx)])
        A = J.T @ J / sig_medida ** 2 + info
        b = -J.T @ d[u] / sig_medida ** 2 - info @ np.array([dx, dy, dth])
        paso = np.linalg.solve(A, b)
        dx, dy, dth = dx + float(paso[0]), dy + float(paso[1]), dth + float(paso[2])
    return dx, dy, dth, n


def rampa(v, v_anterior, paso):
    # Al acelerar, como mucho 'paso' más que la velocidad anterior; al frenar (o parar), sin límite
    return min(v, v_anterior + paso) if v > v_anterior else v


def longitud_camino(puntos):
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(puntos, puntos[1:])))


def rodeo_largo(nuevo, antes, ratio, extra):
    # 06/11 15:10: con el pasillo de E a la puerta tapado (otro robot), el replan daba la vuelta por las mesas, hacia
    # atrás. Un camino así de largo para un obstáculo que se mueve: mejor esperar a que se vaya.
    return nuevo > ratio * antes + extra


def recta_ransac(u, v, max_pendiente, tol=0.02, sep=0.20):
    # Recta v = a + b*u con más puntos a < tol (pares de puntos separados más de sep en u, |b| <= max_pendiente),
    # reajustada por mínimos cuadrados con esos puntos. Devuelve (a, b, máscara de inliers) o None.
    mejor = (0, None)
    for i in range(len(u)):
        for j in range(i + 1, len(u)):
            if abs(u[j] - u[i]) < sep:
                continue
            b = (v[j] - v[i]) / (u[j] - u[i])
            if abs(b) > max_pendiente:
                continue
            dentro = np.abs(v - (v[i] + b * (u - u[i]))) < tol
            if np.sum(dentro) > mejor[0]:
                mejor = (int(np.sum(dentro)), dentro)
    if mejor[1] is None:
        return None
    dentro = mejor[1]
    for _ in range(2):
        b, a = np.polyfit(u[dentro], v[dentro], 1)
        dentro = np.abs(v - (a + b * u)) < tol
    return float(a), float(b), dentro


def ajuste_esquina(px, py, rx, ry, x_pared, y_pared, banda, alcance, max_rms, min_puntos, max_corr, max_ang,
                   max_dif_ang, eje_rumbo='Y'):
    # Relocalización con dos paredes perpendiculares medidas con cinta (06/11: la esquina junto a Q, x = x_pared delante
    # e y = y_pared a un lado). Como comprobar_pose en A, pero sin el mapa, que detrás de la puerta está mal en Y (~0.3 m).
    # px, py: lecturas al mundo con la pose creída (rx, ry). Cada pared: los puntos a menos de 'banda' de su recta y a
    # menos de 'alcance' del robot a lo largo de ella; recta por RANSAC. El rumbo, la media (por puntos) del de las dos
    # rectas; después, girando los puntos alrededor del robot, la distancia a cada pared da dx y dy.
    # eje_rumbo: el rumbo sale solo de la pared de ese eje ('Y' = la recta y = y_pared, como pose_heading_axis en A;
    # '' = las dos). En Q las dos paredes no son perpendiculares: ~3° entre ellas (bags del 06/11).
    # Devuelve (dx, dy, dth, motivo, datos): motivo '' si vale; si no, por qué no se corrige.
    datos = {'nx': 0, 'ny': 0, 'rms_x': None, 'rms_y': None, 'dth_x': None, 'dth_y': None}
    pend = math.tan(max_ang)
    sx = (np.abs(px - x_pared) < banda) & (np.abs(py - ry) < alcance)
    sy = (np.abs(py - y_pared) < banda) & (np.abs(px - rx) < alcance)
    rx_ = recta_ransac(py[sx], px[sx], pend) if np.sum(sx) >= 2 else None   # pared X: x = a + b*y
    ry_ = recta_ransac(px[sy], py[sy], pend) if np.sum(sy) >= 2 else None   # pared Y: y = a + b*x
    if rx_ is None or ry_ is None:
        return 0.0, 0.0, 0.0, 'sin_pared_x' if rx_ is None else 'sin_pared_y', datos
    xa, xb, xin = rx_
    ya, yb, yin = ry_
    qxx, qxy = px[sx][xin], py[sx][xin]     # puntos de la pared X
    qyx, qyy = px[sy][yin], py[sy][yin]     # puntos de la pared Y
    datos.update(nx=len(qxx), ny=len(qyx),
                 rms_x=float(np.sqrt(np.mean((qxx - (xa + xb * qxy)) ** 2))),
                 rms_y=float(np.sqrt(np.mean((qyy - (ya + yb * qyx)) ** 2))),
                 dth_x=math.atan(xb), dth_y=-math.atan(yb))
    if min(datos['nx'], datos['ny']) < min_puntos:
        return 0.0, 0.0, 0.0, 'pocos_puntos', datos
    if max(datos['rms_x'], datos['rms_y']) > max_rms:
        return 0.0, 0.0, 0.0, 'residuo', datos
    if abs(datos['dth_x'] - datos['dth_y']) > max_dif_ang:
        return 0.0, 0.0, 0.0, 'angulos', datos
    if eje_rumbo == 'Y':
        dth = datos['dth_y']
    elif eje_rumbo == 'X':
        dth = datos['dth_x']
    else:
        dth = (datos['dth_x'] * datos['nx'] + datos['dth_y'] * datos['ny']) / (datos['nx'] + datos['ny'])
    c, s = math.cos(dth), math.sin(dth)
    dx = x_pared - float(np.mean(rx + c * (qxx - rx) - s * (qxy - ry)))
    dy = y_pared - float(np.mean(ry + s * (qyx - rx) + c * (qyy - ry)))
    if abs(dx) > max_corr or abs(dy) > max_corr or abs(dth) > max_ang:
        return dx, dy, dth, 'grande', datos
    return dx, dy, dth, '', datos


class Fase1Node(Node):
    def __init__(self):
        super().__init__('fase1_node')

        # 1. Parámetros. Sin valores por defecto: todo viene de config/fase1.yaml.
        #    Si falta alguno, el nodo falla al arrancar en vez de usar un valor inventado.
        D = Parameter.Type.DOUBLE
        DA = Parameter.Type.DOUBLE_ARRAY
        SA = Parameter.Type.STRING_ARRAY
        self.pose_inicial = self.declare_parameter('initial_pose', DA).value  # [x, y, theta]
        self.pose_final = self.declare_parameter('final_pose', DA).value
        nombres = self.declare_parameter('waypoint_names', SA).value
        xs = self.declare_parameter('waypoints_x', DA).value
        ys = self.declare_parameter('waypoints_y', DA).value
        obligatorios = self.declare_parameter('mandatory_waypoints', SA).value

        control_rate = self.declare_parameter('control_rate', D).value
        self.dt_control = 1.0 / control_rate
        # Rampa de aceleración (solo al acelerar; frenar es inmediato). 06/11: de 0 a 0.20 m/s de golpe, el robot
        # cabeceaba y el rumbo de /odom (del IMU de la OpenCR) saltaba ~30°: GIRAR/AVANZAR sin avanzar.
        self.acel_max = self.declare_parameter('max_linear_accel', D).value
        self.tol_waypoint = self.declare_parameter('tol_waypoint', D).value
        self.tol_obligatorio = self.declare_parameter('tol_mandatory', D).value
        self.tol_final_yaw = self.declare_parameter('tol_final_yaw', D).value
        self.max_v = self.declare_parameter('max_linear_speed', D).value
        self.max_w = self.declare_parameter('max_angular_speed', D).value
        self.k_lineal = self.declare_parameter('k_linear', D).value
        self.k_angular = self.declare_parameter('k_angular', D).value
        self.angulo_girar = self.declare_parameter('turn_in_place_angle', D).value
        self.angulo_reorientar = self.declare_parameter('reorient_angle', D).value

        self.stop_distance = self.declare_parameter('stop_distance', D).value
        self.cono = math.radians(self.declare_parameter('stop_half_angle_deg', D).value)
        self.timeout = self.declare_parameter('scan_timeout', D).value
        self.lidar_offset_x = self.declare_parameter('lidar_offset_x', D).value
        self.lidar_yaw_offset = self.declare_parameter('lidar_yaw_offset', D).value
        # Relocalización con la pared de la puerta, en el último waypoint antes de reloc_before (robot parado)
        self.reloc_antes_de = self.declare_parameter('reloc_before', Parameter.Type.STRING).value
        self.reloc_n = self.declare_parameter('reloc_scans', Parameter.Type.INTEGER).value
        self.reloc_bloqueo_dist = self.declare_parameter('reloc_on_block_dist', D).value
        self.reloc_pared_x = self.declare_parameter('reloc_wall_x', D).value
        self.reloc_borde_y = self.declare_parameter('reloc_edge_y', D).value
        self.reloc_ancho = self.declare_parameter('reloc_door_width', D).value
        self.reloc_tol_ancho = self.declare_parameter('reloc_door_width_tol', D).value
        self.reloc_banda = self.declare_parameter('reloc_band', D).value
        self.reloc_largo = self.declare_parameter('reloc_wall_length', D).value
        self.reloc_fondo_marco = self.declare_parameter('reloc_frame_depth', D).value
        self.reloc_max_rms = self.declare_parameter('reloc_max_rms', D).value
        self.reloc_min_puntos = self.declare_parameter('reloc_min_points', Parameter.Type.INTEGER).value
        self.reloc_min_cercano = self.declare_parameter('reloc_min_points_near', Parameter.Type.INTEGER).value
        self.reloc_max_dx = self.declare_parameter('reloc_max_dx', D).value
        self.reloc_max_dy = self.declare_parameter('reloc_max_dy', D).value
        self.reloc_comprobar_dist = self.declare_parameter('reloc_check_dist', D).value
        self.reloc_comprobar_ang = math.radians(self.declare_parameter('reloc_check_angle_deg', D).value)
        self.reloc_max_ang = math.radians(self.declare_parameter('reloc_max_angle_deg', D).value)
        self.reloc_fin_canaleta = self.declare_parameter('reloc_duct_end_y', D).value
        self.reloc_reintento_dist = self.declare_parameter('reloc_retry_dist', D).value
        self.reloc_reintento_max = self.declare_parameter('reloc_retry_max', Parameter.Type.INTEGER).value
        # Relocalización en Q con las dos paredes de la esquina (medidas con cinta), al llegar
        self.reloc_q_en = self.declare_parameter('reloc_q_at', Parameter.Type.STRING).value
        self.reloc_q_veces = self.declare_parameter('reloc_q_times', Parameter.Type.INTEGER).value
        self.reloc_q_pared_x = self.declare_parameter('reloc_q_wall_x', D).value
        self.reloc_q_pared_y = self.declare_parameter('reloc_q_wall_y', D).value
        self.reloc_q_eje_rumbo = self.declare_parameter('reloc_q_heading_axis', Parameter.Type.STRING).value
        self.reloc_q_banda = self.declare_parameter('reloc_q_band', D).value
        self.reloc_q_alcance = self.declare_parameter('reloc_q_span', D).value
        self.reloc_q_max_rms = self.declare_parameter('reloc_q_max_rms', D).value
        self.reloc_q_min_puntos = self.declare_parameter('reloc_q_min_points', Parameter.Type.INTEGER).value
        self.reloc_q_max_corr = self.declare_parameter('reloc_q_max_corr', D).value
        self.reloc_q_max_ang = math.radians(self.declare_parameter('reloc_q_max_angle_deg', D).value)
        self.reloc_q_max_dif = math.radians(self.declare_parameter('reloc_q_max_angle_diff_deg', D).value)
        # Corrección continua con el mapa de distancias. Modo: "apagado", "sombra" (calcula y registra, no aplica) o "activo"
        self.corr_modo = self.declare_parameter('map_corr_mode', Parameter.Type.STRING).value
        self.corr_cada = self.declare_parameter('map_corr_every_scans', Parameter.Type.INTEGER).value
        self.corr_max_w = self.declare_parameter('map_corr_max_turn_rate', D).value
        self.corr_y_max = self.declare_parameter('map_corr_y_max', D).value
        self.corr_alcance = self.declare_parameter('map_corr_max_range', D).value
        self.corr_ventana = self.declare_parameter('map_corr_window', D).value
        self.corr_sig_medida = self.declare_parameter('map_corr_sigma_range', D).value
        self.corr_sig_xy = self.declare_parameter('map_corr_sigma_xy', D).value
        self.corr_sig_th = math.radians(self.declare_parameter('map_corr_sigma_angle_deg', D).value)
        self.corr_max_xy = self.declare_parameter('map_corr_max_step', D).value
        self.corr_max_th = math.radians(self.declare_parameter('map_corr_max_angle_deg', D).value)
        self.corr_min_puntos = self.declare_parameter('map_corr_min_points', Parameter.Type.INTEGER).value
        self.usar_imu = self.declare_parameter('use_imu_yaw', Parameter.Type.BOOL).value
        log_file = self.declare_parameter('log_file', Parameter.Type.STRING).value
        self.log_period = self.declare_parameter('log_period', D).value

        # Paso 3: obstáculos nuevos y replanificación
        self.margen_estatico = self.declare_parameter('obstacle_static_margin', D).value
        self.margen_plan = self.declare_parameter('obstacle_plan_margin', D).value
        self.memoria = self.declare_parameter('obstacle_memory_time', D).value
        self.margen_borrado = self.declare_parameter('obstacle_clear_margin', D).value
        self.escaneos_borrado = self.declare_parameter('obstacle_clear_scans', Parameter.Type.INTEGER).value
        self.dist_min_cero = self.declare_parameter('obstacle_clear_zero_min_dist', D).value
        self.separacion_log = self.declare_parameter('obstacle_log_spacing', D).value
        self.alcance_obstaculos = self.declare_parameter('obstacle_max_range', D).value
        # Más cerca que esto no se recuerda nada (solo puede ser el propio robot o un eco de la ventana del LiDAR).
        # La parada de emergencia del cono frontal NO usa este filtro.
        self.alcance_min_obstaculos = self.declare_parameter('obstacle_min_range', D).value
        self.giro_max_obstaculos = self.declare_parameter('obstacle_max_turn_rate', D).value
        self.alcance_bloqueo = self.declare_parameter('block_lookahead', D).value
        self.histeresis = self.declare_parameter('block_hysteresis', D).value
        self.espera = self.declare_parameter('obstacle_wait_time', D).value
        self.reintento = self.declare_parameter('replan_retry_period', D).value
        # Rodeo largo: un camino nuevo más largo que replan_max_ratio * el que queda + replan_max_extra no se toma hasta
        # llevar replan_long_detour_after s sin camino (un robot o una persona en el pasillo se va: mejor esperar)
        self.rodeo_ratio = self.declare_parameter('replan_max_ratio', D).value
        self.rodeo_extra = self.declare_parameter('replan_max_extra', D).value
        self.rodeo_espera = self.declare_parameter('replan_long_detour_after', D).value
        # Obligatorio tapado: tras mandatory_cover_wait s sin camino, se va al punto libre más cercano a él (a menos de
        # mandatory_cover_radius) y se da por alcanzado (OBLIGATORIO_TAPADO). Nunca para los de mandatory_cover_exclude.
        self.tapado_radio = self.declare_parameter('mandatory_cover_radius', D).value
        self.tapado_espera = self.declare_parameter('mandatory_cover_wait', D).value
        self.tapado_excluir = self.declare_parameter('mandatory_cover_exclude', SA).value
        self.inflado = self.declare_parameter('inflation_radius', D).value
        self.inflados = [self.inflado, self.declare_parameter('inflation_radius_retry', D).value,
                         self.declare_parameter('inflation_radius_min', D).value]
        self.holgura_objetivo = self.declare_parameter('clearance_target', D).value
        self.ganancia = self.declare_parameter('clearance_cost_gain', D).value
        self.simp_tope = self.declare_parameter('simplify_clearance_cap', D).value
        self.simp_margen = self.declare_parameter('simplify_margin', D).value
        self.simp_paso = self.declare_parameter('simplify_step', D).value
        # Anclas: puntos de la secuencia (B, D...) por los que hay que pasar aunque no sean obligatorios
        # (B y D rodean las mesas, que no están en el mapa)
        self.anclas = self.declare_parameter('planning_sequence', SA).value
        self.obligatorios = obligatorios

        # Holgura estática de la rejilla de 5 cm (paso 2) y sus metadatos
        ruta = self.declare_parameter('static_clearance_file', Parameter.Type.STRING).value
        if not os.path.isabs(ruta):
            ruta = os.path.join(get_package_share_directory('autonomous_nav'), ruta)
        self.holgura = np.load(ruta)
        with open(ruta.replace('.npy', '.yaml')) as f:
            m = yaml.safe_load(f)
        self.meta = {'resolucion': m['resolucion'], 'origen_x': m['origen_x'], 'origen_y': m['origen_y']}

        # Mapa de distancias para la corrección con el mapa (lo genera scripts/generar_mapa_distancias.py)
        self.corr_ultimo_scan = None
        self.corr_cuenta = 0
        if self.corr_modo != 'apagado':
            ruta = self.declare_parameter('map_corr_file', Parameter.Type.STRING).value
            if not os.path.isabs(ruta):
                ruta = os.path.join(get_package_share_directory('autonomous_nav'), ruta)
            self.corr_mapa = np.load(ruta)
            with open(ruta.replace('.npy', '.yaml')) as f:
                self.corr_meta = yaml.safe_load(f)

        # 2. Lista de objetivos: (nombre, x, y, tolerancia). Al final, la posición de la pose final.
        if not (len(nombres) == len(xs) == len(ys)):
            raise ValueError('waypoint_names, waypoints_x y waypoints_y deben tener la misma longitud')
        posiciones = [nombres.index(n) for n in obligatorios if n in nombres]
        if len(posiciones) != len(obligatorios) or posiciones != sorted(posiciones):
            self.get_logger().warn(f'Los obligatorios {obligatorios} no están todos, o no en orden, en {nombres}')
        self.objetivos = []
        for nombre, x, y in zip(nombres, xs, ys):
            tol = self.tol_obligatorio if nombre in obligatorios else self.tol_waypoint
            self.objetivos.append((nombre, x, y, tol))
        self.objetivos.append(('final', self.pose_final[0], self.pose_final[1], self.tol_obligatorio))
        self.i_objetivo = 0
        self.estado = 'GIRAR'

        # 3. Estimación de la pose (odometría + giróscopo) y cuándo llegó cada sensor (reloj del nodo)
        self.odom_anterior = None   # última lectura de /odom: (x, y, theta) en el sistema de /odom
        self.x_rel = 0.0            # pose relativa al arranque: lo que se ha movido desde la salida
        self.y_rel = 0.0
        self.th_rel = 0.0
        self.yaw_imu = 0.0          # giro acumulado según el giróscopo desde la salida
        self.yaw_odom = 0.0         # giro acumulado según /odom (solo para el log; no se usa para navegar)
        self.hueco_imu = None       # duración de un hueco de /imu pendiente de escribir en el log
        self.t_imu_anterior = None  # sello de tiempo del último /imu (para el dt de la integración)
        self.t_odom = None
        self.t_imu = None
        self.scan = None
        self.t_scan = None

        # Banderas para registrar los eventos solo cuando cambian (y no en cada ciclo)
        self.sin_sensores = True
        self.frenado = False
        self.t_ultimo_log = None

        # Memoria de obstáculos nuevos: celda de 5 cm -> [x, y, última vez que se vio (s), escaneos seguidos
        # en los que los rayos lo ven libre]
        self.obstaculos = {}
        self.scan_borrado = None    # último scan con el que se contaron esos escaneos (se cuenta una vez por scan)
        self.obstaculos_log = []    # posiciones ya escritas en el Mission Log
        self.t_espera = None        # cuándo empezó la espera en ESPERAR
        self.n_replan = 0
        self.w_anterior = 0.0       # última velocidad angular mandada
        self.v_anterior = 0.0       # última velocidad lineal mandada (rampa de aceleración)
        self.reloc_scans_lista = [] # scans juntados en RELOC
        self.t_sin_camino = None    # desde cuándo no hay camino (para el obligatorio tapado)
        self.sustitutos = {}        # nombre del sustituto -> (obligatorio, x, y)
        self.reloc_ultimo_scan = None
        self.reloc_adelantada = False   # ya se ha relocalizado por un bloqueo antes del waypoint (solo una vez)
        self.reloc_tipo = 'puerta'      # qué relocalización hace el estado RELOC: 'puerta' (ajuste_puerta) o 'Q' (esquina)
        self.reloc_puerta_total = False # ya ha salido una relocalización total en la puerta (si no, se reintenta)
        self.reloc_reintentos = 0
        self.reloc_q_hechas = 0
        self.rodeo_rechazado = None     # (longitud nueva, la que quedaba) del último rodeo no aceptado

        # 4. Publicador y suscriptores (QoS como en la plantilla)
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_profile_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_r)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_b)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_b)  # siempre: se registra en el log

        # 5. Mission Log. Un fichero por ejecución (con la fecha) para no machacar el de otra prueba.
        base, ext = os.path.splitext(log_file)
        ruta_log = f"{base}_{datetime.now().strftime('%Y%m%d_%H%M%S')}{ext}"
        self.fichero = open(ruta_log, 'w', newline='')
        self.csv = csv.writer(self.fichero)
        self.csv.writerow(['timestamp', 'current_phase', 'robot_x', 'robot_y', 'robot_yaw',
                           'state', 'event', 'error', 'obstacle_x', 'obstacle_y', 'yaw_odom', 'yaw_imu'])
        self.get_logger().info(f'Mission Log en {os.path.abspath(ruta_log)}')

        # 6. Bucle de control a ritmo fijo
        self.create_timer(1.0 / control_rate, self.control_loop)

    # ---------- Callbacks ----------
    # /scan solo guarda el último mensaje. /odom y /imu además ACUMULAN: integrar no admite saltarse
    # mensajes, así que la integración (pocas líneas) va aquí y no en el timer.

    def odom_callback(self, msg):
        p = msg.pose.pose
        x, y, th = p.position.x, p.position.y, yaw_de_cuaternion(p.orientation)
        if self.odom_anterior is not None:
            xa, ya, tha = self.odom_anterior
            # Avance (con signo) desde la lectura anterior, en la dirección en que iba según /odom.
            # De /odom solo nos fiamos de la distancia: el rumbo, si se puede, lo da el giróscopo.
            self.yaw_odom += normalizar(th - tha)
            ds = (x - xa) * math.cos(tha) + (y - ya) * math.sin(tha)
            th_antes = self.th_rel
            self.th_rel = self.yaw_imu if self.usar_imu else self.th_rel + normalizar(th - tha)
            th_medio = th_antes + 0.5 * normalizar(self.th_rel - th_antes)
            self.x_rel += ds * math.cos(th_medio)
            self.y_rel += ds * math.sin(th_medio)
        self.odom_anterior = (x, y, th)
        self.t_odom = self.get_clock().now()

    def imu_callback(self, msg):
        # Rumbo integrando la velocidad angular del giróscopo (como hace el driver del TB3 real).
        # Con el sello de tiempo del propio /imu: el dt entre dos lecturas del mismo reloj.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_imu_anterior is not None:
            dt = t - self.t_imu_anterior
            if 0.0 < dt < 0.5:
                self.yaw_imu += msg.angular_velocity.z * dt
            elif dt >= 0.5:
                # Hueco: el giro de ese rato se pierde. Se apunta y control_loop lo escribe en el log.
                self.hueco_imu = dt
                self.get_logger().warn(f'Hueco de {dt:.2f} s en /imu')
        self.t_imu_anterior = t
        self.t_imu = self.get_clock().now()

    def scan_callback(self, msg):
        self.scan = msg
        self.t_scan = self.get_clock().now()

    # ---------- Pose ----------

    def pose_mundo(self):
        # Pose (x, y, theta) del robot en el sistema del enunciado, o None si aún no hay /odom.
        # Es la única función que sabe de dónde sale la pose: con el SLAM solo cambiará esta.
        # (x_rel, y_rel, th_rel) es lo que se ha movido desde la salida, con los ejes del robot al salir.
        if self.odom_anterior is None:
            return None
        # Al mundo: girar theta_i y trasladar a (x_i, y_i)
        xi, yi, thi = self.pose_inicial
        x = xi + math.cos(thi) * self.x_rel - math.sin(thi) * self.y_rel
        y = yi + math.sin(thi) * self.x_rel + math.cos(thi) * self.y_rel
        return x, y, normalizar(thi + self.th_rel)

    # ---------- Seguridad ----------

    def sensor_caducado(self, t):
        # True si el sensor no ha llegado nunca o lleva más de scan_timeout sin llegar
        if t is None:
            return True
        return (self.get_clock().now() - t).nanoseconds * 1e-9 > self.timeout

    def obstaculo_delante(self, x, y, th):
        # Lectura válida más cercana del cono frontal que esté a menos de stop_distance.
        # Devuelve su posición en el mundo (ox, oy), o None si el cono está libre.
        msg = self.scan
        r_min = None
        a_min = 0.0
        for i, r in enumerate(msg.ranges):
            # 0.0, inf, nan o fuera de [range_min, range_max] = sin dato
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            # Ángulo de cada lectura (no suponemos 360 muestras ni que el índice 0 sea el frente)
            a = normalizar(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= self.cono and r < self.stop_distance and (r_min is None or r < r_min):
                r_min = r
                a_min = a
        if r_min is None:
            return None
        # El LiDAR está desplazado lidar_offset_x sobre el eje del robot
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        return lx + r_min * math.cos(th + a_min), ly + r_min * math.sin(th + a_min)

    # ---------- Obstáculos nuevos y replanificación (paso 3) ----------

    def actualizar_obstaculos(self, x, y, th, ahora):
        # Memoria de obstáculos nuevos (lo que el LiDAR ve y el mapa no explica).
        # - Se añade cada lectura válida que cae donde la holgura estática es > obstacle_static_margin.
        # - Se olvida si lleva obstacle_memory_time sin verse, o si ahora un rayo del LiDAR pasa a través
        #   de él (la persona se ha ido). Así no se olvida al girar y dejar de verlo (eso haría oscilar
        #   al robot entre el plan nuevo y el original), pero tampoco quedan obstáculos fantasma.
        # Girando deprisa no se toca la memoria: el scan llega con retraso respecto a la pose y las
        # paredes aparecerían desplazadas (obstáculos falsos, o borrados falsos).
        if abs(self.w_anterior) > self.giro_max_obstaculos:
            return
        msg = self.scan
        r = np.array(msg.ranges, dtype=float)
        n = len(r)
        a = msg.angle_min + np.arange(n) * msg.angle_increment
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)

        # a) Lecturas válidas y cercanas en el mundo, y las que no explica el mapa
        validas = (np.isfinite(r) & (r >= max(msg.range_min, self.alcance_min_obstaculos))
                   & (r <= min(msg.range_max, self.alcance_obstaculos)))
        px = lx + r[validas] * np.cos(th + a[validas])
        py = ly + r[validas] * np.sin(th + a[validas])
        res = self.meta['resolucion']
        fi = np.floor((px - self.meta['origen_x']) / res).astype(int)
        co = np.floor((py - self.meta['origen_y']) / res).astype(int)
        dentro = (fi >= 0) & (fi < self.holgura.shape[0]) & (co >= 0) & (co < self.holgura.shape[1])
        nuevo = np.zeros(len(px), dtype=bool)
        nuevo[dentro] = self.holgura[fi[dentro], co[dentro]] > self.margen_estatico
        for ox, oy, i, j in zip(px[nuevo], py[nuevo], fi[nuevo], co[nuevo]):
            if (i, j) not in self.obstaculos and all(
                    math.hypot(ox - qx, oy - qy) > self.separacion_log for qx, qy in self.obstaculos_log):
                self.obstaculos_log.append((ox, oy))
                self.escribir_log(x, y, th, 'OBSTACULO', None, ox, oy)
            self.obstaculos[(i, j)] = [ox, oy, ahora, 0]

        # b) Olvidar: por tiempo, o porque los rayos que apuntan a él (y sus vecinos) lo ven libre (rayos_libres)
        #    en obstacle_clear_scans escaneos SEGUIDOS: una lectura suelta (un 0.0 de ropa negra) no basta.
        #    Se cuenta una vez por scan: el timer va más rápido que el LiDAR y vería el mismo scan dos veces.
        scan_nuevo = msg is not self.scan_borrado
        self.scan_borrado = msg
        for celda, ob in list(self.obstaculos.items()):
            ox, oy, t = ob[0], ob[1], ob[2]
            if ahora - t > self.memoria:
                del self.obstaculos[celda]
                continue
            if not scan_nuevo:
                continue
            d = math.hypot(ox - lx, oy - ly)
            if not (msg.range_min < d < msg.range_max):
                continue
            k = int(round(((math.atan2(oy - ly, ox - lx) - th - msg.angle_min) % (2 * math.pi)) / msg.angle_increment))
            rayos = [r[(k + dk) % n] for dk in (-1, 0, 1)]
            if rayos_libres(rayos, d, self.margen_borrado, self.dist_min_cero):
                ob[3] += 1
                if ob[3] >= self.escaneos_borrado:
                    del self.obstaculos[celda]
            else:
                ob[3] = 0

    def ruta_bloqueada(self, x, y, extra=0.0):
        # Obstáculo nuevo a menos de inflation_radius de la ruta que queda (los próximos block_lookahead m).
        # 'extra' agranda las dos distancias: en ESPERAR, para dar la ruta por libre se pide ese margen
        # de más (histéresis; sin ella, un obstáculo justo en el límite hace parpadear bloqueo/libre).
        # Devuelve su posición, o None si la ruta está libre.
        # Solo hasta el siguiente obligatorio (o su sustituto, o la pose final): lo de después aún no se ha replanificado
        # y se comprueba al llegar allí. Si no, un obstáculo junto al obligatorio bloquea el tramo siguiente y el replan,
        # que solo llega hasta el obligatorio, no lo arregla nunca (bucle BLOQUEO/REPLANIFICACION junto a E, 06/11).
        if not self.obstaculos:
            return None
        resto = self.objetivos[self.i_objetivo:]
        k = next((k for k, o in enumerate(resto)
                  if o[0] in self.obligatorios or o[0] in self.sustitutos or o[0] == 'final'), len(resto) - 1)
        puntos = [(x, y)] + [(o[1], o[2]) for o in resto[:k + 1]]
        muestras = []
        restante = self.alcance_bloqueo + extra
        for (ax, ay), (bx, by) in zip(puntos, puntos[1:]):
            largo = math.hypot(bx - ax, by - ay)
            if largo == 0.0:
                continue
            usado = min(largo, restante)
            t = np.linspace(0.0, usado / largo, int(math.ceil(usado / 0.05)) + 1)
            muestras.append(np.column_stack([ax + t * (bx - ax), ay + t * (by - ay)]))
            restante -= usado
            if restante <= 0.0:
                break
        if not muestras:
            return None
        s = np.vstack(muestras)
        o = np.array([(v[0], v[1]) for v in self.obstaculos.values()])
        d = np.hypot(s[:, None, 0] - o[None, :, 0], s[:, None, 1] - o[None, :, 1]).min(axis=0)
        # Un obstáculo que ya está a menos de inflation_radius del robot solo bloquea si la ruta se le acerca todavía más.
        # 06/11 15:32: con algo a 0.22 m del robot junto a E, cualquier camino (que sale del robot) pasaba a < 0.25 aunque
        # se alejara: REPLANIFICACION y BLOQUEO cada 2 s durante 25 s. Acercarse sigue bloqueando; la parada de
        # emergencia del cono frontal no cambia.
        d_robot = np.hypot(o[:, 0] - x, o[:, 1] - y)
        bloquea = (d < self.inflado + extra) & (d < d_robot - BLOQUEO_ACERCARSE)
        if not np.any(bloquea):
            return None
        k = int(np.argmin(np.where(bloquea, d, np.inf)))
        return o[k, 0], o[k, 1]

    def esperar(self, x, y, th, ahora):
        # Estado ESPERAR (robot parado). Si la ruta se libera, se sigue con el plan que había.
        # Si a los obstacle_wait_time s sigue bloqueada, se replanifica (con el robot parado).
        if self.ruta_bloqueada(x, y, self.histeresis) is None:
            self.t_sin_camino = None
            self.get_logger().info('Ruta libre: sigo con el plan')
            self.estado = 'GIRAR'
            self.escribir_log(x, y, th, 'RUTA_LIBRE')
            return
        if ahora - self.t_espera < self.espera:
            return
        if self.t_sin_camino is None:
            self.t_sin_camino = ahora
        sustituir = ahora - self.t_sin_camino >= self.tapado_espera
        largo = ahora - self.t_sin_camino >= self.rodeo_espera
        self.rodeo_rechazado = None
        inflado, segundos = self.replanificar(x, y, sustituir, largo)
        if inflado is None:
            self.get_logger().warn(f'Sin camino ({segundos:.2f} s): reintento en {self.reintento:.0f} s')
            self.escribir_log(x, y, th, 'SIN_CAMINO', segundos)
            if self.rodeo_rechazado is not None:
                nuevo, antes = self.rodeo_rechazado
                self.get_logger().warn(f'Rodeo de {nuevo:.1f} m (quedaban {antes:.1f} m): espero')
                self.escribir_log(x, y, th, f'RODEO_LARGO {nuevo:.2f} {antes:.2f}', nuevo)
            self.t_espera = ahora - self.espera + self.reintento   # el próximo intento, dentro de replan_retry_period
            if self.reloc_reintentable(x, y):
                self.reloc_reintentos += 1
                self.get_logger().warn(f'Sin camino junto a la puerta sin relocalización total: relocalizo otra vez '
                                       f'({self.reloc_reintentos}/{self.reloc_reintento_max})')
                self.escribir_log(x, y, th, f'RELOC_REINTENTO {self.reloc_reintentos}')
                self.empezar_reloc('puerta')
            return
        self.get_logger().info(f'Replanificado con inflado {inflado:.2f} m en {segundos:.2f} s: '
                               f'{[o[0] for o in self.objetivos[self.i_objetivo:]]}')
        self.escribir_log(x, y, th, f'REPLANIFICACION inflado={inflado:.2f}', segundos)
        self.t_sin_camino = None
        self.estado = 'GIRAR'

    def replanificar(self, x, y, sustituir=False, largo=True):
        # Desde la pose actual hasta el siguiente punto obligatorio (o la pose final), pasando por las
        # anclas que queden por el camino; después sigue la lista original.
        # Devuelve (inflado usado, segundos), o (None, segundos) si no hay camino ni con el inflado mínimo.
        # sustituir: si el obligatorio está tapado, ir al punto libre más cercano a él (punto_sustituto).
        # largo: si es False, un camino mucho más largo que el que quedaba (rodeo_largo) no vale: se prueba el inflado
        # siguiente y, si ninguno da uno corto, no hay camino (se espera). Queda en self.rodeo_rechazado.
        t0 = time.perf_counter()
        resto = self.objetivos[self.i_objetivo:]
        k = next(k for k, o in enumerate(resto)
                 if o[0] in self.obligatorios or o[0] in self.sustitutos or o[0] == 'final')
        antes = longitud_camino([(x, y)] + [(o[1], o[2]) for o in resto[:k + 1]])
        anclas = [o for o in resto[:k] if o[0] in self.anclas] + [resto[k]]
        despues = resto[k + 1:]
        obst = [(v[0], v[1]) for v in self.obstaculos.values()]
        holgura = pl.holgura_con_obstaculos(self.holgura, self.meta, obst, self.margen_plan, self.holgura_objetivo)
        self.n_replan += 1

        for inflado in self.inflados:
            # Un punto obligatorio tapado por un obstáculo no se puede saltar: no hay camino (se espera), salvo que
            # ya toque sustituirlo por el punto libre más cercano. Un ancla no obligatoria tapada se salta.
            objetivo = anclas[-1]
            if pl.holgura_en(holgura, self.meta, objetivo[1], objetivo[2]) <= inflado:
                if not (sustituir and objetivo[0] in self.obligatorios and objetivo[0] not in self.tapado_excluir):
                    continue
                s = punto_sustituto(holgura, self.meta, objetivo[1], objetivo[2], inflado, self.tapado_radio)
                if s is None:
                    continue
                self.sustitutos[objetivo[0] + '~'] = (objetivo[0], objetivo[1], objetivo[2])
                objetivo = (objetivo[0] + '~', s[0], s[1], self.tol_waypoint)
                self.get_logger().warn(f'{objetivo[0][:-1]} tapado: voy a ({s[0]:.2f}, {s[1]:.2f}) en su lugar')
            lista = [o for o in anclas[:-1] if pl.holgura_en(holgura, self.meta, o[1], o[2]) > inflado] + [objetivo]
            coste = pl.mapa_de_costes(holgura, inflado, self.holgura_objetivo, self.ganancia)
            nuevos = []
            p = (x, y)
            for nombre, ox, oy, tol in lista:
                celdas = pl.planificar_tramo(coste, self.meta, p, (ox, oy))
                if celdas is None:
                    nuevos = None
                    break
                puntos = pl.simplificar(celdas, holgura, self.meta, self.simp_paso, self.simp_tope, self.simp_margen)
                for m, (ix, iy) in enumerate(puntos[1:-1], start=1):
                    nuevos.append((f'replan{self.n_replan}_{m}', ix, iy, self.tol_waypoint))
                nuevos.append((nombre, ox, oy, tol))
                p = (ox, oy)
            if nuevos is None:
                continue
            nuevo = longitud_camino([(x, y)] + [(o[1], o[2]) for o in nuevos])
            if not largo and rodeo_largo(nuevo, antes, self.rodeo_ratio, self.rodeo_extra):
                self.rodeo_rechazado = (nuevo, antes)
                continue
            self.objetivos = self.objetivos[:self.i_objetivo] + nuevos + despues
            return inflado, time.perf_counter() - t0
        return None, time.perf_counter() - t0

    # ---------- Corrección continua con el mapa ----------

    def corregir_con_mapa(self, x, y, th):
        # Cada map_corr_every_scans scans nuevos: si el robot casi no gira (la lectura llega con retraso respecto a la
        # pose) y aún no está en el tramo de la puerta (allí corrige ajuste_puerta), se compara el scan con el mapa.
        # Modo "activo": aplica la corrección, limitada por paso. Modo "sombra": solo la escribe en el log.
        # Devuelve la pose con la que seguir este ciclo.
        msg = self.scan
        if self.corr_modo == 'apagado' or msg is self.corr_ultimo_scan:
            return x, y, th
        self.corr_ultimo_scan = msg
        self.corr_cuenta += 1
        if (self.corr_cuenta % self.corr_cada != 0 or abs(self.w_anterior) > self.corr_max_w or y > self.corr_y_max
                or self.estado not in ('GIRAR', 'AVANZAR', 'ESPERAR')):
            return x, y, th
        r = np.array(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = (np.isfinite(r) & (r >= max(msg.range_min, self.alcance_min_obstaculos))
              & (r <= min(msg.range_max, self.corr_alcance)))
        ok[1::2] = False                # con uno de cada dos rayos basta
        dx, dy, dth, n = ajuste_mapa(r[ok], a[ok], x, y, th, self.lidar_offset_x, self.corr_mapa, self.corr_meta,
                                     self.corr_ventana, self.corr_sig_medida, self.corr_sig_xy, self.corr_sig_th,
                                     self.corr_min_puntos)
        if n < self.corr_min_puntos:
            return x, y, th
        texto = f'dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f} n={n}'
        if self.corr_modo != 'activo':
            self.escribir_log(x, y, th, f'CORR_MAPA_SOMBRA {texto}', math.hypot(dx, dy))
            return x, y, th
        nx = x + limitar(dx, self.corr_max_xy)
        ny = y + limitar(dy, self.corr_max_xy)
        nth = normalizar(th + limitar(dth, self.corr_max_th))
        self.fijar_pose(nx, ny, nth)
        self.escribir_log(nx, ny, nth, f'CORR_MAPA {texto}', math.hypot(dx, dy))
        return nx, ny, nth

    # ---------- Relocalización con las paredes ----------

    def reloc_adelantable(self, x, y):
        # Bloqueo yendo al último waypoint antes de reloc_before (Door), a menos de reloc_on_block_dist de él: casi
        # siempre es el marco de la puerta, visto dentro del hueco porque la Y llega mal (06/11: +0.37 m). Se relocaliza
        # ahí mismo, una vez, en vez de esperar: la relocalización borra los obstáculos, calculados con la Y mala.
        i = self.i_objetivo
        if self.reloc_adelantada or i + 1 >= len(self.objetivos) or self.objetivos[i + 1][0] != self.reloc_antes_de:
            return False
        return math.hypot(self.objetivos[i][1] - x, self.objetivos[i][2] - y) < self.reloc_bloqueo_dist

    def reloc_reintentable(self, x, y):
        # SIN_CAMINO cerca de la puerta (Door aún por delante, a menos de reloc_retry_dist) sin ninguna relocalización
        # total: casi siempre la Y mala (06/11 14:36: RELOC_PARCIAL y SIN_CAMINO sin fin). Se relocaliza otra vez: si
        # alguien tapaba el final de la canaleta y se ha apartado, ahora sale total.
        if self.reloc_puerta_total or self.reloc_reintentos >= self.reloc_reintento_max:
            return False
        puerta = next((o for o in self.objetivos[self.i_objetivo:] if o[0] == self.reloc_antes_de), None)
        return puerta is not None and math.hypot(puerta[1] - x, puerta[2] - y) < self.reloc_reintento_dist

    def empezar_reloc(self, tipo):
        self.reloc_tipo = tipo
        self.reloc_scans_lista = []
        self.estado = 'RELOC'

    def relocalizar(self, x, y, th):
        # Estado RELOC (robot parado): junta reloc_scans scans nuevos y ajusta la pose con la pared de la puerta
        # (ajuste_puerta): total (x, y, rumbo), parcial (x y rumbo) o nada (se sigue con la odometría).
        msg = self.scan
        if msg is self.reloc_ultimo_scan:
            return                  # el timer va más rápido que el LiDAR: cada scan una vez
        self.reloc_ultimo_scan = msg
        r = np.array(msg.ranges, dtype=float)
        r[~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)] = np.nan   # sin dato
        self.reloc_scans_lista.append(r)
        if len(self.reloc_scans_lista) < self.reloc_n:
            return
        # Mediana de los scans; cada lectura al mundo con la pose creída, y el ajuste a la pared de la puerta
        r = cp.mediana_de_scans(self.reloc_scans_lista)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        if self.reloc_tipo == 'Q':
            self.relocalizar_q(r, a, x, y, th)
            return
        dx, dy, dth, nivel, motivo, d = self.ajustar_puerta(r, a, x, y, th)
        if nivel == 'total':
            self.reloc_puerta_total = True
        medido = f"{d['n']} puntos" + (f", rms {d['rms'] * 100:.1f} cm" if d['rms'] is not None else '')
        aviso = ''
        if d['aviso_ancho']:
            aviso = f" AVISO_ANCHO={d['ancho']:.2f}" if d['ancho'] is not None else ' AVISO_ANCHO=sin_lado_lejano'
        if nivel == '':
            texto = f'{motivo} dx={dx:+.3f} dth={math.degrees(dth):+.2f}'
            self.get_logger().warn(f'Relocalización rechazada: {texto} ({medido})')
            self.escribir_log(x, y, th, f'RELOC_RECHAZADA {texto}', abs(dx))
            self.estado = 'GIRAR'
            return
        if nivel == 'total':
            nx, ny, nth = x + dx, y + dy, normalizar(th + dth)
            texto = f'dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}{aviso}' + (f' Y={motivo}' if motivo else '')
            self.get_logger().info(f'Relocalización: {texto} ({medido})')
            self.escribir_log(nx, ny, nth, f'RELOC {texto}', math.hypot(dx, dy))
        else:
            nx, ny, nth = x + dx, y, normalizar(th + dth)
            texto = f'{motivo} dx={dx:+.3f} dth={math.degrees(dth):+.2f} (dy={dy:+.3f} no aplicado){aviso}'
            self.get_logger().warn(f'Relocalización parcial: {texto} ({medido})')
            self.escribir_log(nx, ny, nth, f'RELOC_PARCIAL {texto}', abs(dx))
        self.fijar_pose(nx, ny, nth)
        self.obstaculos = {}        # sus posiciones se calcularon con la pose vieja: se vuelven a detectar
        # Comprobación con el robot aún parado: el mismo ajuste desde la pose corregida debe dar casi 0
        dx2, dy2, dth2, nivel2, motivo2, _ = self.ajustar_puerta(r, a, nx, ny, nth)
        mal = (nivel2 == '' or abs(dx2) > self.reloc_comprobar_dist or abs(dth2) > self.reloc_comprobar_ang
               or (nivel == 'total' and (nivel2 != 'total' or abs(dy2) > self.reloc_comprobar_dist)))
        if mal:
            texto = f'{nivel2 or "rechazada"} {motivo2} dx={dx2:+.3f} dy={dy2:+.3f} dth={math.degrees(dth2):+.2f}'
            self.get_logger().warn(f'Relocalización inconsistente al repetir: {texto}')
            self.escribir_log(nx, ny, nth, f'RELOC_INCONSISTENTE {texto}', math.hypot(dx2, dy2))
        self.estado = 'GIRAR'

    def relocalizar_q(self, r, a, x, y, th):
        # Relocalización con la esquina de Q (ajuste_esquina): x, y y rumbo, o nada. Después el robot vuelve a ir a Q.
        def texto_de(dx, dy, dth, d):
            return (f"dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f} nx={d['nx']} ny={d['ny']}"
                    + (f" dth_x={math.degrees(d['dth_x']):+.2f} dth_y={math.degrees(d['dth_y']):+.2f}"
                       if d['dth_x'] is not None else ''))
        dx, dy, dth, motivo, d = self.ajustar_esquina(r, a, x, y, th)
        self.estado = 'GIRAR'
        if motivo:
            self.get_logger().warn(f'Relocalización en Q rechazada: {motivo} {texto_de(dx, dy, dth, d)}')
            self.escribir_log(x, y, th, f'RELOC_Q_RECHAZADA {motivo} {texto_de(dx, dy, dth, d)}', math.hypot(dx, dy))
            return
        nx, ny, nth = x + dx, y + dy, normalizar(th + dth)
        self.get_logger().info(f'Relocalización en Q: {texto_de(dx, dy, dth, d)}')
        self.escribir_log(nx, ny, nth, f'RELOC_Q {texto_de(dx, dy, dth, d)}', math.hypot(dx, dy))
        self.fijar_pose(nx, ny, nth)
        self.obstaculos = {}
        dx2, dy2, dth2, motivo2, d2 = self.ajustar_esquina(r, a, nx, ny, nth)
        if (motivo2 or abs(dx2) > self.reloc_comprobar_dist or abs(dy2) > self.reloc_comprobar_dist
                or abs(dth2) > self.reloc_comprobar_ang):
            texto = f'{motivo2 or "ok"} {texto_de(dx2, dy2, dth2, d2)}'
            self.get_logger().warn(f'Relocalización en Q inconsistente al repetir: {texto}')
            self.escribir_log(nx, ny, nth, f'RELOC_Q_INCONSISTENTE {texto}', math.hypot(dx2, dy2))

    def ajustar_esquina(self, r, a, x, y, th):
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        ok = ~np.isnan(r)
        px = lx + r[ok] * np.cos(th + self.lidar_yaw_offset + a[ok])
        py = ly + r[ok] * np.sin(th + self.lidar_yaw_offset + a[ok])
        return ajuste_esquina(px, py, x, y, self.reloc_q_pared_x, self.reloc_q_pared_y, self.reloc_q_banda,
                              self.reloc_q_alcance, self.reloc_q_max_rms, self.reloc_q_min_puntos, self.reloc_q_max_corr,
                              self.reloc_q_max_ang, self.reloc_q_max_dif, self.reloc_q_eje_rumbo)

    def ajustar_puerta(self, r, a, x, y, th):
        # Lecturas (r, ángulos a) al mundo con la pose (x, y, th) y ajuste_puerta con los parámetros del yaml
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        ok = ~np.isnan(r)
        px = lx + r[ok] * np.cos(th + self.lidar_yaw_offset + a[ok])
        py = ly + r[ok] * np.sin(th + self.lidar_yaw_offset + a[ok])
        return ajuste_puerta(px, py, x, y, self.reloc_pared_x, self.reloc_borde_y, self.reloc_ancho,
                             self.reloc_tol_ancho, self.reloc_banda, self.reloc_largo, self.reloc_max_rms,
                             self.reloc_min_puntos, self.reloc_min_cercano, self.reloc_max_dx, self.reloc_max_dy,
                             self.reloc_max_ang, self.reloc_fondo_marco, self.reloc_fin_canaleta)

    def fijar_pose(self, x, y, th):
        # Que pose_mundo() pase a dar (x, y, th) sin tocar lo integrado desde la salida (x_rel, y_rel, th_rel):
        # se mueve el origen de la odometría en el mundo (pose_inicial).
        thi = normalizar(th - self.th_rel)
        xi = x - (math.cos(thi) * self.x_rel - math.sin(thi) * self.y_rel)
        yi = y - (math.sin(thi) * self.x_rel + math.cos(thi) * self.y_rel)
        self.pose_inicial = [xi, yi, thi]

    # ---------- Control ----------

    def control_loop(self):
        # 1. Sin datos frescos de algún sensor: parada total
        if (self.sensor_caducado(self.t_odom) or self.sensor_caducado(self.t_scan)
                or (self.usar_imu and self.sensor_caducado(self.t_imu))):
            self.publicar_vel(0.0, 0.0)
            if not self.sin_sensores:
                self.sin_sensores = True
                self.get_logger().warn('Sin /odom, /scan o /imu recientes: parada total')
                x, y, th = self.pose_mundo()
                self.escribir_log(x, y, th, 'TIMEOUT_SENSOR')
            return
        x, y, th = self.pose_mundo()
        if self.sin_sensores:
            self.sin_sensores = False
            self.get_logger().info('Sensores OK')
            self.escribir_log(x, y, th, 'SENSORES_OK')

        if self.hueco_imu is not None:
            self.escribir_log(x, y, th, 'IMU_HUECO', self.hueco_imu)
            self.hueco_imu = None

        x, y, th = self.corregir_con_mapa(x, y, th)

        # 2. Obstáculos nuevos (paso 3): si bloquean la ruta, parar y pasar a ESPERAR
        ahora_s = self.get_clock().now().nanoseconds * 1e-9
        self.actualizar_obstaculos(x, y, th, ahora_s)
        if self.estado in ('GIRAR', 'AVANZAR'):
            bloqueo = self.ruta_bloqueada(x, y)
            if bloqueo is not None and self.reloc_adelantable(x, y):
                self.reloc_adelantada = True
                self.get_logger().warn(f'Bloqueo en ({bloqueo[0]:.2f}, {bloqueo[1]:.2f}) junto a la puerta: relocalizo aquí')
                self.escribir_log(x, y, th, 'RELOC_ADELANTADA', None, bloqueo[0], bloqueo[1])
                self.empezar_reloc('puerta')
            elif bloqueo is not None:
                self.get_logger().warn(f'Ruta bloqueada por un obstáculo en ({bloqueo[0]:.2f}, {bloqueo[1]:.2f}): espero')
                self.estado = 'ESPERAR'
                self.t_espera = ahora_s
                self.escribir_log(x, y, th, 'BLOQUEO', None, bloqueo[0], bloqueo[1])

        # 3. Lo que querría hacer la máquina de estados
        if self.estado == 'ESPERAR':
            v, w = 0.0, 0.0
            self.esperar(x, y, th, ahora_s)
        elif self.estado == 'RELOC':
            v, w = 0.0, 0.0
            self.relocalizar(x, y, th)
        else:
            v, w = self.maquina_estados(x, y, th)

        # 4. Parada de emergencia, por encima de cualquier estado (y del planificador). Solo anula la lineal:
        #    si también bloqueara el giro, mirando a una pared cercana nunca podría girarse para salir.
        obstaculo = self.obstaculo_delante(x, y, th) if v > 0.0 else None
        if obstaculo is not None:
            v = 0.0
            if not self.frenado:
                self.frenado = True
                self.get_logger().warn(f'PARADA DE EMERGENCIA: obstáculo en ({obstaculo[0]:.2f}, {obstaculo[1]:.2f})')
                self.escribir_log(x, y, th, 'PARADA_EMERGENCIA', None, obstaculo[0], obstaculo[1])
        elif self.frenado:
            self.frenado = False
            self.get_logger().info('Cono libre: sigo')
            self.escribir_log(x, y, th, 'FIN_PARADA')

        v = rampa(v, self.v_anterior, self.acel_max * self.dt_control)
        self.publicar_vel(v, w)
        self.w_anterior = w
        self.v_anterior = v

        # 5. Fila periódica del log
        ahora = self.get_clock().now()
        if self.t_ultimo_log is None or (ahora - self.t_ultimo_log).nanoseconds * 1e-9 >= self.log_period:
            self.t_ultimo_log = ahora
            self.escribir_log(x, y, th, '')

    def maquina_estados(self, x, y, th):
        # Devuelve (v, w) según el estado, y cambia de estado y de objetivo cuando toca.
        if self.estado == 'FIN':
            return 0.0, 0.0

        if self.estado == 'ORIENTAR':
            error = normalizar(self.pose_final[2] - th)
            if abs(error) < self.tol_final_yaw:
                self.estado = 'FIN'
                self.get_logger().info('FASE I COMPLETADA')
                self.escribir_log(x, y, th, 'FASE I COMPLETADA', error)
                return 0.0, 0.0
            return 0.0, limitar(self.k_angular * error, self.max_w)

        # LLEGADO: se comprueba antes que nada, esté girando o avanzando
        nombre, ox, oy, tol = self.objetivos[self.i_objetivo]
        dist = math.hypot(ox - x, oy - y)
        es_q = nombre == self.reloc_q_en or self.sustitutos.get(nombre, ('',))[0] == self.reloc_q_en   # también Q~
        if dist < tol and es_q and self.reloc_q_hechas < self.reloc_q_veces:
            # En Q: relocalizar con la esquina antes de darlo por alcanzado; luego se vuelve a ir a Q con la pose nueva
            self.reloc_q_hechas += 1
            self.get_logger().info(f'Relocalizo en {nombre} ({self.reloc_q_hechas}/{self.reloc_q_veces}): '
                                   f'junto {self.reloc_n} scans parado')
            self.escribir_log(x, y, th, f'LLEGADA_RELOC_Q {self.reloc_q_hechas}', dist)
            self.empezar_reloc('Q')
            return 0.0, 0.0
        if dist < tol:
            self.get_logger().info(f'Waypoint {nombre} alcanzado (error {dist:.3f} m)')
            self.escribir_log(x, y, th, f'WAYPOINT {nombre}', dist)
            if nombre in self.sustitutos:
                real, rx_, ry_ = self.sustitutos[nombre]
                d_real = math.hypot(rx_ - x, ry_ - y)
                self.get_logger().warn(f'{real} tapado: dado por alcanzado a {d_real:.2f} m')
                self.escribir_log(x, y, th, f'OBLIGATORIO_TAPADO {real} {d_real:.3f}', d_real)
            self.i_objetivo += 1
            self.estado = 'ORIENTAR' if self.i_objetivo == len(self.objetivos) else 'GIRAR'
            # Último waypoint antes de reloc_before (Door), se llame como se llame (también uno replanificado)
            if self.i_objetivo < len(self.objetivos) and self.objetivos[self.i_objetivo][0] == self.reloc_antes_de:
                self.empezar_reloc('puerta')
                self.get_logger().info(f'Relocalizo en {nombre}: junto {self.reloc_n} scans parado')
            return 0.0, 0.0

        # Error de ángulo hacia el objetivo
        error = normalizar(math.atan2(oy - y, ox - x) - th)
        w = limitar(self.k_angular * error, self.max_w)

        # GIRAR: sobre sí mismo hasta quedar casi alineado (el TB3 no puede ir de lado)
        if self.estado == 'GIRAR':
            if abs(error) >= self.angulo_girar:
                return 0.0, w
            self.estado = 'AVANZAR'

        # AVANZAR: v proporcional a la distancia y w corrige el rumbo.
        # Umbral de salida (30°) mayor que el de entrada (10°): histéresis, para no alternar en cada ciclo.
        if abs(error) > self.angulo_reorientar:
            self.estado = 'GIRAR'
            return 0.0, w
        return min(self.k_lineal * dist, self.max_v), w

    # ---------- Salidas ----------

    def publicar_vel(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # reloj del nodo: vale con use_sim_time
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.publisher.publish(msg)

    def yaws_log(self):
        # Rumbo según /odom y según el giróscopo, ambos en el sistema del mundo (para compararlos)
        thi = self.pose_inicial[2]
        return round(normalizar(thi + self.yaw_odom), 3), round(normalizar(thi + self.yaw_imu), 3)

    def escribir_log(self, x, y, th, evento, error=None, ox=None, oy=None):
        def r3(v):
            return '' if v is None else round(v, 3)
        t = self.get_clock().now().nanoseconds * 1e-9  # en Gazebo, tiempo simulado
        self.csv.writerow([round(t, 3), 'I', r3(x), r3(y), r3(th), self.estado, evento,
                           r3(error), r3(ox), r3(oy), *self.yaws_log()])
        self.fichero.flush()  # que no se pierda nada con un Ctrl+C


def main(args=None):
    # Sin el manejador de Ctrl+C de rclpy: así el contexto de ROS sigue vivo en el finally
    # y la velocidad cero llega de verdad al robot (con el de la plantilla se puede perder).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Fase1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publicar_vel(0.0, 0.0)
        node.fichero.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
