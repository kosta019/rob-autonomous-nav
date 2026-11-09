# Nodo de la misión: sigue la lista de waypoints con parada de emergencia y Mission Log, detecta obstáculos y
# pilares con el LiDAR y replanifica alrededor de ellos (fases I, II y III).
# Sigue la estructura: una clase Node, publicadores y suscriptores en __init__,
# callbacks y main(). El control va en un timer a ritmo fijo, para detectar si un sensor deja de llegar.
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
from autonomous_nav import pilares as pil
from autonomous_nav import planificador as pl

BLOQUEO_ACERCARSE = 0.005   # m: dentro del inflado, la ruta bloquea si se acerca al obstáculo más que esto


def normalizar(a):
    # Ángulo en (-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_cuaternion(q):
    # Yaw a partir del cuaternión, sin tf_transformations
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def limitar(valor, maximo):
    return max(-maximo, min(maximo, valor))


def rayos_libres(rayos, d, margen, dist_min_cero):
    # Dice si los rayos que apuntan a un obstáculo recordado (a distancia d del LiDAR) lo ven libre.
    # Un rayo es libre si da inf, una distancia mayor que d + margen, o 0.0 con el obstáculo lejos. El LiDAR real da
    # 0.0 también cuando algo está más cerca que range_min, por eso un 0.0 solo vale si d > dist_min_cero.
    for r in rayos:
        if r == math.inf or (math.isfinite(r) and r > d + margen) or (r == 0.0 and d > dist_min_cero):
            continue
        return False
    return True


def punto_sustituto(holgura, meta, ox, oy, inflado, radio):
    # Para un obligatorio (ox, oy) tapado, busca la celda libre (holgura > inflado) más cercana a menos de 'radio'.
    # Devuelve su centro (x, y) o None si no hay ninguna.
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
    # Relocalización con la pared de la puerta, con el robot parado. px, py son los puntos del scan en el mundo con la
    # pose creída y (rx, ry) la posición creída. x_pared e y_borde son referencias calibradas con cinta: la X a la que
    # el LiDAR ve la pared (la canaleta) y la Y del lateral del marco. El rumbo y la X salen de la recta de la pared;
    # la Y, del marco (o del final de la canaleta si no se ve). El ancho del hueco es solo un aviso.
    # Devuelve (dx, dy, dth, nivel, motivo, datos), con la corrección a sumar a la pose creída. El nivel es 'total',
    # 'parcial' (sin la Y, que no es fiable) o '' (la recta no es fiable).
    datos = {'n': 0, 'rms': None, 'x_linea': None, 'y1': None, 'y2': None, 'ancho': None, 'aviso_ancho': False,
             'y_fin': None}
    en_banda = np.abs(px - x_pared) < banda
    sel = en_banda & (py > y_borde - largo - max_dy) & (py < y_borde + max_dy + 0.10)
    x, y = px[sel], py[sel]
    datos['n'] = int(len(x))
    if len(x) < min_puntos:
        return 0.0, 0.0, 0.0, '', 'pocos_puntos', datos

    def mejor_recta(qx, qy):
        # Entre las rectas x = a + b*y que pasan por dos puntos separados más de 0.3 m en y (y con menos de 6° de
        # pendiente), elige la que tiene más puntos a menos de 2 cm: la canaleta. Un ajuste de mínimos cuadrados con
        # todos los puntos saldría torcido por el escalón de yeso y el marco.
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
        # Puntos a menos de 2 cm de la recta y, de ellos, el tramo continuo más largo (sin saltos de más de 10 cm en y).
        # Así no entran el escalón de yeso, el marco ni algo que haya en el hueco.
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
        # Lateral del marco: puntos más de fondo_marco por detrás de la recta (x = xl), donde acaba la pared
        m = (qx > xl + fondo_marco) & (qx < xl + banda) & (qy > yfin - 0.05) & (qy < yfin + 0.25)
        return float(np.median(qy[m])) if np.sum(m) >= 3 else None

    # Recta de partida con más puntos alineados y recta definitiva con su tramo continuo más largo
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
    # La recta debería ir en dirección Y: el rumbo se corrige girando los puntos alrededor del robot, y después la X
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
    # Borde cercano: el lateral del marco, donde acaba el tramo de pared
    qx, qy = girar(px[en_banda], py[en_banda])
    yfin = float(np.max(yr))
    datos['y_fin'] = yfin
    y1 = marco_y(datos['x_linea'], qx, qy, yfin)
    if y1 is None:
        # Sin marco se usa el final de la canaleta, que solo es creíble si justo después hay un escalón hacia atrás
        # (el yeso). Basta un punto: de lado, el escalón da 2-3 rayos y el primero entra en la recta.
        escalon = ((qx > datos['x_linea'] + 0.02) & (qx < datos['x_linea'] + 0.20)
                   & (qy > yfin) & (qy < yfin + 0.20))
        if y_fin_canaleta is not None and np.sum(escalon) >= 1 and abs(y_fin_canaleta - yfin) <= max_dy:
            return dx, y_fin_canaleta - yfin, dth, 'total', 'canaleta', datos
        return dx, 0.0, dth, 'parcial', 'sin_borde', datos
    datos['y1'] = y1
    # Borde creíble: pared justo antes de y1 y nada sobre la línea de pared en los 0.5*ancho siguientes
    linea = np.abs(qx - datos['x_linea']) < 0.05
    antes = np.sum(linea & (qy > y1 - 0.5) & (qy <= y1))
    ocupado = np.sum(linea & (qy > y1 + 0.02) & (qy < y1 + 0.5 * ancho))
    # Ancho del hueco, solo como aviso: hasta el primer punto sobre la línea al otro lado (la bisagra de la hoja)
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
    # Corrección de la pose con el mapa, como la actualización del filtro de Kalman extendido: cada punto del LiDAR
    # cerca de una pared es una medida. 'mapa' tiene, por celda, la distancia a la pared y su gradiente.
    # Se resuelve por mínimos cuadrados que mover el robot (dx, dy, dth) lleve cada punto a distancia 0 de su pared,
    # con un término que mantiene la corrección cerca de 0 (confianza en la odometría).
    # Solo se usan los puntos a menos de 'ventana' de una pared, así que mesas y personas quedan fuera.
    # Devuelve (dx, dy, dth, n), con n los puntos usados; la corrección es 0 si son menos de min_puntos.
    res = meta['resolucion']
    info = np.diag([1.0 / sig_xy ** 2, 1.0 / sig_xy ** 2, 1.0 / sig_th ** 2])
    dx, dy, dth = 0.0, 0.0, 0.0
    n = 0
    for _ in range(3):                  # la normal de cada punto cambia al mover la pose
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


def ajuste_paredes(px, py, paredes, ventana, sig_medida, sig_xy, min_puntos):
    # Corrección de la pose en el pasillo con sus paredes medidas con cinta, como un filtro de Kalman en 1D que combina
    # la odometría con la pared. px, py son los puntos del LiDAR en el mundo con la pose creída. paredes es una lista de
    # (eje, valor, desde, hasta): eje 'y' es la recta y = valor para x entre desde y hasta (corrige la Y), y eje 'x'
    # la recta x = valor para y entre desde y hasta (corrige la X). El rumbo no se toca, porque el del IMU es mejor.
    # Cada punto a menos de 'ventana' de una pared mide cuánto se ha desplazado el robot: e = valor - punto.
    # Con n medidas de varianza sig_medida² y la odometría de varianza sig_xy²:
    #     d = (suma de e / sig_medida²) / (n / sig_medida² + 1 / sig_xy²)
    # Devuelve (dx, dy, nx, ny): la corrección y los puntos de cada eje (con menos de min_puntos, la corrección es 0).
    suma = {'x': 0.0, 'y': 0.0}
    n = {'x': 0, 'y': 0}
    for eje, valor, desde, hasta in paredes:
        if eje == 'y':
            cerca, a_lo_largo = py, px
        else:
            cerca, a_lo_largo = px, py
        sel = (np.abs(cerca - valor) < ventana) & (a_lo_largo >= desde) & (a_lo_largo <= hasta)
        suma[eje] += float(np.sum(valor - cerca[sel]))
        n[eje] += int(np.sum(sel))
    d = {}
    for eje in ('x', 'y'):
        if n[eje] < min_puntos:
            d[eje] = 0.0
        else:
            d[eje] = (suma[eje] / sig_medida ** 2) / (n[eje] / sig_medida ** 2 + 1.0 / sig_xy ** 2)
    return d['x'], d['y'], n['x'], n['y']


def revisar_pose_inicial(pose, estimada, resumen, eje_rumbo, min_puntos, max_rms, max_corr, max_ang):
    # Compara la pose de salida dada (initial_pose) con la estimada con las paredes del mapa (comprobar_pose.estimar)
    # y decide qué se aplica. El rumbo solo se corrige si se han visto paredes del eje_rumbo ('' = cualquiera), y cada
    # coordenada solo si se han visto paredes de ese eje (en B y C no se ve la pared del fondo y la X se queda).
    # Devuelve (x, y, th, motivo): motivo '' si vale; si no, por qué no (y la pose dada sin tocar).
    x, y, th = pose
    ejes = {c['eje'] for c in resumen}
    n = sum(c['n'] for c in resumen)
    if not resumen or (eje_rumbo and eje_rumbo not in ejes):
        return x, y, th, 'sin_paredes_rumbo'
    if n < min_puntos:
        return x, y, th, 'pocos_puntos'
    if max(c['rms'] for c in resumen) > max_rms:
        return x, y, th, 'residuo'
    nx = estimada[0] if 'X' in ejes else x
    ny = estimada[1] if 'Y' in ejes else y
    nth = normalizar(estimada[2])
    if math.hypot(nx - x, ny - y) > max_corr or abs(normalizar(nth - th)) > max_ang:
        return x, y, th, 'grande'
    return nx, ny, nth, ''


def quitar_hasta(objetivos, nombre):
    # Devuelve los objetivos sin los que van hasta 'nombre' inclusive. Con '' o un nombre que no está, devuelve todos.
    nombres = [o[0] for o in objetivos]
    return objetivos[nombres.index(nombre) + 1:] if nombre in nombres else objetivos


def rampa(v, v_anterior, paso):
    # Al acelerar, como mucho 'paso' más que la velocidad anterior; al frenar no hay límite
    return min(v, v_anterior + paso) if v > v_anterior else v


def longitud_camino(puntos):
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(puntos, puntos[1:])))


def rodeo_largo(nuevo, antes, ratio, extra):
    # Un camino nuevo mucho más largo que el que quedaba es un rodeo largo: si el obstáculo se mueve, mejor esperar.
    return nuevo > ratio * antes + extra


def recta_ransac(u, v, max_pendiente, tol=0.02, sep=0.20):
    # Recta v = a + b*u con más puntos a menos de tol (probando pares separados más de sep en u, con |b| <=
    # max_pendiente), reajustada por mínimos cuadrados con esos puntos. Devuelve (a, b, máscara de puntos) o None.
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
    # Relocalización con dos paredes perpendiculares medidas con cinta (la esquina junto a Q: x = x_pared delante e
    # y = y_pared a un lado). No usa el mapa, que detrás de la puerta está mal en Y.
    # px, py son las lecturas en el mundo con la pose creída (rx, ry). De cada pared se toman los puntos a menos de
    # 'banda' de su recta y a menos de 'alcance' del robot, y se ajusta una recta con RANSAC. El rumbo sale de las
    # rectas y, girando los puntos alrededor del robot, la distancia a cada pared da dx y dy.
    # eje_rumbo: 'Y' usa solo la pared y = y_pared para el rumbo, 'X' solo la otra y '' las dos.
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

        # Parámetros, sin valores por defecto: vienen de config/fase1.yaml y, si falta alguno, el nodo falla al arrancar
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
        # Rampa de aceleración (solo al acelerar). Sin ella, el tirón al arrancar hace saltar el rumbo del IMU
        self.acel_max = self.declare_parameter('max_linear_accel', D).value
        self.tol_waypoint = self.declare_parameter('tol_waypoint', D).value
        self.tol_obligatorio = self.declare_parameter('tol_mandatory', D).value
        self.tol_final_yaw = self.declare_parameter('tol_final_yaw', D).value
        self.max_v = self.declare_parameter('max_linear_speed', D).value
        self.max_w = self.declare_parameter('max_angular_speed', D).value
        self.k_lineal = self.declare_parameter('k_linear', D).value
        self.v_paso = self.declare_parameter('phase1_pass_speed', D).value    # 0 = para en cada punto, como antes
        self.k_angular = self.declare_parameter('k_angular', D).value
        self.angulo_girar = self.declare_parameter('turn_in_place_angle', D).value
        self.angulo_reorientar = self.declare_parameter('reorient_angle', D).value

        self.stop_distance = self.declare_parameter('stop_distance', D).value
        self.cono = math.radians(self.declare_parameter('stop_half_angle_deg', D).value)
        self.timeout = self.declare_parameter('scan_timeout', D).value
        self.lidar_offset_x = self.declare_parameter('lidar_offset_x', D).value
        self.lidar_yaw_offset = self.declare_parameter('lidar_yaw_offset', D).value
        # Relocalización con la pared de la puerta, en el último waypoint antes de reloc_before
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
        # Relocalización en Q con las dos paredes de la esquina, al llegar
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
        # Relocalización en P Base al acabar la fase II, parado y mirando a +X (igual que al calibrarla)
        self.reloc_p_veces = self.declare_parameter('reloc_pbase_times', Parameter.Type.INTEGER).value   # 0 = apagada
        self.reloc_p_pared_x = self.declare_parameter('reloc_pbase_wall_x', D).value
        self.reloc_p_pared_y = self.declare_parameter('reloc_pbase_wall_y', D).value
        self.reloc_p_banda = self.declare_parameter('reloc_pbase_band', D).value
        self.reloc_p_alcance = self.declare_parameter('reloc_pbase_span', D).value
        self.reloc_p_max_rms = self.declare_parameter('reloc_pbase_max_rms', D).value
        self.reloc_p_min_puntos = self.declare_parameter('reloc_pbase_min_points', Parameter.Type.INTEGER).value
        self.reloc_p_max_corr = self.declare_parameter('reloc_pbase_max_corr', D).value
        self.reloc_p_max_ang = math.radians(self.declare_parameter('reloc_pbase_max_angle_deg', D).value)
        self.reloc_p_max_dif = math.radians(self.declare_parameter('reloc_pbase_max_angle_diff_deg', D).value)
        self.reloc_p_hechas = 0
        # Corrección continua con el mapa de distancias. Modo: "apagado", "sombra" (solo registra) o "activo"
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
        # Corrección en el pasillo con sus paredes (ajuste_paredes), con el robot dentro de corridor_corr_zone.
        # Usa el mismo modo y los mismos parámetros que map_corr_*.
        self.pasillo_modo = self.declare_parameter('corridor_corr_mode', Parameter.Type.STRING).value
        self.pasillo_zona = self.declare_parameter('corridor_corr_zone', DA).value   # [x_min, x_max, y_min, y_max]
        self.pasillo_alcance = self.declare_parameter('corridor_corr_max_range', D).value
        ejes = self.declare_parameter('corridor_wall_axis', SA).value
        valores = self.declare_parameter('corridor_wall_value', DA).value
        desde = self.declare_parameter('corridor_wall_from', DA).value
        hasta = self.declare_parameter('corridor_wall_to', DA).value
        if not (len(ejes) == len(valores) == len(desde) == len(hasta)) or any(e not in ('x', 'y') for e in ejes):
            raise ValueError('corridor_wall_*: misma longitud, y cada eje "x" o "y"')
        self.pasillo_paredes = list(zip(ejes, valores, desde, hasta))
        self.usar_imu = self.declare_parameter('use_imu_yaw', Parameter.Type.BOOL).value
        log_file = self.declare_parameter('log_file', Parameter.Type.STRING).value
        self.log_period = self.declare_parameter('log_period', D).value

        # Obstáculos nuevos y replanificación
        self.margen_estatico = self.declare_parameter('obstacle_static_margin', D).value
        self.margen_plan = self.declare_parameter('obstacle_plan_margin', D).value
        self.memoria = self.declare_parameter('obstacle_memory_time', D).value
        self.margen_borrado = self.declare_parameter('obstacle_clear_margin', D).value
        self.escaneos_borrado = self.declare_parameter('obstacle_clear_scans', Parameter.Type.INTEGER).value
        self.dist_min_cero = self.declare_parameter('obstacle_clear_zero_min_dist', D).value
        self.separacion_log = self.declare_parameter('obstacle_log_spacing', D).value
        self.alcance_obstaculos = self.declare_parameter('obstacle_max_range', D).value
        # Más cerca que esto no se recuerda nada (será el propio robot o un eco). La parada de emergencia no lo usa.
        self.alcance_min_obstaculos = self.declare_parameter('obstacle_min_range', D).value
        self.giro_max_obstaculos = self.declare_parameter('obstacle_max_turn_rate', D).value
        # Pilares de las estaciones de carga (fase II, pilares.py)
        self.zona_pilares = self.declare_parameter('pillar_zone', DA).value   # [x_min, x_max, y_min, y_max]
        self.pilar_alcance = self.declare_parameter('pillar_max_range', D).value
        self.pilar_min_det = self.declare_parameter('pillar_min_detections', Parameter.Type.INTEGER).value
        self.pilar_olvido = self.declare_parameter('pillar_forget_time', D).value
        self.estacion_lado = self.declare_parameter('station_side', D).value
        self.estacion_tol = self.declare_parameter('station_tol', D).value
        self.estacion_confirmar = self.declare_parameter('station_confirm_scans', Parameter.Type.INTEGER).value
        self.alcance_bloqueo = self.declare_parameter('block_lookahead', D).value
        self.histeresis = self.declare_parameter('block_hysteresis', D).value
        self.espera = self.declare_parameter('obstacle_wait_time', D).value
        self.reintento = self.declare_parameter('replan_retry_period', D).value
        # Un camino nuevo más largo que replan_max_ratio * el que queda + replan_max_extra no se toma hasta llevar
        # replan_long_detour_after s sin camino, porque un robot o una persona en el pasillo se acaba yendo
        self.rodeo_ratio = self.declare_parameter('replan_max_ratio', D).value
        self.rodeo_extra = self.declare_parameter('replan_max_extra', D).value
        self.rodeo_espera = self.declare_parameter('replan_long_detour_after', D).value
        # Obligatorio tapado: tras mandatory_cover_wait s sin camino, va al punto libre más cercano (a menos de
        # mandatory_cover_radius) y lo da por alcanzado (OBLIGATORIO_TAPADO), salvo los de mandatory_cover_exclude
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
        # Anclas: puntos de la secuencia (B, D...) por los que hay que pasar aunque no sean obligatorios,
        # porque rodean las mesas, que no están en el mapa
        self.anclas = self.declare_parameter('planning_sequence', SA).value
        self.obligatorios = obligatorios

        # Holgura estática de la rejilla de 5 cm y sus metadatos
        ruta = self.declare_parameter('static_clearance_file', Parameter.Type.STRING).value
        if not os.path.isabs(ruta):
            ruta = os.path.join(get_package_share_directory('autonomous_nav'), ruta)
        self.holgura = np.load(ruta)
        with open(ruta.replace('.npy', '.yaml')) as f:
            m = yaml.safe_load(f)
        self.meta = {'resolucion': m['resolucion'], 'origen_x': m['origen_x'], 'origen_y': m['origen_y']}

        # Mapa de distancias para la corrección con el mapa (lo genera generar_mapa_distancias.py)
        self.corr_ultimo_scan = None
        self.corr_cuenta = 0
        if self.corr_modo != 'apagado':
            ruta = self.declare_parameter('map_corr_file', Parameter.Type.STRING).value
            if not os.path.isabs(ruta):
                ruta = os.path.join(get_package_share_directory('autonomous_nav'), ruta)
            self.corr_mapa = np.load(ruta)
            with open(ruta.replace('.npy', '.yaml')) as f:
                self.corr_meta = yaml.safe_load(f)

        # Lista de objetivos: (nombre, x, y, tolerancia). Al final, la posición de la pose final
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
        self.fase = 'I'             # current_phase del Mission Log
        # Fase II (phase2_enabled): al acabar la fase I espera phase2_start_delay s, recorre la ruta de cobertura del
        # pasillo y vuelve a P Base (phase2_final_pose)
        self.fase2_activa = self.declare_parameter('phase2_enabled', Parameter.Type.BOOL).value
        self.fase2_nombres = self.declare_parameter('phase2_waypoint_names', SA).value
        self.fase2_x = self.declare_parameter('phase2_waypoints_x', DA).value
        self.fase2_y = self.declare_parameter('phase2_waypoints_y', DA).value
        self.fase2_final = self.declare_parameter('phase2_final_pose', DA).value
        self.fase2_espera = self.declare_parameter('phase2_start_delay', D).value
        self.fase2_max_v = self.declare_parameter('phase2_max_linear_speed', D).value
        if not (len(self.fase2_nombres) == len(self.fase2_x) == len(self.fase2_y)):
            raise ValueError('phase2_waypoint_names, phase2_waypoints_x y phase2_waypoints_y: misma longitud')
        # Segunda cobertura: si tras la primera no se ha confirmado la estación cuadrada, estas pasadas antes de P Base
        self.fase2b_activa = self.declare_parameter('phase2_second_coverage', Parameter.Type.BOOL).value
        self.fase2b_nombres = self.declare_parameter('phase2_second_waypoint_names', SA).value
        self.fase2b_x = self.declare_parameter('phase2_second_waypoints_x', DA).value
        self.fase2b_y = self.declare_parameter('phase2_second_waypoints_y', DA).value
        if not (len(self.fase2b_nombres) == len(self.fase2b_x) == len(self.fase2b_y)):
            raise ValueError('phase2_second_waypoint_names, _x y _y: misma longitud')
        self.cobertura = 1          # qué cobertura del pasillo se está haciendo (fase II)
        # Fase III (phase3_enabled): de P Base a delante de la estación cuadrada, mide sus patas parado, entra recto
        # hasta su centro y comprueba el aparcado (estados MEDIR3, ENTRAR y VERIFICAR3)
        self.fase3_activa = self.declare_parameter('phase3_enabled', Parameter.Type.BOOL).value
        self.f3_dist = self.declare_parameter('phase3_front_dist', D).value
        self.f3_n = self.declare_parameter('phase3_scans', Parameter.Type.INTEGER).value
        self.f3_max_desvio = self.declare_parameter('phase3_max_offset', D).value
        self.f3_intentos_max = self.declare_parameter('phase3_attempts', Parameter.Type.INTEGER).value
        self.f3_max_desalineado = math.radians(self.declare_parameter('phase3_max_misalign_deg', D).value)
        self.f3_espera_lado = self.declare_parameter('phase3_side_wait', D).value
        self.f3_v = self.declare_parameter('phase3_speed', D).value
        self.f3_tol = self.declare_parameter('phase3_tol', D).value
        self.f3_radio_pata = self.declare_parameter('phase3_leg_clearance', D).value
        self.f3_tol_aparcado = self.declare_parameter('phase3_park_tol', D).value
        self.f3_scans = []          # pilares vistos (frame del LiDAR) en cada scan, parado
        self.f3_ultimo_scan = None
        self.f3_intentos = 0
        self.f3_centro = None       # objetivo de ENTRAR en el mundo (el centro de la estación medido desde delante)
        self.f3_patas = []          # patas de la estación en el mundo: la parada de emergencia no para por ellas
        self.f3_estacion = None     # (cx, cy, ang) de la estación a la que se va
        self.f3_lados_descartados = []   # puntos de delante sin camino: no se vuelven a elegir
        self.t_pausa = None
        # Salida desde A, B, C o D. start_after quita los waypoints hasta ese inclusive. Con plan_at_start, el robot
        # parado junta start_scans scans para llenar la memoria de obstáculos y planifica desde la pose inicial.
        despues_de = self.declare_parameter('start_after', Parameter.Type.STRING).value
        self.planificar_al_salir = self.declare_parameter('plan_at_start', Parameter.Type.BOOL).value
        self.scans_salida = self.declare_parameter('start_scans', Parameter.Type.INTEGER).value
        self.plan_final_dist = self.declare_parameter('final_plan_dist', D).value
        self.objetivos = quitar_hasta(self.objetivos, despues_de)
        if self.planificar_al_salir:
            self.estado = 'INICIO'
        self.salida_scans = 0
        self.salida_ultimo_scan = None
        # Pose de salida con el LiDAR (estado POSE_INICIAL): el robot se coloca a ojo y, parado, el nodo corrige el
        # rumbo y la posición con las paredes del mapa. Si no vale, no se mueve y reintenta con scans nuevos.
        self.pi_activa = self.declare_parameter('start_pose_check', Parameter.Type.BOOL).value
        self.pi_n = self.declare_parameter('start_pose_scans', Parameter.Type.INTEGER).value
        self.pi_min_puntos = self.declare_parameter('start_pose_min_points', Parameter.Type.INTEGER).value
        self.pi_max_rms = self.declare_parameter('start_pose_max_rms', D).value
        self.pi_max_corr = self.declare_parameter('start_pose_max_corr', D).value
        self.pi_max_ang = math.radians(self.declare_parameter('start_pose_max_angle_deg', D).value)
        self.pi_eje_rumbo = self.declare_parameter('pose_heading_axis', Parameter.Type.STRING).value
        self.pi_scans = []          # scans juntados en POSE_INICIAL (nan = sin dato)
        self.pi_wz = []             # y, mientras, el giro de /imu y el rumbo de /odom (con el robot quieto, ~0)
        self.pi_yaws = []
        self.pi_ultimo_scan = None
        self.estado_tras_pose = self.estado   # INICIO o GIRAR
        if self.pi_activa:
            self.caras = cp.cargar_caras()
            self.estado = 'POSE_INICIAL'

        # Estimación de la pose (odometría y giróscopo) y cuándo llegó cada sensor (reloj del nodo)
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

        # Banderas para registrar los eventos solo cuando cambian
        self.sin_sensores = True
        self.frenado = False
        self.t_ultimo_log = None

        # Memoria de obstáculos nuevos: cada celda de 5 cm guarda [x, y, última vez visto (s), scans seguidos viéndolo libre]
        self.obstaculos = {}
        self.scan_borrado = None    # último scan con el que se contaron esos escaneos (se cuenta una vez por scan)
        self.obstaculos_log = []    # posiciones ya escritas en el Mission Log
        self.scan_pilares = None    # último scan pasado por el detector de pilares (uno por scan)
        self.grupos_pilares = []    # [x, y, n, t_ultimo] de cada pilar visto hace poco (pilares.agrupar)
        self.estaciones = []        # estaciones encontradas: {'tipo', 'x', 'y', 'ang', 'patas', 'vistas'}
        self.t_espera = None        # cuándo empezó la espera en ESPERAR
        self.n_replan = 0
        self.w_anterior = 0.0       # última velocidad angular mandada
        self.v_anterior = 0.0       # última velocidad lineal mandada (rampa de aceleración)
        self.reloc_scans_lista = [] # scans juntados en RELOC
        self.t_sin_camino = None    # desde cuándo no hay camino (para el obligatorio tapado)
        self.sustitutos = {}        # sustituto: (obligatorio, x, y)
        self.reloc_ultimo_scan = None
        self.reloc_adelantada = False   # ya se ha relocalizado por un bloqueo antes del waypoint (solo una vez)
        self.reloc_tipo = 'puerta'      # relocalización del estado RELOC: 'puerta' (ajuste_puerta) o 'Q' (esquina)
        self.reloc_puerta_total = False # ya hubo una relocalización total en la puerta (si no, se reintenta)
        self.reloc_reintentos = 0
        self.reloc_q_hechas = 0
        self.rodeo_rechazado = None     # (longitud nueva, la que quedaba) del último rodeo no aceptado

        # Publicador y suscriptores, con el QoS de la plantilla
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_profile_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_r)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_b)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_b)  # se registra siempre en el log

        # Mission Log: un fichero por ejecución (con la fecha) para no machacar el anterior
        base, ext = os.path.splitext(log_file)
        ruta_log = f"{base}_{datetime.now().strftime('%Y%m%d_%H%M%S')}{ext}"
        self.fichero = open(ruta_log, 'w', newline='')
        self.csv = csv.writer(self.fichero)
        self.csv.writerow(['timestamp', 'current_phase', 'robot_x', 'robot_y', 'robot_yaw',
                           'state', 'event', 'error', 'obstacle_x', 'obstacle_y', 'yaw_odom', 'yaw_imu'])
        self.get_logger().info(f'Mission Log en {os.path.abspath(ruta_log)}')

        # Bucle de control a ritmo fijo
        self.create_timer(1.0 / control_rate, self.control_loop)

    # Callbacks
    # /scan solo guarda el último mensaje. /odom y /imu además se integran aquí, porque integrar no admite saltarse
    # mensajes.

    def odom_callback(self, msg):
        p = msg.pose.pose
        x, y, th = p.position.x, p.position.y, yaw_de_cuaternion(p.orientation)
        if self.odom_anterior is not None:
            xa, ya, tha = self.odom_anterior
            # Avance (con signo) desde la lectura anterior, en la dirección en que iba según /odom.
            # De /odom solo se usa la distancia: el rumbo, si se puede, lo da el giróscopo.
            self.yaw_odom += normalizar(th - tha)
            ds = (x - xa) * math.cos(tha) + (y - ya) * math.sin(tha)
            th_antes = self.th_rel
            self.th_rel = self.yaw_imu if self.usar_imu else self.th_rel + normalizar(th - tha)
            th_medio = th_antes + 0.5 * normalizar(self.th_rel - th_antes)
            self.x_rel += ds * math.cos(th_medio)
            self.y_rel += ds * math.sin(th_medio)
        if self.estado == 'POSE_INICIAL':
            self.pi_yaws.append(th)
        self.odom_anterior = (x, y, th)
        self.t_odom = self.get_clock().now()

    def imu_callback(self, msg):
        # Rumbo integrando la velocidad angular del giróscopo, con el sello de tiempo del propio /imu para el dt
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
        if self.estado == 'POSE_INICIAL':
            self.pi_wz.append(msg.angular_velocity.z)
        self.t_imu = self.get_clock().now()

    def scan_callback(self, msg):
        self.scan = msg
        self.t_scan = self.get_clock().now()

    #  Pose

    def pose_mundo(self):
        # Pose (x, y, theta) del robot en el sistema del enunciado, o None si aún no hay /odom.
        # (x_rel, y_rel, th_rel) es lo que se ha movido desde la salida, con los ejes del robot al salir.
        if self.odom_anterior is None:
            return None
        # Al mundo: se gira theta_i y se traslada a (x_i, y_i)
        xi, yi, thi = self.pose_inicial
        x = xi + math.cos(thi) * self.x_rel - math.sin(thi) * self.y_rel
        y = yi + math.sin(thi) * self.x_rel + math.cos(thi) * self.y_rel
        return x, y, normalizar(thi + self.th_rel)

    #  Seguridad

    def sensor_caducado(self, t):
        # Devuelve True si el sensor no ha llegado nunca o lleva más de scan_timeout sin llegar
        if t is None:
            return True
        return (self.get_clock().now() - t).nanoseconds * 1e-9 > self.timeout

    def obstaculo_delante(self, x, y, th):
        # Busca la lectura válida más cercana del cono frontal a menos de stop_distance.
        # Devuelve su posición en el mundo (ox, oy), o None si el cono está libre.
        msg = self.scan
        r_min = None
        a_min = 0.0
        for i, r in enumerate(msg.ranges):
            # 0.0, inf, nan o fuera de [range_min, range_max] = sin dato
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            # Ángulo de cada lectura (sin suponer 360 muestras ni que el índice 0 sea el frente)
            a = normalizar(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= self.cono and r < self.stop_distance and (r_min is None or r < r_min):
                if self.estado == 'ENTRAR' and self.es_pata(x, y, th, r, a):
                    continue        # en la fase III las patas de la estación no paran al robot, pero lo demás sí
                r_min = r
                a_min = a
        if r_min is None:
            return None
        # El LiDAR está desplazado lidar_offset_x respecto al eje del robot
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        return lx + r_min * math.cos(th + a_min), ly + r_min * math.sin(th + a_min)

    def es_pata(self, x, y, th, r, a):
        # Dice si la lectura (r, a) del LiDAR cae a menos de phase3_leg_clearance de una pata conocida de la estación
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        px, py = lx + r * math.cos(th + a), ly + r * math.sin(th + a)
        return any(math.hypot(px - qx, py - qy) < self.f3_radio_pata for qx, qy in self.f3_patas)

    #  Obstáculos nuevos y replanificación

    def actualizar_obstaculos(self, x, y, th, ahora):
        # Memoria de obstáculos nuevos (lo que el LiDAR ve y el mapa no explica). Se añade cada lectura válida que cae
        # donde la holgura estática es mayor que obstacle_static_margin. Se olvida si lleva obstacle_memory_time sin
        # verse o si un rayo del LiDAR pasa a través de él, pero no por dejar de verlo al girar.
        # Girando deprisa no se toca la memoria: el scan llega con retraso y las paredes saldrían desplazadas.
        if abs(self.w_anterior) > self.giro_max_obstaculos:
            return
        msg = self.scan
        r = np.array(msg.ranges, dtype=float)
        n = len(r)
        a = msg.angle_min + np.arange(n) * msg.angle_increment
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)

        # Lecturas válidas y cercanas en el mundo, y las que no explica el mapa
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

    def detectar_pilares(self, x, y, th, ahora):
        # Fase II: detecta las patas de las estaciones (pilares.py), una vez por scan nuevo y sin girar deprisa.
        # Solo cuentan los pilares dentro de pillar_zone (el pasillo), porque en la sala las patas de sillas y mesas
        # podrían formar un cuadrado de 44 cm.
        msg = self.scan
        if msg is self.scan_pilares or abs(self.w_anterior) > self.giro_max_obstaculos:
            return
        self.scan_pilares = msg
        vistos = pil.pilares_scan(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                                  alcance=self.pilar_alcance)
        x0, x1, y0, y1 = self.zona_pilares
        puntos = [p for p in pil.a_mundo(vistos, (x, y, th), self.lidar_offset_x)
                  if x0 <= p[0] <= x1 and y0 <= p[1] <= y1]
        pil.agrupar(self.grupos_pilares, puntos, ahora)
        pil.olvidar(self.grupos_pilares, ahora, self.pilar_olvido)
        est = pil.buscar_estacion([(g[0], g[1]) for g in self.grupos_pilares if g[2] >= self.pilar_min_det],
                                  self.estacion_lado, self.estacion_tol)
        if est is None:
            return
        tipo, cx, cy, ang, patas = est
        # Si ya la conocemos (del mismo tipo y con el centro a menos de un lado), se actualiza
        for e in self.estaciones:
            if e['tipo'] == tipo and math.hypot(e['x'] - cx, e['y'] - cy) < self.estacion_lado:
                break
        else:
            e = {'vistas': 0}
            self.estaciones.append(e)
        # Se guarda la última estimación, porque los grupos ya promedian muchos scans
        e.update(tipo=tipo, x=cx, y=cy, ang=ang, patas=patas)
        e['vistas'] += 1
        # Al Mission Log, una vez, cuando se ha visto en station_confirm_scans scans: el centro (con el ángulo en grados
        # en la columna error) y cada pata. Las de la triangular van como PILAR_DESCARTADA
        if e['vistas'] == self.estacion_confirmar:
            self.get_logger().info(f'Estación {tipo} en ({cx:.2f}, {cy:.2f}), {math.degrees(ang):.1f}°')
            self.escribir_log(x, y, th, 'ESTACION_' + tipo.upper(), math.degrees(ang), cx, cy)
            for px, py in patas:
                self.escribir_log(x, y, th, 'PILAR' if tipo == 'cuadrada' else 'PILAR_DESCARTADA', None, px, py)

    def ruta_bloqueada(self, x, y, extra=0.0):
        # Busca un obstáculo nuevo a menos de inflation_radius de la ruta que queda (los próximos block_lookahead m) y
        # devuelve su posición, o None si la ruta está libre. 'extra' agranda las distancias: en ESPERAR se pide ese
        # margen de más para dar la ruta por libre (histéresis).
        # Solo se mira hasta el siguiente obligatorio (o su sustituto, o la pose final), porque el replan no llega más
        # lejos y un obstáculo junto al obligatorio bloquearía el tramo siguiente sin que el replan lo arreglase.
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
        # Un obstáculo que ya está a menos de inflation_radius del robot solo bloquea si la ruta se le acerca más.
        # Si no, cualquier camino que sale del robot pasaría a menos del inflado y replanificaría en bucle.
        d_robot = np.hypot(o[:, 0] - x, o[:, 1] - y)
        bloquea = (d < self.inflado + extra) & (d < d_robot - BLOQUEO_ACERCARSE)
        if not np.any(bloquea):
            return None
        k = int(np.argmin(np.where(bloquea, d, np.inf)))
        return o[k, 0], o[k, 1]

    def esperar(self, x, y, th, ahora):
        # Estado ESPERAR (robot parado). Si la ruta se libera, sigue con el plan que había; si a los
        # obstacle_wait_time s sigue bloqueada, replanifica.
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
            if self.cambiar_de_lado(x, y, th, ahora):
                return
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

    def empezar_fase2(self, x, y, th):
        # Objetivos de la fase II: la ruta de cobertura y P Base. Todos son anclas, para que un replan no se salte una
        # pasada. Solo se planifica el primer tramo, y solo si queda lejos: si se planificasen las pasadas, el
        # planificador las llevaría al centro del pasillo (busca holgura) y dejarían de cubrirlo.
        self.fase = 'II'
        self.cobertura = 1
        self.objetivos = [(n, px, py, self.tol_waypoint) for n, px, py in zip(self.fase2_nombres, self.fase2_x,
                                                                              self.fase2_y)]
        self.objetivos.append(('final', self.fase2_final[0], self.fase2_final[1], self.tol_obligatorio))
        self.i_objetivo = 0
        self.pose_final = self.fase2_final
        self.anclas = list(self.fase2_nombres)
        self.obligatorios = self.fase2_nombres[:1]   # replanificar va hasta el siguiente obligatorio
        self.max_v = self.fase2_max_v
        self.estado = 'GIRAR'
        self.get_logger().info(f'FASE II: {self.fase2_nombres} y P Base')
        self.escribir_log(x, y, th, 'FASE II INICIO')
        if self.fase2_nombres and math.hypot(self.fase2_x[0] - x, self.fase2_y[0] - y) > self.plan_final_dist:
            self.planificar_final(x, y, th, 'FASE2')

    def empezar_fase3(self, x, y, th):
        # Fase III: va a un punto delante de la estación cuadrada (a phase3_front_dist de su centro, perpendicular a uno
        # de sus lados) mirando a su centro. Al llegar y orientarse, pasa de ORIENTAR a MEDIR3.
        self.fase = 'III'
        self.f3_intentos = 0
        est = self.estacion_cuadrada()
        if est is None:
            self.get_logger().warn('FASE III: no hay estación cuadrada confirmada; no hago nada')
            self.escribir_log(x, y, th, 'FASE_III_SIN_ESTACION')
            self.estado = 'FIN'
            return
        self.escribir_log(x, y, th, 'FASE III INICIO', None, est['x'], est['y'])
        self.ir_delante(x, y, th, est['x'], est['y'], est['ang'], 'FASE3')

    def ir_delante(self, x, y, th, cx, cy, ang, nombre):
        # Objetivo: el punto a phase3_front_dist del centro (cx, cy), perpendicular al lado (ángulo ang + k·90°) más
        # cercano al robot que tenga holgura en el mapa, mirando al centro. Sin ninguno, FASE_III_SIN_HUECO y para.
        # La holgura incluye los obstáculos que ha visto el LiDAR: un panel justo delante de un lado lo descarta
        obst = [(v[0], v[1]) for v in self.obstaculos.values()]
        holgura = pl.holgura_con_obstaculos(self.holgura, self.meta, obst, self.margen_plan, self.holgura_objetivo)
        self.f3_estacion = (cx, cy, ang)
        delante = []
        for k in range(4):
            a = ang + k * math.pi / 2
            px, py = cx + self.f3_dist * math.cos(a), cy + self.f3_dist * math.sin(a)
            descartado = any(math.hypot(px - qx, py - qy) < 0.05 for qx, qy in self.f3_lados_descartados)
            if not descartado and pl.holgura_en(holgura, self.meta, px, py) > self.inflado:
                delante.append((math.hypot(px - x, py - y), px, py))
        if not delante:
            self.get_logger().warn('FASE III: ningún lado de la estación tiene sitio delante; no hago nada')
            self.escribir_log(x, y, th, 'FASE_III_SIN_HUECO', None, cx, cy)
            self.estado = 'FIN'
            return
        _, px, py = min(delante)
        self.objetivos = [('final', px, py, self.tol_obligatorio)]
        self.i_objetivo = 0
        self.pose_final = [px, py, math.atan2(cy - py, cx - px)]
        self.anclas, self.obligatorios = [], []
        self.max_v = self.fase2_max_v
        self.estado = 'GIRAR'
        self.get_logger().info(f'FASE III: a ({px:.2f}, {py:.2f}), delante de la estación ({cx:.2f}, {cy:.2f})')
        if math.hypot(px - x, py - y) > self.plan_final_dist:
            self.planificar_final(x, y, th, nombre)

    def cambiar_de_lado(self, x, y, th, ahora):
        # Fase III, yendo al punto de delante de la estación: si lleva phase3_side_wait s sin camino (un obstáculo lo
        # tapa), se descarta ese lado y se va al siguiente. Con phase3_side_wait 0 no se cambia nunca. Devuelve True si
        # ha cambiado de objetivo.
        if (self.fase != 'III' or self.f3_espera_lado <= 0.0 or self.f3_estacion is None
                or ahora - self.t_sin_camino < self.f3_espera_lado or not self.objetivos):
            return False
        px, py = self.objetivos[-1][1], self.objetivos[-1][2]
        self.f3_lados_descartados.append((px, py))
        self.get_logger().warn(f'FASE III: sin camino a ({px:.2f}, {py:.2f}); pruebo otro lado de la estación')
        self.escribir_log(x, y, th, 'FASE3_OTRO_LADO', None, px, py)
        self.t_sin_camino = None
        self.ir_delante(x, y, th, *self.f3_estacion, 'FASE3')
        return True

    def juntar_patas(self, max_rayos):
        # Con el robot parado, guarda los pilares de cada scan nuevo en el frame del LiDAR, sin usar la pose. Al juntar
        # phase3_scans scans los agrupa (un grupo es una pata) y devuelve (centro de la cuadrada o None, patas).
        # Antes de juntarlos devuelve None.
        msg = self.scan
        if msg is self.f3_ultimo_scan:
            return None
        self.f3_ultimo_scan = msg
        self.f3_scans.append(pil.pilares_scan(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                              msg.range_max, max_rayos=max_rayos))
        if len(self.f3_scans) < self.f3_n:
            return None
        grupos = []
        for k, vistos in enumerate(self.f3_scans):
            pil.agrupar(grupos, [(p[0], p[1]) for p in vistos], k)
        self.f3_scans = []
        patas = [(g[0], g[1]) for g in grupos if g[2] >= self.f3_n // 3]
        return pil.centro_cuadrada(patas, self.estacion_lado), patas

    def medir3(self, x, y, th):
        # Estado MEDIR3 (parado delante de la estación): mide el centro con sus patas, relativo al robot. Es el objetivo
        # de ENTRAR, así el aparcado no depende del error de la pose sino de la odometría del último metro.
        r = self.juntar_patas(8)
        if r is None:
            return
        centro, patas = r
        est = self.estacion_cuadrada()
        motivo, desvio = '', None
        if centro is None:
            motivo = 'sin_patas'
        else:
            cx, cy = pil.a_mundo([centro[:2]], (x, y, th), self.lidar_offset_x)[0]
            desvio = math.hypot(cx - est['x'], cy - est['y'])
            if desvio > self.f3_max_desvio:
                motivo = 'lejos'
        if not motivo:
            # Comprueba si entra perpendicular a un lado: ángulo entre la dirección al centro y la normal del lado
            ang = normalizar(centro[3] + th)               # ángulo de la estación en el mundo
            phi = math.atan2(cy - y, cx - x)
            desalineado = (phi - ang + math.pi / 4) % (math.pi / 2) - math.pi / 4
            if abs(desalineado) > self.f3_max_desalineado:
                self.f3_intentos += 1
                self.get_logger().warn(
                    f'FASE III: desalineado {math.degrees(desalineado):+.1f}° con la estación: me recoloco')
                self.escribir_log(x, y, th, f'FASE3_REALINEAR {math.degrees(desalineado):+.1f}', desvio, cx, cy)
                if self.f3_intentos >= self.f3_intentos_max:
                    self.escribir_log(x, y, th, 'FASE_III_ABANDONADA')
                    self.estado = 'FIN'
                    return
                self.ir_delante(x, y, th, cx, cy, ang, 'FASE3_REALINEAR')
                return
        if motivo:
            self.f3_intentos += 1
            self.get_logger().warn(f'FASE III: medida de las patas fallida ({motivo}, {len(patas)} patas), '
                                   f'intento {self.f3_intentos}/{self.f3_intentos_max}')
            self.escribir_log(x, y, th, f'FASE3_MEDIDA_FALLIDA {motivo} patas={len(patas)}', desvio)
            if self.f3_intentos >= self.f3_intentos_max:
                self.get_logger().warn('FASE III abandonada: no entro sin ver bien las patas')
                self.escribir_log(x, y, th, 'FASE_III_ABANDONADA')
                self.estado = 'FIN'
            return
        self.f3_centro = (cx, cy)
        # Solo se guarda lo que está donde debe estar una pata (a media diagonal del centro, ±5 cm), para que algo ajeno
        # dentro de la estación siga parando al robot
        semidiagonal = self.estacion_lado / math.sqrt(2)
        self.f3_patas = [p for p in pil.a_mundo(patas, (x, y, th), self.lidar_offset_x)
                         if abs(math.hypot(p[0] - cx, p[1] - cy) - semidiagonal) < 0.05]
        self.get_logger().info(f'FASE III: centro en ({cx:.3f}, {cy:.3f}) con {centro[2]} patas: entro')
        self.escribir_log(x, y, th, f'FASE3_CENTRO patas={centro[2]}', desvio, cx, cy)
        self.estado = 'ENTRAR'

    def entrar(self, x, y, th):
        # Estado ENTRAR: recto hasta el centro medido, despacio. Solo gira sobre sí mismo al principio, lejos de las
        # patas; dentro, corrige el rumbo avanzando. Para al llegar (phase3_tol) o si lo ha pasado. Devuelve (v, w).
        tx, ty = self.f3_centro
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < self.f3_tol or dx * math.cos(th) + dy * math.sin(th) <= 0.0:
            self.escribir_log(x, y, th, 'FASE3_DENTRO', dist)
            self.estado = 'VERIFICAR3'
            self.f3_scans = []
            return 0.0, 0.0
        error = normalizar(math.atan2(dy, dx) - th)
        if dist > 0.75 * self.f3_dist and abs(error) > self.angulo_girar:
            return 0.0, limitar(self.k_angular * error, self.max_w)
        w = limitar(self.k_angular * error, self.max_w) if dist > 0.05 else 0.0
        return min(max(self.k_lineal * dist, 0.02), self.f3_v), w

    def verificar3(self, x, y, th):
        # Estado VERIFICAR3 (parado, dentro): las patas alrededor dan la distancia del robot al centro de la estación.
        # Cada pata está a ~0.31 m y ocupa ~9 rayos, más de los 8 de la detección normal.
        r = self.juntar_patas(15)
        if r is None:
            return
        centro, patas = r
        self.estado = 'FIN'
        if centro is None:
            self.get_logger().warn(f'FASE III COMPLETADA, sin verificar (veo {len(patas)} patas)')
            self.escribir_log(x, y, th, f'FASE III COMPLETADA sin_verificar patas={len(patas)}')
            return
        error = math.hypot(centro[0] + self.lidar_offset_x, centro[1])
        self.get_logger().info(f'FASE III COMPLETADA: centro del robot a {error * 100:.1f} cm del de la estación')
        self.escribir_log(x, y, th, f'FASE III COMPLETADA patas={centro[2]}', error)
        if error > self.f3_tol_aparcado:
            self.escribir_log(x, y, th, 'APARCADO_FUERA_DE_TOLERANCIA', error)

    def estacion_cuadrada(self):
        # Devuelve la estación cuadrada (la del enunciado) si ya está confirmada (station_confirm_scans), o None
        return next((e for e in self.estaciones if e['tipo'] == 'cuadrada' and e['vistas'] >= self.estacion_confirmar),
                    None)

    def fin_de_cobertura(self, x, y, th):
        # Se llama al alcanzar el último punto de la cobertura (lo siguiente es P Base). Si no hay estación cuadrada
        # confirmada, hace la segunda cobertura (phase2_second_*) y, si tampoco, ESTACION_NO_ENCONTRADA y vuelve.
        if (self.fase != 'II' or self.i_objetivo >= len(self.objetivos) or self.objetivos[self.i_objetivo][0] != 'final'
                or self.estacion_cuadrada() is not None):
            return
        if self.cobertura == 1 and self.fase2b_activa:
            self.cobertura = 2
            nuevos = [(n, px, py, self.tol_waypoint)
                      for n, px, py in zip(self.fase2b_nombres, self.fase2b_x, self.fase2b_y)]
            self.objetivos = self.objetivos[:self.i_objetivo] + nuevos + self.objetivos[self.i_objetivo:]
            self.anclas = self.anclas + list(self.fase2b_nombres)
            self.obligatorios = self.fase2b_nombres[:1]   # como en empezar_fase2
            self.get_logger().warn(f'Sin estación cuadrada tras la cobertura: segunda cobertura {self.fase2b_nombres}')
            self.escribir_log(x, y, th, 'SEGUNDA_COBERTURA')
            if math.hypot(self.fase2b_x[0] - x, self.fase2b_y[0] - y) > self.plan_final_dist:
                self.planificar_final(x, y, th, 'COBERTURA2')
            return
        self.get_logger().warn('Pasillo recorrido sin encontrar la estación cuadrada: vuelvo a P Base')
        self.escribir_log(x, y, th, 'ESTACION_NO_ENCONTRADA')

    def planificar_final(self, x, y, th, nombre='FINAL'):
        # Planifica hasta el siguiente obligatorio (la pose final o el primer punto de la fase II). Eventos del log:
        # PLAN_<nombre> o <nombre>_SIN_CAMINO.
        inflado, segundos = self.replanificar(x, y)
        if inflado is None:
            ahora = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().warn(f'Sin camino ({nombre}): espero y reintento')
            self.escribir_log(x, y, th, f'{nombre}_SIN_CAMINO', segundos)
            self.estado = 'ESPERAR'
            self.t_espera = ahora
            self.t_sin_camino = ahora
            return
        self.get_logger().info(f'Plan ({nombre}) con inflado {inflado:.2f}')
        self.escribir_log(x, y, th, f'PLAN_{nombre} inflado={inflado:.2f}', segundos)

    def inicio(self, x, y, th, ahora):
        # Estado INICIO (robot parado): start_scans scans nuevos llenan la memoria de obstáculos y luego planifica desde
        # la pose inicial. Sin camino, pasa a ESPERAR, que reintenta.
        if self.scan is not self.salida_ultimo_scan:
            self.salida_ultimo_scan = self.scan
            self.salida_scans += 1
        if self.salida_scans < self.scans_salida:
            return
        inflado, segundos = self.replanificar(x, y)
        if inflado is None:
            self.get_logger().warn('Sin camino desde la salida: espero y reintento')
            self.escribir_log(x, y, th, 'SALIDA_SIN_CAMINO', segundos)
            self.estado = 'ESPERAR'
            self.t_espera = ahora
            self.t_sin_camino = ahora
            return
        self.get_logger().info(f'Plan desde la salida con inflado {inflado:.2f}: {[o[0] for o in self.objetivos]}')
        self.escribir_log(x, y, th, f'PLAN_SALIDA inflado={inflado:.2f} obstaculos={len(self.obstaculos)}', segundos)
        self.estado = 'GIRAR'

    def pose_inicial_lidar(self, x, y, th):
        # Estado POSE_INICIAL (robot parado, antes de moverse): con la mediana de start_pose_scans scans estima la pose
        # con las paredes del mapa (comprobar_pose.estimar) y revisar_pose_inicial decide qué se aplica. Si no vale, o
        # si el giróscopo marca giro con el robot quieto, no se mueve: POSE_INICIAL_RECHAZADA y repite con scans nuevos.
        msg = self.scan
        if msg is self.pi_ultimo_scan:
            return                  # el timer va más rápido que el LiDAR: cada scan se usa una vez
        self.pi_ultimo_scan = msg
        r = np.array(msg.ranges, dtype=float)
        r[~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)] = np.nan   # sin dato
        self.pi_scans.append(r)
        if len(self.pi_scans) < self.pi_n:
            return
        info = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        xe, ye, the, resumen = cp.estimar((x, y, th), self.lidar_offset_x, cp.mediana_de_scans(self.pi_scans), info,
                                          self.caras, self.pi_eje_rumbo)
        the -= self.lidar_yaw_offset     # las paredes dan el rumbo del LiDAR: el del robot es ese menos el desfase
        nx, ny, nth, motivo = revisar_pose_inicial((x, y, th), (xe, ye, the), resumen, self.pi_eje_rumbo,
                                                   self.pi_min_puntos, self.pi_max_rms, self.pi_max_corr,
                                                   self.pi_max_ang)
        giro = cp.aviso_giroscopo(self.pi_wz, self.pi_yaws)
        if not motivo and giro:
            motivo = 'giroscopo'
        ejes = ''.join(sorted({c['eje'] for c in resumen}))
        n = sum(c['n'] for c in resumen)
        rms = max([c['rms'] for c in resumen] or [0.0])
        medido = (f'estimada ({xe:.3f}, {ye:.3f}, {math.degrees(the):+.2f}°) paredes {ejes or "-"} n={n} '
                  f'rms={rms * 100:.1f}cm')
        ecos = cp.fraccion_con_ecos(self.pi_scans, self.alcance_min_obstaculos)
        self.pi_scans, self.pi_wz, self.pi_yaws = [], [], []
        if ecos > cp.ECOS_MAX:
            self.get_logger().warn(f'El {ecos * 100:.0f} % de los scans ve algo a menos de '
                                   f'{self.alcance_min_obstaculos:.2f} m: ¿ventana del LiDAR sucia o algo pegado?')
            self.escribir_log(x, y, th, f'AVISO_ECOS {ecos:.2f}')
        if motivo:
            texto = f'{motivo} {medido}' + (f' ({giro})' if motivo == 'giroscopo' else '')
            self.get_logger().warn(f'Pose de salida no válida: {texto}. No me muevo; repito con scans nuevos')
            self.escribir_log(x, y, th, f'POSE_INICIAL_RECHAZADA {texto}')
            return
        texto = (f'dx={nx - x:+.3f} dy={ny - y:+.3f} dth={math.degrees(normalizar(nth - th)):+.2f} {medido}'
                 + ('' if 'X' in ejes else ' (X sin paredes: la de initial_pose)')
                 + ('' if 'Y' in ejes else ' (Y sin paredes: la de initial_pose)'))
        self.get_logger().info(f'Pose de salida: ({nx:.3f}, {ny:.3f}, {math.degrees(nth):+.2f}°): {texto}')
        self.fijar_pose(nx, ny, nth)
        self.escribir_log(nx, ny, nth, f'POSE_INICIAL {texto}', math.hypot(nx - x, ny - y))
        self.obstaculos = {}
        self.estado = self.estado_tras_pose

    def replanificar(self, x, y, sustituir=False, largo=True):
        # Planifica desde la pose actual hasta el siguiente punto obligatorio (o la pose final), pasando por las anclas
        # que queden por el camino; después sigue la lista original.
        # Devuelve (inflado usado, segundos), o (None, segundos) si no hay camino ni con el inflado mínimo.
        # sustituir: si el obligatorio está tapado, va al punto libre más cercano a él (punto_sustituto).
        # largo: si es False, un camino mucho más largo que el que quedaba (rodeo_largo) no vale. Se prueba el inflado
        # siguiente y, si ninguno da uno corto, no hay camino. El rodeo rechazado queda en self.rodeo_rechazado.
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
            # Un obligatorio tapado no se puede saltar: no hay camino, salvo que ya toque sustituirlo por el punto libre
            # más cercano. Un ancla no obligatoria tapada se salta.
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
                if p == (x, y) and len(celdas) > 2 and pl.holgura_en(holgura, self.meta, x, y) <= inflado:
                    # El robot está dentro del inflado: primero se aparta a la celda libre más cercana (celdas[1]) y
                    # se simplifica desde ahí. Si no, la holgura exigida sería casi 0 y el atajo iría pegado a él.
                    puntos = [celdas[0]] + pl.simplificar(celdas[1:], holgura, self.meta, self.simp_paso,
                                                          self.simp_tope, self.simp_margen)
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

    #  Corrección continua con el mapa

    def corregir_con_mapa(self, x, y, th):
        # Cada map_corr_every_scans scans nuevos, si el robot casi no gira y aún no está en el tramo de la puerta (allí
        # corrige ajuste_puerta), compara el scan con el mapa. Dentro de corridor_corr_zone (el pasillo) usa las paredes
        # medidas con cinta (corregir_con_paredes).
        # Modo "activo": aplica la corrección, limitada por paso. Modo "sombra": solo la escribe en el log.
        # Devuelve la pose con la que seguir este ciclo.
        msg = self.scan
        if (self.corr_modo == 'apagado' and self.pasillo_modo == 'apagado') or msg is self.corr_ultimo_scan:
            return x, y, th
        self.corr_ultimo_scan = msg
        self.corr_cuenta += 1
        if (self.corr_cuenta % self.corr_cada != 0 or abs(self.w_anterior) > self.corr_max_w
                or self.estado not in ('GIRAR', 'AVANZAR', 'ESPERAR')):
            return x, y, th
        x0, x1, y0, y1 = self.pasillo_zona
        if x0 <= x <= x1 and y0 <= y <= y1:
            return self.corregir_con_paredes(x, y, th)
        if self.corr_modo == 'apagado' or y > self.corr_y_max:
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

    def corregir_con_paredes(self, x, y, th):
        # En el pasillo usa las paredes medidas con cinta (ajuste_paredes) en vez del mapa, que allí está mal en Y.
        # Solo cuentan los puntos a menos de corridor_corr_max_range: más lejos, el LiDAR da las paredes con ±5 cm.
        if self.pasillo_modo == 'apagado':
            return x, y, th
        msg = self.scan
        r = np.array(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = (np.isfinite(r) & (r >= max(msg.range_min, self.alcance_min_obstaculos))
              & (r <= min(msg.range_max, self.pasillo_alcance)))
        ok[1::2] = False                # con uno de cada dos rayos basta
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        px = lx + r[ok] * np.cos(th + self.lidar_yaw_offset + a[ok])
        py = ly + r[ok] * np.sin(th + self.lidar_yaw_offset + a[ok])
        dx, dy, nx, ny = ajuste_paredes(px, py, self.pasillo_paredes, self.corr_ventana, self.corr_sig_medida,
                                        self.corr_sig_xy, self.corr_min_puntos)
        if nx < self.corr_min_puntos and ny < self.corr_min_puntos:
            return x, y, th
        texto = f'dx={dx:+.3f} dy={dy:+.3f} nx={nx} ny={ny}'
        if self.pasillo_modo != 'activo':
            self.escribir_log(x, y, th, f'CORR_PASILLO_SOMBRA {texto}', math.hypot(dx, dy))
            return x, y, th
        x2, y2 = x + limitar(dx, self.corr_max_xy), y + limitar(dy, self.corr_max_xy)
        self.fijar_pose(x2, y2, th)
        self.escribir_log(x2, y2, th, f'CORR_PASILLO {texto}', math.hypot(dx, dy))
        return x2, y2, th

    #  Relocalización con las paredes

    def reloc_adelantable(self, x, y):
        # Dice si un bloqueo, yendo al último waypoint antes de reloc_before (Door) y a menos de reloc_on_block_dist
        # de él, se debe resolver relocalizando. Casi siempre es el marco de la puerta visto dentro del hueco porque
        # la Y llega mal. Se relocaliza ahí mismo, una vez, y se borran los obstáculos calculados con la Y mala.
        i = self.i_objetivo
        if self.reloc_adelantada or i + 1 >= len(self.objetivos) or self.objetivos[i + 1][0] != self.reloc_antes_de:
            return False
        return math.hypot(self.objetivos[i][1] - x, self.objetivos[i][2] - y) < self.reloc_bloqueo_dist

    def reloc_reintentable(self, x, y):
        # Dice si conviene relocalizar otra vez tras un SIN_CAMINO cerca de la puerta (Door por delante, a menos de
        # reloc_retry_dist) sin ninguna relocalización total. Si alguien tapaba el final de la canaleta y se ha
        # apartado, ahora puede salir total.
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
        # (ajuste_puerta): total (x, y y rumbo), parcial (x y rumbo) o nada (se sigue con la odometría).
        msg = self.scan
        if msg is self.reloc_ultimo_scan:
            return                  # el timer va más rápido que el LiDAR: cada scan se usa una vez
        self.reloc_ultimo_scan = msg
        r = np.array(msg.ranges, dtype=float)
        r[~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)] = np.nan   # sin dato
        self.reloc_scans_lista.append(r)
        if len(self.reloc_scans_lista) < self.reloc_n:
            return
        # Mediana de los scans, cada lectura al mundo con la pose creída y ajuste a la pared de la puerta
        r = cp.mediana_de_scans(self.reloc_scans_lista)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        if self.reloc_tipo == 'Q':
            self.relocalizar_q(r, a, x, y, th)
            return
        if self.reloc_tipo == 'P':
            self.relocalizar_q(r, a, x, y, th, 'RELOC_P', self.ajustar_esquina_p)
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
            texto = (f'dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}{aviso}'
                     + (f' Y={motivo}' if motivo else ''))
            self.get_logger().info(f'Relocalización: {texto} ({medido})')
            self.escribir_log(nx, ny, nth, f'RELOC {texto}', math.hypot(dx, dy))
        else:
            nx, ny, nth = x + dx, y, normalizar(th + dth)
            texto = f'{motivo} dx={dx:+.3f} dth={math.degrees(dth):+.2f} (dy={dy:+.3f} no aplicado){aviso}'
            self.get_logger().warn(f'Relocalización parcial: {texto} ({medido})')
            self.escribir_log(nx, ny, nth, f'RELOC_PARCIAL {texto}', abs(dx))
        self.fijar_pose(nx, ny, nth)
        self.obstaculos = {}        # se calcularon con la pose vieja: se vuelven a detectar
        # Comprobación con el robot parado: el mismo ajuste desde la pose corregida debe dar casi 0
        dx2, dy2, dth2, nivel2, motivo2, _ = self.ajustar_puerta(r, a, nx, ny, nth)
        mal = (nivel2 == '' or abs(dx2) > self.reloc_comprobar_dist or abs(dth2) > self.reloc_comprobar_ang
               or (nivel == 'total' and (nivel2 != 'total' or abs(dy2) > self.reloc_comprobar_dist)))
        if mal:
            texto = f'{nivel2 or "rechazada"} {motivo2} dx={dx2:+.3f} dy={dy2:+.3f} dth={math.degrees(dth2):+.2f}'
            self.get_logger().warn(f'Relocalización inconsistente al repetir: {texto}')
            self.escribir_log(nx, ny, nth, f'RELOC_INCONSISTENTE {texto}', math.hypot(dx2, dy2))
        self.estado = 'GIRAR'

    def relocalizar_q(self, r, a, x, y, th, etiqueta='RELOC_Q', ajustar=None):
        # Relocalización con dos paredes (ajuste_esquina): x, y y rumbo, o nada. Después el robot vuelve al punto.
        # Por defecto, la esquina de Q; con etiqueta y ajustar, las paredes de P Base.
        ajustar = ajustar or self.ajustar_esquina

        def texto_de(dx, dy, dth, d):
            return (f"dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f} nx={d['nx']} ny={d['ny']}"
                    + (f" dth_x={math.degrees(d['dth_x']):+.2f} dth_y={math.degrees(d['dth_y']):+.2f}"
                       if d['dth_x'] is not None else ''))
        dx, dy, dth, motivo, d = ajustar(r, a, x, y, th)
        self.estado = 'GIRAR'
        if motivo:
            self.get_logger().warn(f'{etiqueta} rechazada: {motivo} {texto_de(dx, dy, dth, d)}')
            self.escribir_log(x, y, th, f'{etiqueta}_RECHAZADA {motivo} {texto_de(dx, dy, dth, d)}', math.hypot(dx, dy))
            return
        nx, ny, nth = x + dx, y + dy, normalizar(th + dth)
        self.get_logger().info(f'{etiqueta}: {texto_de(dx, dy, dth, d)}')
        self.escribir_log(nx, ny, nth, f'{etiqueta} {texto_de(dx, dy, dth, d)}', math.hypot(dx, dy))
        self.fijar_pose(nx, ny, nth)
        self.obstaculos = {}
        dx2, dy2, dth2, motivo2, d2 = ajustar(r, a, nx, ny, nth)
        if (motivo2 or abs(dx2) > self.reloc_comprobar_dist or abs(dy2) > self.reloc_comprobar_dist
                or abs(dth2) > self.reloc_comprobar_ang):
            texto = f'{motivo2 or "ok"} {texto_de(dx2, dy2, dth2, d2)}'
            self.get_logger().warn(f'{etiqueta} inconsistente al repetir: {texto}')
            self.escribir_log(nx, ny, nth, f'{etiqueta}_INCONSISTENTE {texto}', math.hypot(dx2, dy2))

    def ajustar_esquina_p(self, r, a, x, y, th):
        # Como ajustar_esquina, con las paredes de P Base: el tramo perpendicular a X del lado de la sala y el extintor
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        ok = ~np.isnan(r)
        px = lx + r[ok] * np.cos(th + self.lidar_yaw_offset + a[ok])
        py = ly + r[ok] * np.sin(th + self.lidar_yaw_offset + a[ok])
        return ajuste_esquina(px, py, x, y, self.reloc_p_pared_x, self.reloc_p_pared_y, self.reloc_p_banda,
                              self.reloc_p_alcance, self.reloc_p_max_rms, self.reloc_p_min_puntos,
                              self.reloc_p_max_corr, self.reloc_p_max_ang, self.reloc_p_max_dif, 'Y')

    def ajustar_esquina(self, r, a, x, y, th):
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        ok = ~np.isnan(r)
        px = lx + r[ok] * np.cos(th + self.lidar_yaw_offset + a[ok])
        py = ly + r[ok] * np.sin(th + self.lidar_yaw_offset + a[ok])
        return ajuste_esquina(px, py, x, y, self.reloc_q_pared_x, self.reloc_q_pared_y, self.reloc_q_banda,
                              self.reloc_q_alcance, self.reloc_q_max_rms, self.reloc_q_min_puntos,
                              self.reloc_q_max_corr, self.reloc_q_max_ang, self.reloc_q_max_dif,
                              self.reloc_q_eje_rumbo)

    def ajustar_puerta(self, r, a, x, y, th):
        # Pasa las lecturas (r, ángulos a) al mundo con la pose (x, y, th) y llama a ajuste_puerta con los parámetros
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
        # Hace que pose_mundo() dé (x, y, th) sin tocar lo integrado desde la salida (x_rel, y_rel, th_rel), moviendo
        # el origen de la odometría en el mundo (pose_inicial).
        thi = normalizar(th - self.th_rel)
        xi = x - (math.cos(thi) * self.x_rel - math.sin(thi) * self.y_rel)
        yi = y - (math.sin(thi) * self.x_rel + math.cos(thi) * self.y_rel)
        self.pose_inicial = [xi, yi, thi]

    #  Control

    def control_loop(self):
        # Sin datos frescos de algún sensor: parada total
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

        # Obstáculos nuevos: si bloquean la ruta, para y pasa a ESPERAR
        ahora_s = self.get_clock().now().nanoseconds * 1e-9
        if self.estado != 'POSE_INICIAL':       # con la pose de salida sin corregir, no se guarda nada
            self.actualizar_obstaculos(x, y, th, ahora_s)
            self.detectar_pilares(x, y, th, ahora_s)
        if self.estado in ('GIRAR', 'AVANZAR'):
            bloqueo = self.ruta_bloqueada(x, y)
            if bloqueo is not None and self.reloc_adelantable(x, y):
                self.reloc_adelantada = True
                self.get_logger().warn(
                    f'Bloqueo en ({bloqueo[0]:.2f}, {bloqueo[1]:.2f}) junto a la puerta: relocalizo aquí')
                self.escribir_log(x, y, th, 'RELOC_ADELANTADA', None, bloqueo[0], bloqueo[1])
                self.empezar_reloc('puerta')
            elif bloqueo is not None:
                self.get_logger().warn(
                    f'Ruta bloqueada por un obstáculo en ({bloqueo[0]:.2f}, {bloqueo[1]:.2f}): espero')
                self.estado = 'ESPERAR'
                self.t_espera = ahora_s
                self.escribir_log(x, y, th, 'BLOQUEO', None, bloqueo[0], bloqueo[1])

        # Lo que quiere hacer la máquina de estados
        if self.estado == 'ESPERAR':
            v, w = 0.0, 0.0
            self.esperar(x, y, th, ahora_s)
        elif self.estado == 'RELOC':
            v, w = 0.0, 0.0
            self.relocalizar(x, y, th)
        elif self.estado == 'INICIO':
            v, w = 0.0, 0.0
            self.inicio(x, y, th, ahora_s)
        elif self.estado == 'POSE_INICIAL':
            v, w = 0.0, 0.0
            self.pose_inicial_lidar(x, y, th)
        elif self.estado == 'MEDIR3':
            v, w = 0.0, 0.0
            self.medir3(x, y, th)
        elif self.estado == 'ENTRAR':
            v, w = self.entrar(x, y, th)
        elif self.estado == 'VERIFICAR3':
            v, w = 0.0, 0.0
            self.verificar3(x, y, th)
        else:
            v, w = self.maquina_estados(x, y, th)

        # Parada de emergencia, por encima de cualquier estado. Solo anula la velocidad lineal: si también bloqueara el
        # giro, mirando a una pared cercana el robot nunca podría girar para salir.
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

        # Fila periódica del log
        ahora = self.get_clock().now()
        if self.t_ultimo_log is None or (ahora - self.t_ultimo_log).nanoseconds * 1e-9 >= self.log_period:
            self.t_ultimo_log = ahora
            self.escribir_log(x, y, th, '')

    def maquina_estados(self, x, y, th):
        # Devuelve (v, w) según el estado y cambia de estado y de objetivo cuando toca
        if self.estado == 'FIN':
            return 0.0, 0.0

        if self.estado == 'PAUSA':
            # Entre una fase y la siguiente, parado
            if self.get_clock().now().nanoseconds * 1e-9 - self.t_pausa >= self.fase2_espera:
                if self.fase == 'I':
                    self.empezar_fase2(x, y, th)
                else:
                    self.empezar_fase3(x, y, th)
            return 0.0, 0.0

        if self.estado == 'ORIENTAR_P':
            # En P Base: gira a mirar a +X y se relocaliza parado, igual que cuando se calibraron las paredes
            error = normalizar(0.0 - th)
            if abs(error) < self.tol_final_yaw:
                self.get_logger().info(f'Relocalizo en P Base: junto {self.reloc_n} scans parado')
                self.empezar_reloc('P')
                return 0.0, 0.0
            return 0.0, limitar(self.k_angular * error, self.max_w)

        if self.estado == 'ORIENTAR':
            error = normalizar(self.pose_final[2] - th)
            if abs(error) < self.tol_final_yaw and self.fase == 'III':
                # Fase III: delante de la estación, mirando a su centro. Ahora mide sus patas parado (MEDIR3)
                self.get_logger().info('Delante de la estación: mido sus patas')
                self.escribir_log(x, y, th, 'FASE3_DELANTE', error)
                self.estado = 'MEDIR3'
                self.f3_scans = []
                return 0.0, 0.0
            if abs(error) < self.tol_final_yaw:
                self.get_logger().info(f'FASE {self.fase} COMPLETADA')
                self.escribir_log(x, y, th, f'FASE {self.fase} COMPLETADA', error)
                self.estado = 'FIN'
                if (self.fase == 'I' and self.fase2_activa) or (self.fase == 'II' and self.fase3_activa):
                    self.estado = 'PAUSA'
                    self.t_pausa = self.get_clock().now().nanoseconds * 1e-9
                return 0.0, 0.0
            return 0.0, limitar(self.k_angular * error, self.max_w)

        # La llegada al objetivo se comprueba antes que nada, esté girando o avanzando
        nombre, ox, oy, tol = self.objetivos[self.i_objetivo]
        dist = math.hypot(ox - x, oy - y)
        # Cuenta también el sustituto de Q (Q~). Con reloc_q_at "" no se relocaliza nunca
        es_q = bool(self.reloc_q_en) and (nombre == self.reloc_q_en
                                          or self.sustitutos.get(nombre, ('',))[0] == self.reloc_q_en)
        if dist < tol and es_q and self.reloc_q_hechas < self.reloc_q_veces:
            # En Q se relocaliza con la esquina antes de darlo por alcanzado y luego se vuelve a Q con la pose nueva
            self.reloc_q_hechas += 1
            self.get_logger().info(f'Relocalizo en {nombre} ({self.reloc_q_hechas}/{self.reloc_q_veces}): '
                                   f'junto {self.reloc_n} scans parado')
            self.escribir_log(x, y, th, f'LLEGADA_RELOC_Q {self.reloc_q_hechas}', dist)
            self.empezar_reloc('Q')
            return 0.0, 0.0
        if dist < tol and self.fase == 'II' and nombre == 'final' and self.reloc_p_hechas < self.reloc_p_veces:
            # En P Base, lo mismo que en Q: se relocaliza antes de darlo por alcanzado y luego vuelve con la pose nueva
            self.reloc_p_hechas += 1
            self.escribir_log(x, y, th, f'LLEGADA_RELOC_P {self.reloc_p_hechas}', dist)
            self.estado = 'ORIENTAR_P'
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
            self.fin_de_cobertura(x, y, th)
            self.estado = 'ORIENTAR' if self.i_objetivo == len(self.objetivos) else 'GIRAR'
            est = self.estacion_cuadrada() if self.estado == 'ORIENTAR' and self.fase == 'II' else None
            if est is not None:
                # En P Base mira a la estación encontrada, porque el enunciado no da el ángulo de P Base
                th_est = math.atan2(est['y'] - y, est['x'] - x)
                self.pose_final = [self.pose_final[0], self.pose_final[1], th_est]
                self.escribir_log(x, y, th, f'THETA_HACIA_ESTACION {math.degrees(th_est):+.1f}', None,
                                  est['x'], est['y'])
            # La pose final asignada puede no ser Q: si queda lejos del último obligatorio, se planifica hasta ella.
            # Sin camino pasa a ESPERAR, que reintenta.
            sig = self.objetivos[self.i_objetivo] if self.i_objetivo < len(self.objetivos) else None
            if sig is not None and sig[0] == 'final' and math.hypot(sig[1] - x, sig[2] - y) > self.plan_final_dist:
                self.planificar_final(x, y, th)
            # Último waypoint antes de reloc_before (Door), se llame como se llame (también uno de un replan)
            if self.i_objetivo < len(self.objetivos) and self.objetivos[self.i_objetivo][0] == self.reloc_antes_de:
                self.empezar_reloc('puerta')
                self.get_logger().info(f'Relocalizo en {nombre}: junto {self.reloc_n} scans parado')
            if self.v_paso > 0.0 and self.fase == 'I' and self.estado == 'GIRAR':
                # Fase I: no para en el punto, sigue ya hacia el siguiente. Si hay que girar más de 30°, AVANZAR pasa a GIRAR
                self.estado = 'AVANZAR'
                return self.maquina_estados(x, y, th)
            return 0.0, 0.0

        # Error de ángulo hacia el objetivo
        error = normalizar(math.atan2(oy - y, ox - x) - th)
        w = limitar(self.k_angular * error, self.max_w)

        # GIRAR: sobre sí mismo hasta quedar casi alineado, porque el robot no puede ir de lado
        if self.estado == 'GIRAR':
            if abs(error) >= self.angulo_girar:
                return 0.0, w
            self.estado = 'AVANZAR'

        # AVANZAR: v proporcional a la distancia y w corrige el rumbo. El umbral de salida (30°) es mayor que el de
        # entrada (10°) para no alternar de estado en cada ciclo.
        if abs(error) > self.angulo_reorientar:
            self.estado = 'GIRAR'
            return 0.0, w
        v = min(self.k_lineal * dist, self.max_v)
        if (self.v_paso > 0.0 and self.fase == 'I' and nombre != 'final' and not es_q
                and abs(error) < self.angulo_girar):
            v = max(v, self.v_paso)     # bien alineado: no se arrastra al acercarse (la tolerancia no cambia)
        return v, w

    #  Salidas

    def publicar_vel(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # reloj del nodo: vale con use_sim_time
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.publisher.publish(msg)

    def yaws_log(self):
        # Rumbo según /odom y según el giróscopo, ambos en el sistema del mundo, para compararlos
        thi = self.pose_inicial[2]
        return round(normalizar(thi + self.yaw_odom), 3), round(normalizar(thi + self.yaw_imu), 3)

    def escribir_log(self, x, y, th, evento, error=None, ox=None, oy=None):
        def r3(v):
            return '' if v is None else round(v, 3)
        t = self.get_clock().now().nanoseconds * 1e-9  # en Gazebo, tiempo simulado
        self.csv.writerow([round(t, 3), self.fase, r3(x), r3(y), r3(th), self.estado, evento,
                           r3(error), r3(ox), r3(oy), *self.yaws_log()])
        self.fichero.flush()  # que no se pierda nada con un Ctrl+C


def main(args=None):
    # Sin el manejador de Ctrl+C de rclpy, el contexto de ROS sigue vivo en el finally y la velocidad cero llega
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
