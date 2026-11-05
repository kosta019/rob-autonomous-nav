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

from autonomous_nav import planificador as pl


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
        self.giro_max_obstaculos = self.declare_parameter('obstacle_max_turn_rate', D).value
        self.alcance_bloqueo = self.declare_parameter('block_lookahead', D).value
        self.histeresis = self.declare_parameter('block_hysteresis', D).value
        self.espera = self.declare_parameter('obstacle_wait_time', D).value
        self.reintento = self.declare_parameter('replan_retry_period', D).value
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
        validas = np.isfinite(r) & (r >= msg.range_min) & (r <= min(msg.range_max, self.alcance_obstaculos))
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
        if not self.obstaculos:
            return None
        puntos = [(x, y)] + [(o[1], o[2]) for o in self.objetivos[self.i_objetivo:]]
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
        k = int(np.argmin(d))
        return (o[k, 0], o[k, 1]) if d[k] < self.inflado + extra else None

    def esperar(self, x, y, th, ahora):
        # Estado ESPERAR (robot parado). Si la ruta se libera, se sigue con el plan que había.
        # Si a los obstacle_wait_time s sigue bloqueada, se replanifica (con el robot parado).
        if self.ruta_bloqueada(x, y, self.histeresis) is None:
            self.get_logger().info('Ruta libre: sigo con el plan')
            self.estado = 'GIRAR'
            self.escribir_log(x, y, th, 'RUTA_LIBRE')
            return
        if ahora - self.t_espera < self.espera:
            return
        inflado, segundos = self.replanificar(x, y)
        if inflado is None:
            self.get_logger().warn(f'Sin camino ({segundos:.2f} s): reintento en {self.reintento:.0f} s')
            self.escribir_log(x, y, th, 'SIN_CAMINO', segundos)
            self.t_espera = ahora - self.espera + self.reintento   # el próximo intento, dentro de replan_retry_period
            return
        self.get_logger().info(f'Replanificado con inflado {inflado:.2f} m en {segundos:.2f} s: '
                               f'{[o[0] for o in self.objetivos[self.i_objetivo:]]}')
        self.escribir_log(x, y, th, f'REPLANIFICACION inflado={inflado:.2f}', segundos)
        self.estado = 'GIRAR'

    def replanificar(self, x, y):
        # Desde la pose actual hasta el siguiente punto obligatorio (o la pose final), pasando por las
        # anclas que queden por el camino; después sigue la lista original.
        # Devuelve (inflado usado, segundos), o (None, segundos) si no hay camino ni con el inflado mínimo.
        t0 = time.perf_counter()
        resto = self.objetivos[self.i_objetivo:]
        k = next(k for k, o in enumerate(resto) if o[0] in self.obligatorios or o[0] == 'final')
        anclas = [o for o in resto[:k] if o[0] in self.anclas] + [resto[k]]
        despues = resto[k + 1:]
        obst = [(v[0], v[1]) for v in self.obstaculos.values()]
        holgura = pl.holgura_con_obstaculos(self.holgura, self.meta, obst, self.margen_plan, self.holgura_objetivo)
        self.n_replan += 1

        for inflado in self.inflados:
            # Un punto obligatorio tapado por un obstáculo no se puede saltar: no hay camino (se espera).
            # Un ancla no obligatoria tapada se salta.
            if pl.holgura_en(holgura, self.meta, anclas[-1][1], anclas[-1][2]) <= inflado:
                continue
            lista = [o for o in anclas[:-1] if pl.holgura_en(holgura, self.meta, o[1], o[2]) > inflado] + [anclas[-1]]
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
            if nuevos is not None:
                self.objetivos = self.objetivos[:self.i_objetivo] + nuevos + despues
                return inflado, time.perf_counter() - t0
        return None, time.perf_counter() - t0

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

        # 2. Obstáculos nuevos (paso 3): si bloquean la ruta, parar y pasar a ESPERAR
        ahora_s = self.get_clock().now().nanoseconds * 1e-9
        self.actualizar_obstaculos(x, y, th, ahora_s)
        if self.estado in ('GIRAR', 'AVANZAR'):
            bloqueo = self.ruta_bloqueada(x, y)
            if bloqueo is not None:
                self.get_logger().warn(f'Ruta bloqueada por un obstáculo en ({bloqueo[0]:.2f}, {bloqueo[1]:.2f}): espero')
                self.estado = 'ESPERAR'
                self.t_espera = ahora_s
                self.escribir_log(x, y, th, 'BLOQUEO', None, bloqueo[0], bloqueo[1])

        # 3. Lo que querría hacer la máquina de estados
        if self.estado == 'ESPERAR':
            v, w = 0.0, 0.0
            self.esperar(x, y, th, ahora_s)
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

        self.publicar_vel(v, w)
        self.w_anterior = w

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
        if dist < tol:
            self.get_logger().info(f'Waypoint {nombre} alcanzado (error {dist:.3f} m)')
            self.escribir_log(x, y, th, f'WAYPOINT {nombre}', dist)
            self.i_objetivo += 1
            self.estado = 'ORIENTAR' if self.i_objetivo == len(self.objetivos) else 'GIRAR'
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
