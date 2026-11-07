# Figuras nuevas de la memoria técnica: arquitectura (nodos y topics), máquina de estados, ruta de cobertura de la
# fase II y mapa entregable del SLAM. Script OFFLINE (matplotlib, Pillow): no lo usa ningún nodo.
# Uso (desde la raíz del repo):  python3 docs/figuras_memoria.py
# El mapa del SLAM sale del bag de bags/slam/entregable_131350 (el ensayo del 06/11 16:31 pasado por slam.launch.py);
# si no está, esa figura se salta.
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(RAIZ, 'docs')
sys.path.insert(0, os.path.join(RAIZ, 'autonomous_nav', 'scripts'))

AZUL = '#1f5fa8'
NARANJA = '#d9822b'
GRIS = '#555555'


# ---------- Utilidades de dibujo ----------

def caja(eje, x, y, w, h, texto, color, tam=10, negrita=True):
    eje.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle='round,pad=0.02,rounding_size=0.08',
                                 facecolor=color, edgecolor='black', linewidth=1.0, alpha=0.95))
    eje.text(x, y, texto, ha='center', va='center', fontsize=tam, fontweight='bold' if negrita else 'normal')


def flecha(eje, p, q, texto='', color='black', curva=0.0, tam=8.5, pos=0.5, desp=(0, 0)):
    eje.add_patch(FancyArrowPatch(p, q, arrowstyle='-|>', mutation_scale=13, color=color, linewidth=1.2,
                                  connectionstyle=f'arc3,rad={curva}', shrinkA=2, shrinkB=2))
    if texto:
        mx = p[0] + pos * (q[0] - p[0]) + desp[0]
        my = p[1] + pos * (q[1] - p[1]) + desp[1]
        eje.text(mx, my, texto, ha='center', va='center', fontsize=tam, color=color,
                 bbox=dict(facecolor='white', edgecolor='none', pad=1.0, alpha=0.9))


# ---------- Figura: arquitectura ----------

def arquitectura():
    fig, eje = plt.subplots(figsize=(11, 6.2))
    eje.set_xlim(0, 11)
    eje.set_ylim(0, 6.2)
    eje.axis('off')

    caja(eje, 1.6, 4.3, 2.6, 1.6, 'TurtleBot3 Burger\n(bringup: OpenCR,\nLDS, Raspberry Pi)', '#e8e8e8')
    caja(eje, 6.3, 4.3, 2.6, 1.6, 'fase1_node\n(fases I, II y III)', '#cfe0f5', tam=11)
    caja(eje, 4.4, 1.5, 2.0, 0.8, 'scan_slam', '#fbe3c8')
    caja(eje, 7.0, 1.5, 2.0, 0.8, 'slam_toolbox', '#fbe3c8')
    caja(eje, 9.7, 1.5, 2.0, 0.8, 'map_saver_cli', '#fbe3c8')

    # Ficheros (sin caja redondeada: no son nodos)
    for x, y, t in [(9.6, 5.3, 'config/*.yaml\n(puntos, zonas,\nparámetros)'),
                    (9.6, 4.3, 'mapas de holgura\ny de distancias'),
                    (9.6, 3.3, 'Mission Log\n(CSV)'),
                    (9.7, 0.35, 'mapa .pgm + .yaml\n(entregable)')]:
        eje.add_patch(Rectangle((x - 0.95, y - 0.38), 1.9, 0.76, facecolor='white', edgecolor=GRIS, linestyle='--'))
        eje.text(x, y, t, ha='center', va='center', fontsize=8.5, color=GRIS)

    # Topics del robot al nodo y vuelta
    flecha(eje, (2.9, 4.85), (5.0, 4.85), '/scan  (LaserScan, 5 Hz)', AZUL)
    flecha(eje, (2.9, 4.4), (5.0, 4.4), '/odom  (Odometry, 20 Hz)', AZUL)
    flecha(eje, (2.9, 3.95), (5.0, 3.95), '/imu  (Imu, 20 Hz)', AZUL)
    flecha(eje, (5.0, 3.6), (2.9, 3.6), '/cmd_vel  (TwistStamped, 10 Hz)', 'darkred')
    flecha(eje, (8.65, 5.3), (7.6, 4.75), '', GRIS)
    flecha(eje, (8.65, 4.3), (7.6, 4.3), '', GRIS)
    flecha(eje, (7.6, 3.85), (8.65, 3.3), '', GRIS)

    # SLAM (solo para el mapa entregable: el nodo no lo usa para navegar)
    flecha(eje, (1.6, 3.5), (3.4, 1.5), '/scan', AZUL, curva=0.25, pos=0.35, desp=(-0.25, 0))
    flecha(eje, (5.4, 1.5), (6.0, 1.5), '', NARANJA)
    eje.text(5.7, 1.75, '/scan_slam\n(sello −0.1 s)', ha='center', va='bottom', fontsize=8, color=NARANJA)
    flecha(eje, (1.6, 3.5), (6.6, 1.95), '/tf  (odom → base)', AZUL, curva=-0.12, pos=0.62, desp=(0, 0.1))
    flecha(eje, (8.0, 1.5), (8.7, 1.5), '', NARANJA)
    eje.text(8.35, 1.75, '/map', ha='center', va='bottom', fontsize=8, color=NARANJA)
    eje.text(7.0, 0.95, 'publica /tf map → odom', ha='center', va='top', fontsize=8, color=NARANJA)
    flecha(eje, (9.7, 1.1), (9.7, 0.75), '', NARANJA)

    eje.text(0.1, 6.0, 'Navegación (lazo de control)', fontsize=10, color=AZUL, fontweight='bold')
    eje.text(0.1, 2.45, 'Mapa entregable (fases I y II)', fontsize=10, color=NARANJA, fontweight='bold')
    eje.plot([0.1, 10.9], [2.7, 2.7], color='0.75', linewidth=0.8, linestyle=':')
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, 'fig_arquitectura.png'), dpi=150)
    plt.close(fig)


# ---------- Figura: máquina de estados ----------

def estados():
    fig, eje = plt.subplots(figsize=(11, 6.9))
    eje.set_xlim(0, 11.1)
    eje.set_ylim(0, 6.9)
    eje.axis('off')
    w, h = 1.55, 0.6
    P = {'POSE_INICIAL': (1.0, 5.8), 'INICIO': (3.2, 5.8), 'GIRAR': (3.2, 3.9), 'AVANZAR': (6.1, 3.9),
         'ESPERAR': (5.6, 5.9), 'RELOC': (4.65, 2.2), 'ORIENTAR': (8.6, 3.9), 'PAUSA': (8.6, 5.6),
         'FIN': (10.25, 3.9), 'MEDIR3': (8.6, 1.9), 'ENTRAR': (8.6, 0.7), 'VERIFICAR3': (10.25, 0.7)}
    colores = {'MEDIR3': '#fbe3c8', 'ENTRAR': '#fbe3c8', 'VERIFICAR3': '#fbe3c8', 'ESPERAR': '#f6d0d0',
               'RELOC': '#d8ecd2', 'POSE_INICIAL': '#d8ecd2'}
    for n, (x, y) in P.items():
        caja(eje, x, y, w if n != 'POSE_INICIAL' else 1.75, h, n, colores.get(n, '#cfe0f5'), tam=9)

    def f(a, b, t='', **k):
        (xa, ya), (xb, yb) = P[a], P[b]
        flecha(eje, (xa, ya), (xb, yb), t, tam=7.5, **k)

    # Las flechas se recortan a mano en el borde de las cajas (FancyArrowPatch no conoce las cajas)
    flecha(eje, (1.9, 5.8), (2.4, 5.8), '', tam=7.5)
    eje.text(1.0, 6.25, 'pose de salida con las paredes', ha='center', fontsize=7.5)
    eje.text(3.0, 6.3, 'salida desde B, C o D:\nplanifica con lo que ve', ha='center', fontsize=7.5)
    flecha(eje, (1.0, 5.5), (2.4, 4.0), 'salida desde A', tam=7.5, curva=0.25, pos=0.4, desp=(-0.35, -0.1))
    flecha(eje, (3.2, 5.5), (3.2, 4.2), '', tam=7.5)
    flecha(eje, (4.0, 4.0), (5.3, 4.0), '|error| < 10°', tam=7.5, desp=(0, 0.2))
    flecha(eje, (5.3, 3.75), (4.0, 3.75), '|error| > 30°', tam=7.5, desp=(0, -0.2))
    flecha(eje, (3.75, 4.2), (5.0, 5.6), 'bloqueo', tam=7.5, desp=(-0.35, 0.1))
    flecha(eje, (6.1, 4.2), (5.9, 5.6), 'bloqueo', tam=7.5, pos=0.3, desp=(0.4, 0))
    flecha(eje, (4.8, 5.75), (3.45, 4.2), '', tam=7.5, curva=0.25)
    eje.text(5.6, 6.35, 'ruta libre, o replan\n(tras 2 s; reintento cada 5 s)', ha='center', fontsize=7.5)
    flecha(eje, (3.4, 3.6), (4.2, 2.5), 'en la puerta\no en Q', tam=7.5, desp=(-0.45, 0))
    flecha(eje, (5.1, 2.5), (5.85, 3.6), 'pose\ncorregida', tam=7.5, desp=(0.45, 0))
    flecha(eje, (6.9, 3.9), (7.8, 3.9), 'último\nwaypoint', tam=7.5, desp=(0, 0.35))
    flecha(eje, (9.4, 3.9), (9.45, 3.9), '', tam=7.5)
    flecha(eje, (9.4, 3.9), (9.5, 3.9), '', tam=7.5)
    eje.text(9.42, 4.25, 'fin', ha='center', fontsize=7.5)
    flecha(eje, (8.6, 4.2), (8.6, 5.3), 'fase I o II\ncompletada', tam=7.5, desp=(0.6, 0))
    flecha(eje, (7.8, 5.5), (4.0, 4.15), '', tam=7.5)
    eje.text(8.6, 6.35, 'tras 3 s: empieza la fase II (cobertura)\no la III (delante de la estación)', ha='center', fontsize=7.5,
             bbox=dict(facecolor='white', edgecolor='none', pad=1.0))
    flecha(eje, (8.6, 3.6), (8.6, 2.2), 'fase III: delante\nde la estación', tam=7.5, desp=(0.75, 0))
    flecha(eje, (8.6, 1.6), (8.6, 1.0), '', tam=7.5)
    eje.text(9.0, 1.3, 'centro relativo a las patas', fontsize=7.5, va='center')
    flecha(eje, (9.4, 0.7), (9.45, 0.7), '', tam=7.5)
    flecha(eje, (10.25, 1.0), (10.25, 3.6), 'aparcado\nmedido', tam=7.5, desp=(0.45, 0))
    flecha(eje, (7.8, 1.9), (5.4, 3.65), 'realinear', tam=7.5, curva=-0.2, pos=0.45, desp=(0, -0.1))

    eje.add_patch(Rectangle((0.15, 0.2), 6.0, 1.15, facecolor='#fff5d6', edgecolor='darkred'))
    eje.text(0.3, 0.78, 'Por encima de todos los estados, en cada ciclo (10 Hz):\n'
                        '• parada de emergencia: algo a < 0.30 m en el cono frontal de ±30° → v = 0\n'
                        '• sin /scan, /odom o /imu en 0.5 s → parada total (TIMEOUT_SENSOR)', fontsize=8, va='center')
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, 'fig_estados.png'), dpi=150)
    plt.close(fig)


# ---------- Figura: cobertura de la fase II ----------

def mapa_oficial():
    from PIL import Image
    img = np.array(Image.open(os.path.join(RAIZ, 'autonomous_nav', 'maps', 'mapa_1cm.png')).convert('L'))
    return img < 128          # fila = 100·X + 71, col = 100·Y + 2 (CLAUDE.md)


def cobertura():
    cfg = yaml.safe_load(open(os.path.join(RAIZ, 'autonomous_nav', 'config', 'fase1.yaml')))['fase1_node']
    p = cfg['ros__parameters']
    ocupado = mapa_oficial()
    fig, eje = plt.subplots(figsize=(10.5, 4.6))
    # El pasillo es largo en X: aquí X va en horizontal e Y en vertical (hacia arriba), como en la figura del pasillo
    ext = [-71 / 100, (ocupado.shape[0] - 71) / 100, -2 / 100, (ocupado.shape[1] - 2) / 100]
    eje.imshow(np.where(ocupado, 0.35, 1.0).T, cmap='gray', vmin=0, vmax=1, extent=ext, origin='lower')
    eje.set_xlim(-0.9, 10.0)
    eje.set_ylim(12.8, 17.5)

    # Zona a <= 1.2 m de las pasadas (donde una pata da 2-3 rayos)
    for y in p['phase2_waypoints_y'][1:3]:
        eje.add_patch(Rectangle((0.10, y - 1.2), 8.70, 2.4, facecolor='tab:green', alpha=0.08, edgecolor='none'))

    q = (8.97, 14.497)
    pb = p['phase2_final_pose']
    x1 = [q[0]] + list(p['phase2_waypoints_x']) + [pb[0]]
    y1 = [q[1]] + list(p['phase2_waypoints_y']) + [pb[1]]
    eje.plot(x1, y1, color=AZUL, linewidth=2.0, marker='o', markersize=4, label='cobertura (pasadas en Y 14.35 y 15.55)')
    for n, x, y in zip(p['phase2_waypoint_names'], p['phase2_waypoints_x'], p['phase2_waypoints_y']):
        eje.text(x, y - 0.32, n, fontsize=8, color=AZUL, ha='center')
    x2 = list(p['phase2_second_waypoints_x']) + [pb[0]]
    y2 = list(p['phase2_second_waypoints_y']) + [pb[1]]
    eje.plot(x2, y2, color=NARANJA, linewidth=1.4, linestyle='--', label='segunda cobertura (si no ha confirmado la estación)')

    # Paredes y suelo irregular medidos con cinta (06/11)
    eje.plot([-0.55, 9.57], [17.15, 17.15], color='darkred', linewidth=1.6, label='pared del extintor medida (Y 17.15)')
    eje.add_patch(Rectangle((-0.55, 16.52), 4.30 + 0.55, 17.15 - 16.52, facecolor='none', edgecolor='darkred',
                            hatch='//', linewidth=0.8, label='suelo irregular (cinta)'))

    # Estaciones: R medida con cinta (paralela a las paredes); T, su punto del enunciado (orientación sin medir)
    rx, ry, m = 7.17, 16.51, 0.22
    eje.add_patch(Rectangle((rx - m, ry - m), 2 * m, 2 * m, facecolor='none', edgecolor='saddlebrown'))
    for dx in (-m, m):
        for dy in (-m, m):
            eje.plot(rx + dx, ry + dy, 'o', color='saddlebrown', markersize=4)
    eje.text(rx - 0.6, ry, 'R', fontsize=10, color='saddlebrown', ha='center', va='center', fontweight='bold')
    eje.plot(2.136, 15.30, '^', color='saddlebrown', markersize=10, markerfacecolor='none')
    eje.text(2.136 - 0.55, 15.30, 'T', fontsize=10, color='saddlebrown', ha='center', va='center', fontweight='bold')

    for nombre, (x, y) in {'Q': q, 'P Base': (pb[0], pb[1])}.items():
        eje.plot(x, y, 's', color='black', markersize=6)
        eje.text(x + 0.12, y + 0.12, nombre, fontsize=9, fontweight='bold')
    eje.set_xlabel('X (m)')
    eje.set_ylabel('Y (m)')
    eje.set_aspect('equal')
    eje.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=2, fontsize=8)
    eje.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, 'fig_cobertura.png'), dpi=150)
    plt.close(fig)


# ---------- Figura: mapa entregable del SLAM ----------

def mapa_slam():
    bag = os.path.join(RAIZ, 'bags', 'slam', 'entregable_131350', 'entregable_131350_0.mcap')
    log = os.path.join(RAIZ, 'test_real', 'datos_20261106', 'mission_log_ensayo_fase1.csv')
    if not os.path.exists(bag):
        print('Sin el bag del SLAM: no hago fig_mapa_slam.png')
        return
    from evaluar_slam import PoseSlam, componer, filas_log, inversa
    from leer_mcap import leer
    f1 = filas_log(log)
    t0 = float(f1[0]['timestamp'])
    salida = (float(f1[0]['robot_x']), float(f1[0]['robot_y']), float(f1[0]['robot_yaw']))
    m = None
    for _, _, x in leer(bag, ['/map']):
        m = x
    x0, y0, th = componer(salida, inversa(PoseSlam(bag)(t0)))   # del frame map del SLAM al mundo
    res = m['resolution']
    # Esquinas de las celdas en el frame map, y al mundo (la rejilla queda girada: pcolormesh admite eso)
    cx = m['origin'][0] + np.arange(m['width'] + 1) * res
    cy = m['origin'][1] + np.arange(m['height'] + 1) * res
    MX, MY = np.meshgrid(cx, cy)
    WX = x0 + math.cos(th) * MX - math.sin(th) * MY
    WY = y0 + math.sin(th) * MX + math.cos(th) * MY
    d = m['data'].astype(float)
    gris = np.where(d < 0, 0.8, np.where(d >= 65, 0.0, 1.0))          # como el .pgm: desconocido, ocupado, libre

    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 5.2), gridspec_kw={'width_ratios': [2.2, 1]})
    for eje in (a, b):
        eje.pcolormesh(WY, WX, gris, cmap='gray', vmin=0, vmax=1, shading='flat')
        eje.set_aspect('equal')
        eje.set_xlabel('Y (m)')
        eje.set_ylabel('X (m)')
    a.set_xlim(-0.5, 17.8)
    a.set_ylim(10.2, -1.0)
    a.add_patch(Rectangle((15.7, 6.5), 1.6, 1.4, facecolor='none', edgecolor='tab:red', linewidth=1.2))
    a.text(15.0, 6.3, 'R', color='tab:red', fontsize=10, fontweight='bold')
    a.add_patch(Rectangle((14.6, 1.4), 1.2, 1.4, facecolor='none', edgecolor='tab:red', linewidth=1.2))
    a.text(14.0, 1.3, 'T', color='tab:red', fontsize=10, fontweight='bold')
    a.set_title('Mapa de slam_toolbox (fases I y II), en el sistema del mundo', fontsize=10)
    b.set_xlim(15.7, 17.3)
    b.set_ylim(7.9, 6.5)
    b.set_title('Detalle: las 4 patas de R', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, 'fig_mapa_slam.png'), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    arquitectura()
    estados()
    cobertura()
    mapa_slam()
