#!/usr/bin/env python3
# OFFLINE (host): analiza una batería de misiones de sim/bateria.sh.
# Uso:  python3 sim/analizar.py sim/resultados/<fecha_hora>
# Escribe en esa carpeta: resumen.csv (una fila por misión), resumen.md (tasas y peores casos) y fig_<id>.png por cada
# misión fallida o con choque. Las distancias salen de la pose REAL de Gazebo (real.csv), no de la /odom.
# Solo numpy, yaml, matplotlib y Pillow.
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import yaml
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import escenarios as esc_mod  # noqa: E402

RAIZ = esc_mod.RAIZ
EVENTOS = ['BLOQUEO', 'REPLANIFICACION', 'SIN_CAMINO', 'RODEO_LARGO', 'PARADA_EMERGENCIA', 'OBLIGATORIO_TAPADO']


def cargar_mapa():
    # Píxeles ocupados del mapa de 1 cm, sin la mota suelta de (502, 1217)
    img = np.array(Image.open(os.path.join(RAIZ, 'autonomous_nav', 'maps', 'mapa_1cm.png')).convert('L'))
    ocupado = img < 128
    ocupado[502, 1217] = False
    return img, ocupado


def dist_paredes(ocupado, xs, ys):
    # Distancia (m) de cada punto al píxel ocupado más cercano (ventana de +-0.5 m; 9.9 si no hay ninguno)
    filas, cols = ocupado.shape
    d = np.full(len(xs), 9.9)
    w = 50
    for k, (x, y) in enumerate(zip(xs, ys)):
        f, c = int(math.floor(100 * x + 71)), int(math.floor(100 * y + 2))
        f0, f1, c0, c1 = max(f - w, 0), min(f + w + 1, filas), max(c - w, 0), min(c + w + 1, cols)
        if f0 >= f1 or c0 >= c1:
            continue
        ff, cc = np.nonzero(ocupado[f0:f1, c0:c1])
        if len(ff):
            px = (ff + f0 + 0.5 - 71) / 100
            py = (cc + c0 + 0.5 - 2) / 100
            d[k] = max(np.min(np.hypot(px - x, py - y)) - 0.005, 0.0)
    return d


def leer_log(ruta):
    with open(ruta) as f:
        filas = list(csv.DictReader(f))
    for r in filas:
        r['t'] = float(r['timestamp'])
    return filas


def leer_real(ruta, npers):
    datos = np.loadtxt(ruta, delimiter=',', skiprows=1, ndmin=2)
    return datos


def recorrido(t, x, y, t0, t1):
    m = (t >= t0) & (t <= t1)
    if m.sum() < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(x[m]), np.diff(y[m]))))


def atascos(t, x, y, t0, t1, ventana=120.0, minimo=0.1):
    # Ventanas disjuntas de 'ventana' s en las que el robot no se aleja 'minimo' m de donde estaba al empezar
    n, seg, i = 0, 0.0, 0
    while i < len(t):
        if t[i] < t0:
            i += 1
            continue
        if t[i] + ventana > t1:
            break
        j = np.searchsorted(t, t[i] + ventana)
        if np.max(np.hypot(x[i:j] - x[i], y[i:j] - y[i])) < minimo:
            n += 1
            seg += ventana
            i = j
        else:
            i += 1
    return n, seg


def analizar_mision(d, mapa_ocupado, sim):
    esc = yaml.safe_load(open(os.path.join(d, 'escenario.yaml')))
    logs = sorted(glob.glob(os.path.join(d, 'mission_log*.csv')))
    r = {'id': os.path.basename(d), 'semilla': esc['semilla'], 'dificultad': esc['dificultad'], 'salida': esc['salida'],
         'n_obstaculos': len(esc['obstaculos']), 'n_personas': len(esc['personas'])}
    if not logs or not os.path.exists(os.path.join(d, 'real.csv')):
        r.update({'fallo': 'sin_datos', 'completada': False})
        return r, None
    log = leer_log(logs[0])
    if len(log) < 2:
        r.update({'fallo': 'log_vacio', 'completada': False})
        return r, None
    npers = len(esc['personas'])
    real = leer_real(os.path.join(d, 'real.csv'), npers)
    if len(real) < 10:
        r.update({'fallo': 'sin_pose_real', 'completada': False})
        return r, None
    t, x, y, yaw = real[:, 0], real[:, 1], real[:, 2], real[:, 3]
    pers = [(real[:, 4 + 2 * k], real[:, 5 + 2 * k]) for k in range(npers)]
    t_ini, t_fin = log[0]['t'], log[-1]['t']
    eventos = [r_['event'] for r_ in log]

    # Fases completadas, tiempo y distancia por fase
    def hay(prefijo):
        return any(e.startswith(prefijo) for e in eventos)
    r['fase1'] = hay('FASE I COMPLETADA')
    r['fase2'] = hay('FASE II COMPLETADA')
    r['fase3'] = hay('FASE III COMPLETADA')
    for fase in ('I', 'II', 'III'):
        ts = [e['t'] for e in log if e['current_phase'] == fase]
        if ts:
            r[f't_fase{fase}'] = round(ts[-1] - ts[0], 1)
            r[f'dist_fase{fase}'] = round(recorrido(t, x, y, ts[0], ts[-1]), 2)
        else:
            r[f't_fase{fase}'] = r[f'dist_fase{fase}'] = ''
    r['completada'] = r['fase3']
    r['t_total'] = round(t_fin - t_ini, 1)
    r['fin'] = open(os.path.join(d, 'fin.txt')).read().strip() if os.path.exists(os.path.join(d, 'fin.txt')) else ''
    r['reloj_rtf'] = open(os.path.join(d, 'tiempos.txt')).read().strip() if os.path.exists(os.path.join(d, 'tiempos.txt')) else ''

    # Distancias mínimas de la huella (círculo de radio_robot) a paredes, obstáculos, patas y personas
    rr = sim['radio_robot']
    hp = dist_paredes(mapa_ocupado, x, y) - rr
    ho = np.full(len(t), 9.9)
    # Obstáculos rectangulares, vectorizado por obstáculo
    for ob in esc['obstaculos']:
        c, s = math.cos(ob['angulo']), math.sin(ob['angulo'])
        dx, dy = x - ob['centro'][0], y - ob['centro'][1]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        ex = np.maximum(np.abs(lx) - ob['tamano'][0] / 2, 0.0)
        ey = np.maximum(np.abs(ly) - ob['tamano'][1] / 2, 0.0)
        ho = np.minimum(ho, np.hypot(ex, ey) - rr)
    hl = np.full(len(t), 9.9)
    for px, py, _ in esc_mod.patas_estaciones(sim):
        hl = np.minimum(hl, np.hypot(x - px, y - py) - sim['estaciones']['radio_pata'] - rr)
    h_est = np.minimum(np.minimum(hp, ho), hl)
    k = int(np.argmin(h_est))
    r['min_hueco_m'] = round(float(h_est[k]), 3)
    r['min_hueco_con'] = ['pared', 'obstaculo', 'pata'][int(np.argmin([hp[k], ho[k], hl[k]]))]
    r['min_hueco_pared'] = round(float(hp.min()), 3)
    r['min_hueco_obstaculo'] = round(float(ho.min()), 3) if esc['obstaculos'] else ''
    r['min_hueco_pata'] = round(float(hl.min()), 3)
    # Choque con lo estático: huella solapada con algo que no se mueve
    r['choque'] = bool(h_est[k] < 0)
    r['t_min_hueco'] = round(float(t[k]), 1)

    # Persona: distancia mínima, contacto y si el robot se movía
    v = np.hypot(np.gradient(x, t), np.gradient(y, t))
    w = np.abs(np.gradient(np.unwrap(yaw), t))
    movil = (v > 0.02) | (w > 0.1)
    r['min_hueco_persona'] = ''
    r['contacto_persona'] = False
    r['contacto_persona_robot_movil'] = False
    h_per = np.full(len(t), 9.9)
    for px, py in pers:
        h_per = np.minimum(h_per, np.hypot(x - px, y - py) - 0.15 - rr)
    if npers:
        r['min_hueco_persona'] = round(float(h_per.min()), 3)
        r['contacto_persona'] = bool(h_per.min() < 0)
        r['contacto_persona_robot_movil'] = bool(np.any((h_per < 0) & movil))

    # Atascos (sin avanzar 0.1 m en 2 min). Se ignora lo que pasa tras acabar la misión.
    t_fin_util = t_fin
    for e in log:
        if e['event'].startswith('FASE III COMPLETADA') or e['event'].startswith('FASE_III_'):
            t_fin_util = e['t']
    r['atascos'], r['t_atascado_s'] = atascos(t, x, y, t_ini, t_fin_util)

    # Contadores de eventos y paradas de emergencia por la persona
    for ev in EVENTOS:
        r['n_' + ev] = sum(1 for e in eventos if e.startswith(ev))
    pe_pers = 0
    for e in log:
        if e['event'].startswith('PARADA_EMERGENCIA') and e['obstacle_x'] != '':
            ox, oy = float(e['obstacle_x']), float(e['obstacle_y'])
            for px, py in pers:
                if math.hypot(np.interp(e['t'], t, px) - ox, np.interp(e['t'], t, py) - oy) < 0.6:
                    pe_pers += 1
                    break
    r['n_PARADA_EMERGENCIA_persona'] = pe_pers

    # Fase III
    errores = [float(e['error']) for e in log if e['event'].startswith('FASE III COMPLETADA') and e['error'] != '']
    r['f3_error_cm'] = round(errores[-1] * 100, 1) if errores else ''
    r['f3_realinear'] = sum(1 for e in eventos if e.startswith('FASE3_REALINEAR'))
    r['f3_abandonada'] = hay('FASE_III_ABANDONADA')
    r['f3_sin_estacion'] = hay('FASE_III_SIN_ESTACION') or hay('FASE_III_SIN_HUECO')
    r['f3_fuera_tolerancia'] = hay('APARCADO_FUERA_DE_TOLERANCIA')

    # Fallo de la simulación, no del robot: la física de una persona revienta (pose de miles de km) y Gazebo se congela
    datos_sim = np.loadtxt(os.path.join(d, 'real.csv'), delimiter=',', skiprows=1, ndmin=2)
    r['fallo_sim'] = datos_sim.shape[1] > 4 and bool(np.nanmax(np.abs(datos_sim[:, 4:])) > 100)

    # Motivo del fallo
    motivos = []
    if r['fallo_sim']:
        motivos.append('SIMULACION: persona fuera de control (no es fallo del robot)')
    if r['choque']:
        motivos.append(f"choque con {r['min_hueco_con']} ({r['min_hueco_m']:.3f} m)")
    if r['contacto_persona_robot_movil']:
        motivos.append('contacto con persona (robot en movimiento)')
    if not r['completada']:
        if not r['fase1']:
            motivos.append('no termina la fase I')
        elif not r['fase2']:
            motivos.append('no termina la fase II')
        else:
            motivos.append('fase III: ' + ('abandonada' if r['f3_abandonada'] else
                                           'sin estacion/hueco' if r['f3_sin_estacion'] else 'no termina'))
        if r['fin'].startswith('TIEMPO'):
            motivos.append('tiempo limite')
    if r['atascos']:
        motivos.append(f"{r['atascos']} atasco(s)")
    r['fallo'] = '; '.join(motivos)
    datos = {'esc': esc, 't': t, 'x': x, 'y': y, 'pers': pers, 'k_min': k, 'log': log, 'resultado': r}
    return r, datos


def figura(datos, ruta, img):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    esc, r = datos['esc'], datos['resultado']
    fig, ax = plt.subplots(figsize=(13, 8))
    # Eje horizontal: Y; vertical: X hacia abajo, como la figura del enunciado
    ax.imshow(img, cmap='gray', extent=[-0.02, 17.80, 9.94, -0.71], alpha=0.5)
    for ob in esc['obstaculos']:
        c, s = math.cos(ob['angulo']), math.sin(ob['angulo'])
        hx, hy = ob['tamano'][0] / 2, ob['tamano'][1] / 2
        esq = [(ob['centro'][0] + c * a - s * b, ob['centro'][1] + s * a + c * b) for a, b in
               ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        ax.add_patch(Polygon([(q[1], q[0]) for q in esq], color='tab:orange', alpha=0.9))
    for px, py, _ in esc_mod.patas_estaciones(yaml.safe_load(open(os.path.join(os.path.dirname(__file__), 'sim.yaml')))):
        ax.add_patch(Circle((py, px), 0.025, color='tab:blue'))
    for pe in esc['personas']:
        ax.plot([pe['extremo_a'][1], pe['extremo_b'][1]], [pe['extremo_a'][0], pe['extremo_b'][0]], 'r--', lw=1)
    sc = ax.scatter(datos['y'], datos['x'], c=datos['t'], s=3, cmap='viridis', zorder=3)
    plt.colorbar(sc, ax=ax, label='t (s simulados)', shrink=0.6)
    k = datos['k_min']
    ax.add_patch(Circle((datos['y'][k], datos['x'][k]), 0.105, fill=False, color='red', lw=1.5, zorder=4))
    ax.annotate(f"hueco mínimo {r['min_hueco_m']:.3f} m con {r['min_hueco_con']}\nt={r['t_min_hueco']} s",
                (datos['y'][k], datos['x'][k]), color='red', fontsize=8, zorder=5,
                xytext=(10, 10), textcoords='offset points')
    for e in datos['log']:   # sucesos importantes del Mission Log sobre el recorrido
        if e['event'].startswith(('BLOQUEO', 'PARADA_EMERGENCIA', 'SIN_CAMINO')) and e['robot_x'] != '':
            ax.plot(float(e['robot_y']), float(e['robot_x']), 'x', color='magenta', ms=4, zorder=4)
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('X (m)')
    ax.set_title(f"{r['id']}  salida {r['salida']}  {r['dificultad']}  ({r['n_obstaculos']} obst., {r['n_personas']} pers.)"
                 f"\n{r['fallo'] or 'sin fallo'}")
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(ruta, dpi=90)
    plt.close(fig)


def tasa(filas, f):
    n = len(filas)
    return f'{sum(1 for r in filas if f(r))}/{n}' if n else '-'


def tabla(filas, clave):
    out = [f'| {clave} | misiones | fase I | fase II | fase III | choques | atascos | contacto persona (robot móvil) |',
           '|---|---|---|---|---|---|---|---|']
    for v in sorted({r[clave] for r in filas}):
        g = [r for r in filas if r[clave] == v]
        out.append(f"| {v} | {len(g)} | {tasa(g, lambda r: r.get('fase1'))} | {tasa(g, lambda r: r.get('fase2'))} | "
                   f"{tasa(g, lambda r: r.get('fase3'))} | {tasa(g, lambda r: r.get('choque'))} | "
                   f"{tasa(g, lambda r: r.get('atascos'))} | {tasa(g, lambda r: r.get('contacto_persona_robot_movil'))} |")
    return out


def gravedad(r):
    # Mayor = peor: choque > contacto móvil con persona > misión sin completar > atasco > paradas de emergencia
    g = 0.0
    if r.get('choque'):
        g += 1000 - 100 * r['min_hueco_m']
    if r.get('contacto_persona_robot_movil'):
        g += 800
    if not r.get('completada'):
        g += 300 + 50 * (3 - sum(1 for k in ('fase1', 'fase2', 'fase3') if r.get(k)))
    g += 20 * r.get('atascos', 0) + r.get('n_PARADA_EMERGENCIA', 0) + r.get('n_SIN_CAMINO', 0) * 0.1
    return g


def main():
    carpeta = os.path.abspath(sys.argv[1])
    with open(os.path.join(os.path.dirname(__file__), 'sim.yaml')) as f:
        sim = yaml.safe_load(f)
    img, ocupado = cargar_mapa()
    filas, figuras = [], []
    for d in sorted(glob.glob(os.path.join(carpeta, '*/'))):
        d = d.rstrip('/')
        if not os.path.exists(os.path.join(d, 'escenario.yaml')):
            continue
        print('analizando', os.path.basename(d))
        r, datos = analizar_mision(d, ocupado, sim)
        filas.append(r)
        if datos is not None and (r['fallo'] or r['choque']):
            ruta = os.path.join(carpeta, f"fig_{r['id']}.png")
            figura(datos, ruta, img)
            figuras.append(ruta)
    columnas = []
    for r in filas:
        columnas += [k for k in r if k not in columnas]
    with open(os.path.join(carpeta, 'resumen.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columnas)
        w.writeheader()
        w.writerows(filas)

    md = [f'# Resumen de la batería {os.path.basename(carpeta)}', '', f'{len(filas)} misiones analizadas.', '',
          f"- Completadas (fase III): {tasa(filas, lambda r: r.get('completada'))}",
          f"- Con choque: {tasa(filas, lambda r: r.get('choque'))}",
          f"- Con contacto con persona y robot en movimiento: {tasa(filas, lambda r: r.get('contacto_persona_robot_movil'))}",
          f"- Con atasco: {tasa(filas, lambda r: r.get('atascos'))}", '']
    ok = [r for r in filas if r.get('t_total')]
    if ok:
        md.append(f"- Tiempo simulado medio por misión: {np.mean([r['t_total'] for r in ok]):.0f} s; "
                  f"mínimo hueco medio {np.mean([r['min_hueco_m'] for r in ok if 'min_hueco_m' in r]):.3f} m")
        md.append('')
    md += ['## Por dificultad', ''] + tabla(filas, 'dificultad') + ['', '## Por salida', ''] + tabla(filas, 'salida')
    md += ['', '## Los 10 peores casos', '', '| id (semilla) | salida | dificultad | motivo | hueco mínimo (m) | figura |',
           '|---|---|---|---|---|---|']
    for r in sorted(filas, key=gravedad, reverse=True)[:10]:
        fig = f"fig_{r['id']}.png" if os.path.exists(os.path.join(carpeta, f"fig_{r['id']}.png")) else ''
        md.append(f"| {r['id']} | {r['salida']} | {r['dificultad']} | {r.get('fallo') or 'sin fallo'} | "
                  f"{r.get('min_hueco_m', '')} | {fig} |")
    with open(os.path.join(carpeta, 'resumen.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
