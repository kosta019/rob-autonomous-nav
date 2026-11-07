#!/usr/bin/env python3
# OFFLINE (host o Docker; solo numpy y yaml): genera escenarios de simulación para Gazebo.
# Un escenario = obstáculos aleatorios (paneles, cajas, personas que pasan) + salida aleatoria, siempre con camino
# posible según el mismo planificador que usa fase1_node. Guarda escenario.yaml y mundo.sdf.
# Uso:  python3 sim/escenarios.py --n 20 --semilla 100 --salida sim/resultados/xxx
#       (crea 20 carpetas <semilla>/ con semillas consecutivas; las que no tienen camino se saltan)
import argparse
import math
import os
import random
import sys

import numpy as np
import yaml

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PKG = os.path.join(RAIZ, 'autonomous_nav')
sys.path.insert(0, os.path.join(PKG, 'autonomous_nav'))
import planificador as pl  # noqa: E402


# ---------- Datos ----------
def cargar():
    with open(os.path.join(os.path.dirname(__file__), 'sim.yaml')) as f:
        sim = yaml.safe_load(f)
    with open(os.path.join(PKG, 'config', 'fase1.yaml')) as f:
        p = yaml.safe_load(f)['fase1_node']['ros__parameters']
    with open(os.path.join(PKG, 'maps', 'holgura_5cm.yaml')) as f:
        m = yaml.safe_load(f)
    meta = {'resolucion': m['resolucion'], 'origen_x': m['origen_x'], 'origen_y': m['origen_y']}
    holgura = np.load(os.path.join(PKG, 'maps', 'holgura_5cm.npy'))
    puntos = dict(zip(p['point_names'], zip(p['point_x'], p['point_y'])))
    return sim, p, holgura, meta, puntos


def esquinas_a_puntos(cx, cy, lx, ly, ang, paso=0.05):
    # Puntos cada 'paso' que cubren el rectángulo (lx x ly) girado 'ang', centrado en (cx, cy)
    xs = np.unique(np.append(np.arange(-lx / 2, lx / 2, paso), lx / 2))
    ys = np.unique(np.append(np.arange(-ly / 2, ly / 2, paso), ly / 2))
    gx, gy = np.meshgrid(xs, ys)
    c, s = math.cos(ang), math.sin(ang)
    return np.stack([cx + c * gx.ravel() - s * gy.ravel(), cy + s * gx.ravel() + c * gy.ravel()], axis=1)


def dist_punto_rect(px, py, ob):
    # Distancia de un punto al rectángulo del obstáculo (0 si está dentro)
    c, s = math.cos(ob['angulo']), math.sin(ob['angulo'])
    dx, dy = px - ob['centro'][0], py - ob['centro'][1]
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ex = max(abs(lx) - ob['tamano'][0] / 2, 0.0)
    ey = max(abs(ly) - ob['tamano'][1] / 2, 0.0)
    return math.hypot(ex, ey)


def patas_estaciones(sim):
    # Centros de las patas de las dos estaciones: [(x, y, estacion)]
    e = sim['estaciones']
    patas = []
    for nombre, est in (('cuadrada', e['cuadrada']), ('triangular', e['triangular'])):
        cx, cy = est['centro']
        g = math.radians(est['giro_grados'])
        if est['patas'] == 4:
            h = est['lado'] / 2
            locales = [(h, h), (-h, h), (-h, -h), (h, -h)]
        else:
            r = est['lado'] / math.sqrt(3)
            locales = [(r * math.cos(math.radians(90 + 120 * k)), r * math.sin(math.radians(90 + 120 * k)))
                       for k in range(3)]
        for lx, ly in locales:
            patas.append((cx + lx * math.cos(g) - ly * math.sin(g), cy + lx * math.sin(g) + ly * math.cos(g), nombre))
    return patas


# ---------- Misión: tramos y puntos clave ----------
def tramos_mision(p, puntos, salida):
    # Lista de puntos (nombre, (x, y)) que recorre la misión desde la salida, y los puntos clave
    nombres = p['waypoint_names']
    ruta = dict(zip(nombres, zip(p['waypoints_x'], p['waypoints_y'])))
    pasos = {'A': ['B', 'D'], 'B': ['D'], 'C': ['D'], 'D': []}[salida]
    seq = [(salida, puntos[salida])] + [(n, ruta[n]) for n in pasos]
    seq += [('E', puntos['E']), ('Door', puntos['Door']), ('Q', puntos['Q']),
            ('final', tuple(p['final_pose'][:2]))]
    clave = [(salida, puntos[salida]), ('E', puntos['E']), ('Door', puntos['Door']), ('Q', puntos['Q']),
             ('Base', puntos['Base'])]
    cob = list(zip(p['phase2_waypoints_x'], p['phase2_waypoints_y']))
    if p.get('phase2_second_coverage'):
        cob += list(zip(p['phase2_second_waypoints_x'], p['phase2_second_waypoints_y']))
    seq += [(f'cob{i}', c) for i, c in enumerate(cob)]
    seq += [('Base', tuple(p['phase2_final_pose'][:2]))]
    clave += [(f'cob{i}', c) for i, c in enumerate(cob)]
    return seq, clave


def hay_camino(p, holgura, meta, obstaculos, seq, front_dist, estacion):
    # Comprueba con el planificador del nodo que cada tramo de la misión tiene camino con el inflado normal.
    # Devuelve (True, ruta densa [(x, y)...]) o (False, motivo).
    pts = []
    for ob in obstaculos:
        pts += [tuple(q) for q in esquinas_a_puntos(ob['centro'][0], ob['centro'][1], ob['tamano'][0],
                                                    ob['tamano'][1], ob['angulo'])]
    h = pl.holgura_con_obstaculos(holgura, meta, pts, p['obstacle_plan_margin'], p['clearance_target'])
    inflado = p['inflation_radius']
    coste = pl.mapa_de_costes(h, inflado, p['clearance_target'], p['clearance_cost_gain'])
    ruta = []
    for (na, a), (nb, b) in zip(seq, seq[1:]):
        cam = pl.planificar_tramo(coste, meta, a, b)
        if cam is None:
            return False, f'sin camino {na}->{nb}'
        ruta += cam
    # Fase III: al menos un lado de la estación cuadrada con su punto de delante libre y alcanzable desde P Base
    cx, cy = estacion
    base = seq[-1][1]
    for k in range(4):
        a = k * math.pi / 2
        q = (cx + front_dist * math.cos(a), cy + front_dist * math.sin(a))
        if pl.holgura_en(h, meta, q[0], q[1]) > inflado and pl.planificar_tramo(coste, meta, base, q) is not None:
            return True, ruta
    return False, 'ningun lado de la estacion alcanzable'


# ---------- Generación ----------
def elegir(rng, dic):
    r, acum = rng.random(), 0.0
    for nombre, d in dic.items():
        acum += d['prob']
        if r < acum:
            return nombre
    return nombre


def colocar_obstaculo(rng, sim, holgura, meta, clave, centros_est, previos):
    so = sim['obstaculos']
    zonas = so['zonas']
    nombres = list(zonas)
    for _ in range(so['intentos']):
        z = rng.choices(nombres, weights=[zonas[n]['peso'] for n in nombres])[0]
        x0, x1, y0, y1 = zonas[z]['rect']
        cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
        if rng.random() < so['prob_panel']:
            tipo = 'panel'
            tam = [round(rng.uniform(so['panel']['largo_min'], so['panel']['largo_max']), 2), so['panel']['grueso']]
            alto = so['panel']['alto']
        else:
            tipo = 'caja'
            tam = [so['caja']['lado']] * 2
            alto = so['caja']['lado']
        ob = {'tipo': tipo, 'centro': [round(cx, 3), round(cy, 3)], 'tamano': tam, 'alto': alto,
              'angulo': round(rng.uniform(0, math.pi), 3), 'zona': z}
        pts = esquinas_a_puntos(cx, cy, tam[0], tam[1], ob['angulo'], 0.02)
        if min(pl.holgura_en(holgura, meta, a, b) for a, b in pts) < so['holgura_min_pared']:
            continue
        if any(dist_punto_rect(px, py, ob) < so['dist_min_puntos'] for _, (px, py) in clave):
            continue
        if any(dist_punto_rect(px, py, ob) < so['dist_min_estacion'] for px, py in centros_est):
            continue
        if any(min(dist_punto_rect(a, b, o) for a, b in pts) < 0.15 for o in previos):
            continue
        return ob
    return None


def colocar_persona(rng, sim, holgura_ob, meta, ruta, clave, centros_est, previas):
    # Segmento que cruza la ruta prevista, perpendicular a ella, con los extremos fuera de la ruta
    sp = sim['persona']
    ruta = np.array(ruta)
    cl = np.array([c for _, c in clave])
    for _ in range(300):
        i = rng.randrange(5, len(ruta) - 5)
        c = ruta[i]
        if np.min(np.hypot(*(cl - c).T)) < sp['dist_ruta_min_punto_clave']:
            continue
        t = ruta[i + 5] - ruta[i - 5]
        if np.hypot(*t) < 1e-6:
            continue
        n = np.array([-t[1], t[0]]) / np.hypot(*t)
        if rng.random() < 0.5:
            n = -n
        # Recorre cada lado mientras haya sitio; el extremo debe quedar a >= fuera_de_ruta de la ruta
        lados = []
        for sgn in (1, -1):
            libre, lejos = 0.0, None
            for s in np.arange(0.05, sp['largo_max'] / 2 + 1e-9, 0.05):
                q = c + sgn * s * n
                if pl.holgura_en(holgura_ob, meta, q[0], q[1]) < sp['holgura_min']:
                    break
                libre = s
                if lejos is None and np.min(np.hypot(*(ruta - q).T)) >= sp['fuera_de_ruta']:
                    lejos = s
            lados.append(libre if lejos is not None else None)
        if None in lados or sum(lados) < sp['largo_min']:
            continue
        a, b = c + lados[0] * n, c - lados[1] * n
        muestras = [a + (b - a) * k / 40 for k in range(41)]
        if any(np.min(np.hypot(*(cl - q).T)) < sp['dist_min_puntos'] for q in muestras):
            continue
        if any(math.hypot(q[0] - e[0], q[1] - e[1]) < sim['obstaculos']['dist_min_estacion']
               for q in muestras for e in centros_est):
            continue
        return {'extremo_a': [round(float(a[0]), 3), round(float(a[1]), 3)],
                'extremo_b': [round(float(b[0]), 3), round(float(b[1]), 3)],
                'velocidad': round(rng.uniform(*sp['velocidad']), 2),
                'pausa': round(rng.uniform(*sp['pausa']), 1),
                'arranque': round(rng.uniform(*sp['arranque_s']), 1),
                'diametro': sp['diametro'], 'alto': sp['alto']}
    return None


def generar(semilla, sim, p, holgura, meta, puntos):
    # Devuelve el escenario (dict) o (None, motivo)
    rng = random.Random(semilla)
    dificultad = elegir(rng, sim['obstaculos']['dificultad'])
    n = rng.randint(*sim['obstaculos']['dificultad'][dificultad]['n'])
    salida = rng.choice(sim['salidas'])
    seq, clave = tramos_mision(p, puntos, salida)
    estacion = sim['estaciones']['cuadrada']['centro']
    centros_est = [sim['estaciones']['cuadrada']['centro'], sim['estaciones']['triangular']['centro']]
    obstaculos = []
    for _ in range(n):
        ob = colocar_obstaculo(rng, sim, holgura, meta, clave[:1] + clave[1:], centros_est, obstaculos)
        if ob is None:
            return None, 'no cabe un obstaculo'
        obstaculos.append(ob)
    ok, info = hay_camino(p, holgura, meta, obstaculos, seq, p['phase3_front_dist'], estacion)
    if not ok:
        return None, info
    ruta = info
    # Personas
    sp = sim['persona']
    if dificultad == 'exigente':
        npers = rng.randint(*sp['n_exigente'])
    else:
        npers = 1 if rng.random() < sp['prob_tranquila_media'] else 0
    pts = []
    for ob in obstaculos:
        pts += [tuple(q) for q in esquinas_a_puntos(ob['centro'][0], ob['centro'][1], ob['tamano'][0],
                                                    ob['tamano'][1], ob['angulo'])]
    h_ob = pl.holgura_con_obstaculos(holgura, meta, pts, 0.0, 1.0)
    personas = []
    for _ in range(npers):
        pe = colocar_persona(rng, sim, h_ob, meta, ruta, clave, centros_est, personas)
        if pe is None:
            return None, 'no cabe una persona'
        personas.append(pe)
    return {'semilla': semilla, 'dificultad': dificultad, 'salida': salida,
            'pose_salida': [float(puntos[salida][0]), float(puntos[salida][1]), 0.0],
            'obstaculos': obstaculos, 'personas': personas}, 'ok'


# ---------- SDF ----------
def sdf_caja(nombre, cx, cy, z, lx, ly, lz, ang, color):
    g = f'<box><size>{lx:.3f} {ly:.3f} {lz:.3f}</size></box>'
    return (f'    <model name="{nombre}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 {ang:.4f}</pose>'
            f'<link name="l"><collision name="c"><geometry>{g}</geometry></collision>'
            f'<visual name="v"><geometry>{g}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
            f'</material></visual></link></model>\n')


def sdf_cilindro(nombre, cx, cy, z, r, alto, color):
    g = f'<cylinder><radius>{r}</radius><length>{alto}</length></cylinder>'
    return (f'    <model name="{nombre}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>'
            f'<link name="l"><collision name="c"><geometry>{g}</geometry></collision>'
            f'<visual name="v"><geometry>{g}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
            f'</material></visual></link></model>\n')


def sdf_persona(nombre, x, y, d, alto):
    # Modelo dinámico sin gravedad que se mueve por velocidad (VelocityControl); tiene visual, que es lo que ve el LiDAR
    g = f'<cylinder><radius>{d / 2}</radius><length>{alto}</length></cylinder>'
    return (f'    <model name="{nombre}"><pose>{x:.4f} {y:.4f} {alto / 2:.3f} 0 0 0</pose>'
            f'<link name="l"><gravity>false</gravity>'
            f'<inertial><mass>70</mass><inertia><ixx>50</ixx><iyy>50</iyy><izz>50</izz></inertia></inertial>'
            f'<collision name="c"><geometry>{g}</geometry></collision>'
            f'<visual name="v"><geometry>{g}</geometry><material><ambient>0.9 0.2 0.2 1</ambient>'
            f'<diffuse>0.9 0.2 0.2 1</diffuse></material></visual></link>'
            f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>'
            f'</model>\n')


def sdf_estaciones(sim):
    e = sim['estaciones']
    s = ''
    for k, (x, y, est) in enumerate(patas_estaciones(sim)):
        s += sdf_cilindro(f'pata_{est}_{k}', x, y, e['alto_pata'] / 2, e['radio_pata'], e['alto_pata'], '0.2 0.2 0.8 1')
    for nombre in ('cuadrada', 'triangular'):
        est = e[nombre]
        s += sdf_caja(f'tablero_{nombre}', est['centro'][0], est['centro'][1], e['tablero_z'], est['lado'] + 0.05,
                      est['lado'] + 0.05, e['tablero_grueso'], math.radians(est['giro_grados']), '0.8 0.7 0.4 1')
    return s


def escribir_mundo(esc, sim, ruta):
    with open(os.path.join(RAIZ, sim['mundo_base'])) as f:
        sdf = f.read()
    rtf = float(sim['real_time_factor'])
    sdf = sdf.replace('<real_time_update_rate>1000.0</real_time_update_rate>',
                      f'<real_time_update_rate>{0 if rtf == 0 else 1000 * rtf}</real_time_update_rate>')
    sdf = sdf.replace('<real_time_factor>1</real_time_factor>', f'<real_time_factor>{rtf}</real_time_factor>')
    extra = '    <!-- Estaciones y obstáculos del escenario (sim/escenarios.py) -->\n' + sdf_estaciones(sim)
    for k, ob in enumerate(esc['obstaculos']):
        extra += sdf_caja(f'{ob["tipo"]}_{k}', ob['centro'][0], ob['centro'][1], ob['alto'] / 2, ob['tamano'][0],
                          ob['tamano'][1], ob['alto'], ob['angulo'], '0.7 0.45 0.2 1')
    for k, pe in enumerate(esc['personas']):
        extra += sdf_persona(f'persona_{k}', pe['extremo_a'][0], pe['extremo_a'][1], pe['diametro'], pe['alto'])
    sdf = sdf.replace('  </world>', extra + '  </world>')
    with open(ruta, 'w') as f:
        f.write(sdf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=1)
    ap.add_argument('--semilla', type=int, default=0)
    ap.add_argument('--salida', required=True)
    ap.add_argument('--forzar', help='dificultad:salida[:persona], p. ej. tranquila:A:persona (busca una semilla que lo cumpla)')
    a = ap.parse_args()
    sim, p, holgura, meta, puntos = cargar()
    semilla, hechos, saltados = a.semilla, 0, 0
    while hechos < a.n:
        esc, motivo = generar(semilla, sim, p, holgura, meta, puntos)
        if esc is not None and a.forzar:
            d, s, *resto = a.forzar.split(':')    # dificultad:salida[:persona]
            if esc['dificultad'] != d or esc['salida'] != s or (resto and not esc['personas']):
                esc, motivo = None, 'no es la pedida'
        if esc is None:
            saltados += 1
            print(f'semilla {semilla}: descartada ({motivo})', file=sys.stderr)
        else:
            d = os.path.join(a.salida, f'{semilla:05d}')
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, 'escenario.yaml'), 'w') as f:
                yaml.safe_dump(esc, f, sort_keys=False)
            escribir_mundo(esc, sim, os.path.join(d, 'mundo.sdf'))
            print(f'{d} {esc["dificultad"]} salida={esc["salida"]} obstaculos={len(esc["obstaculos"])} '
                  f'personas={len(esc["personas"])}')
            hechos += 1
        semilla += 1
    print(f'{hechos} escenarios, {saltados} semillas descartadas', file=sys.stderr)


if __name__ == '__main__':
    main()
