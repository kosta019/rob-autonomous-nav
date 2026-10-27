#!/usr/bin/env python3
# OFFLINE (en el portátil): genera el mundo de Gazebo del laboratorio a partir de maps/mapa_1cm.png.
# Usa Pillow, así que no se ejecuta en el robot: el .sdf generado se sube al repo.
#
# Uso:  python3 autonomous_nav/scripts/generar_mundo_lab.py   ->  worlds/laboratorio.sdf
#
# - Paredes: los píxeles negros agrupados en rectángulos (a 1 cm, la geometría exacta del mapa).
# - El origen del mundo de Gazebo es el del enunciado: cada bloque va en sus coordenadas (X, Y).
#   Así la pose real que da Gazebo está directamente en coordenadas del enunciado.
# - Zonas prohibidas (suelo irregular): parches visuales SIN colisión, para ver si el robot las pisa.
# - Puntos del enunciado: discos visuales sin colisión.
# - Mesas entre B y C: no están, faltan las medidas (TODO).
import os

import numpy as np
import yaml
from PIL import Image

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # carpeta del paquete autonomous_nav/
ALTURA_PARED = 0.5   # m (el LiDAR está a 0.17 m)


def cargar_parametros():
    with open(os.path.join(RAIZ, 'config', 'fase1.yaml')) as f:
        return yaml.safe_load(f)['fase1_node']['ros__parameters']


def rectangulos(ocupado):
    # Agrupa los píxeles ocupados en rectángulos: tramos horizontales de cada fila que se
    # alargan hacia abajo mientras la fila siguiente tenga exactamente el mismo tramo.
    # Devuelve (fila0, fila1, col0, col1), con fila1 y col1 excluidos.
    abiertos = {}
    resultado = []
    for i in range(ocupado.shape[0] + 1):
        tramos = set()
        if i < ocupado.shape[0]:
            d = np.diff(np.concatenate([[0], ocupado[i].astype(int), [0]]))
            tramos = set(zip(np.where(d == 1)[0], np.where(d == -1)[0]))
        nuevos = {}
        for t in tramos:
            nuevos[t] = abiertos.pop(t) if t in abiertos else i
        for (c0, c1), i0 in abiertos.items():
            resultado.append((i0, i, int(c0), int(c1)))
        abiertos = nuevos
    return resultado


def caja(nombre, x, y, z, sx, sy, sz, color, colision=True):
    material = f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
    geometria = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    col = f'<collision name="c">{geometria}</collision>' if colision else ''
    return (f'    <model name="{nombre}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
            f'<link name="l">{col}<visual name="v">{geometria}{material}</visual></link></model>\n')


def disco(nombre, x, y, color):
    material = f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
    return (f'    <model name="{nombre}"><static>true</static><pose>{x:.3f} {y:.3f} 0.001 0 0 0</pose>'
            f'<link name="l"><visual name="v"><geometry><cylinder><radius>0.04</radius><length>0.002</length>'
            f'</cylinder></geometry>{material}</visual></link></model>\n')


def main():
    p = cargar_parametros()
    res = p['map_resolution']
    origen_x = -p['map_origin_row'] * res
    origen_y = -p['map_origin_col'] * res

    imagen = np.array(Image.open(os.path.join(RAIZ, p['map_image'])).convert('L'))
    ocupado = imagen < 128
    for fila, col in zip(p['map_remove_rows'], p['map_remove_cols']):
        ocupado[fila, col] = False
    rects = rectangulos(ocupado)

    modelos = ''
    for k, (i0, i1, j0, j1) in enumerate(rects):
        x = origen_x + (i0 + i1) / 2 * res
        y = origen_y + (j0 + j1) / 2 * res
        modelos += caja(f'pared_{k}', x, y, ALTURA_PARED / 2, (i1 - i0) * res, (j1 - j0) * res, ALTURA_PARED,
                        '0.6 0.6 0.6 1')
    for n, x0, x1, y0, y1 in zip(p['nogo_names'], p['nogo_x_min'], p['nogo_x_max'], p['nogo_y_min'], p['nogo_y_max']):
        modelos += caja(f'nogo_{n}', (x0 + x1) / 2, (y0 + y1) / 2, 0.001, x1 - x0, y1 - y0, 0.002,
                        '1 0.6 0.1 1', colision=False)
    for n, x, y in zip(p['point_names'], p['point_x'], p['point_y']):
        color = '1 0.4 0 1' if n in p['mandatory_waypoints'] else '0.1 0.6 0.1 1'
        modelos += disco(f'punto_{n}', x, y, color)

    with open(os.path.join(RAIZ, 'scripts', 'plantilla_mundo.sdf')) as f:
        sdf = f.read().replace('<!-- MODELOS -->\n', modelos)
    os.makedirs(os.path.join(RAIZ, 'worlds'), exist_ok=True)
    ruta = os.path.join(RAIZ, 'worlds', 'laboratorio.sdf')
    with open(ruta, 'w') as f:
        f.write(sdf)
    print(f'{len(rects)} bloques de pared, {len(p["nogo_names"])} zonas prohibidas, '
          f'{len(p["point_names"])} puntos -> {ruta}')


if __name__ == '__main__':
    main()
