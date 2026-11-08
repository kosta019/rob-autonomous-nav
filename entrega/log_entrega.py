# Mission Log para la entrega: el de una ejecución, con los obstáculos solo del pasillo, como pide el enunciado
# ("position of obstacles detected within the passadis area"). Se mantienen todas las demás filas: la pose periódica,
# los eventos, la estación y sus patas. La zona del pasillo es pillar_zone de config/fase1.yaml.
# Uso (desde la raíz del repo):  python3 entrega/log_entrega.py <mission_log_FECHA.csv> [entrega/mission_log.csv]
import csv
import os
import sys

import yaml

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    entrada = sys.argv[1]
    salida = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RAIZ, 'entrega', 'mission_log.csv')
    with open(os.path.join(RAIZ, 'autonomous_nav', 'config', 'fase1.yaml')) as f:
        x0, x1, y0, y1 = yaml.safe_load(f)['fase1_node']['ros__parameters']['pillar_zone']
    with open(entrada, newline='') as f:
        filas = list(csv.DictReader(f))
    columnas = list(filas[0].keys())
    quedan, quitados = [], 0
    for r in filas:
        if r['event'] == 'OBSTACULO':
            ox, oy = float(r['obstacle_x']), float(r['obstacle_y'])
            if not (x0 <= ox <= x1 and y0 <= oy <= y1):
                quitados += 1
                continue
        quedan.append(r)
    with open(salida, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columnas)
        w.writeheader()
        w.writerows(quedan)
    n_obst = sum(r['event'] == 'OBSTACULO' for r in quedan)
    n_pilar = sum(r['event'].startswith('PILAR') for r in quedan)
    print(f'{salida}: {len(quedan)} filas; {n_obst} obstáculos del pasillo ({quitados} de fuera quitados), '
          f'{n_pilar} patas de estaciones')


if __name__ == '__main__':
    main()
