# Resumen de la corrección con el mapa a partir de un Mission Log.
# Uso:  python3 test_real/corr_mapa_log.py mission_log_XXXX.csv [segundos entre filas, 5 por defecto]
import csv
import sys
import time

paso = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
ultimo = 0.0
filas = 0
for a in csv.DictReader(open(sys.argv[1])):
    e = a['event']
    if e.startswith('RELOC') or e.startswith('WAYPOINT') or e.startswith('OBLIGATORIO'):
        print(time.strftime('%H:%M:%S', time.localtime(float(a['timestamp']))), e)
    if e.startswith('CORR_MAPA'):
        filas += 1
        t = float(a['timestamp'])
        if t - ultimo >= paso:
            ultimo = t
            print(time.strftime('%H:%M:%S', time.localtime(t)), f"({a['robot_x']}, {a['robot_y']})", e)
print(f'{filas} correcciones')
