# Offline (scipy): genera maps/distancias_2cm.npy y .yaml para la corrección con el mapa de fase1_node.
# Para cada punto del plano: distancia a la pared visible más cercana y su gradiente (la normal de la pared).
# Las paredes salen de maps/paredes.yaml, con las caras que ve el LiDAR (canaleta incluida).
# Solo cuenta como pared el borde que toca el espacio libre de la sala: el interior de una zona negra no da puntos
# "falsamente buenos", y lo que cruza una pared sigue teniendo distancia y empuja de vuelta.
# Uso (desde autonomous_nav/):  python3 scripts/generar_mapa_distancias.py
import numpy as np
import yaml
from scipy.ndimage import binary_dilation, distance_transform_edt, gaussian_filter, label

RES = 0.01              # m/celda con la que se calcula
FACTOR = 2              # se guarda 1 de cada 2 celdas (2 cm)
MARGEN = 0.60           # m alrededor de las paredes
SUAVIZADO = 0.015       # m: quita el pico de la distancia justo sobre la pared

p = yaml.safe_load(open('maps/paredes.yaml'))
c = yaml.safe_load(open('config/fase1.yaml'))['fase1_node']['ros__parameters']
x0 = min(p['x_min']) - MARGEN
y0 = min(p['y_min']) - MARGEN
filas = int(np.ceil((max(p['x_max']) + MARGEN - x0) / RES))
cols = int(np.ceil((max(p['y_max']) + MARGEN - y0) / RES))

ocupado = np.zeros((filas, cols), dtype=bool)
for a, b, e, f in zip(p['x_min'], p['x_max'], p['y_min'], p['y_max']):
    ocupado[int(round((a - x0) / RES)):int(round((b - x0) / RES)), int(round((e - y0) / RES)):int(round((f - y0) / RES))] = True

# Espacio libre conectado con la salida A: lo que el robot puede ver
etiquetas, _ = label(~ocupado)
i = c['point_names'].index('A')
libre = etiquetas == etiquetas[int((c['point_x'][i] - x0) / RES), int((c['point_y'][i] - y0) / RES)]
borde = ocupado & binary_dilation(libre, structure=np.ones((3, 3)))

d = distance_transform_edt(~borde) * RES
g = gaussian_filter(d, SUAVIZADO / RES)
gx, gy = np.gradient(g, RES)
mapa = np.stack([d, gx, gy]).astype(np.float32)[:, ::FACTOR, ::FACTOR]

np.save('maps/distancias_2cm.npy', mapa)
with open('maps/distancias_2cm.yaml', 'w') as f:
    yaml.safe_dump({'resolucion': RES * FACTOR, 'origen_x': float(x0), 'origen_y': float(y0)}, f)
print(f'maps/distancias_2cm.npy: {mapa.shape}, {mapa.nbytes / 1e6:.1f} MB; libre {100 * libre.mean():.0f} % del plano')
