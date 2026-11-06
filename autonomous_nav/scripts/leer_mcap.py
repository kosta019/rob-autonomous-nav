# Lector mínimo de bags MCAP (los de `ros2 bag record`) SIN ROS: para analizar los bags en el portátil.
# Solo la librería estándar y numpy. Decodifica /scan (LaserScan), /odom (Odometry) y /tf (TFMessage) en CDR.
# Script OFFLINE: no lo usa ningún nodo (en el PC del laboratorio, con ROS, vale rosbag2_py).
# Uso como módulo:
#   from leer_mcap import leer
#   for topic, t, msg in leer('bag.mcap', ['/scan', '/odom']): ...   (t en segundos, msg es un dict)
import struct
import sys

import numpy as np


class Cdr:
    # Lee un mensaje serializado en CDR (little endian). La alineación cuenta desde después de los 4 bytes de cabecera.
    def __init__(self, datos):
        self.d = datos
        self.p = 4

    def alinear(self, n):
        self.p += (-(self.p - 4)) % n

    def num(self, fmt, n):
        self.alinear(n)
        v = struct.unpack_from('<' + fmt, self.d, self.p)[0]
        self.p += n
        return v

    def f32(self):
        return self.num('f', 4)

    def f64(self):
        return self.num('d', 8)

    def u32(self):
        return self.num('I', 4)

    def texto(self):
        n = self.u32()
        s = self.d[self.p:self.p + n - 1].decode()
        self.p += n
        return s

    def cabecera(self):
        seg = self.num('i', 4)
        nseg = self.u32()
        return {'stamp': seg + nseg * 1e-9, 'frame_id': self.texto()}

    def vector_f32(self):
        n = self.u32()
        self.alinear(4)
        v = np.frombuffer(self.d, '<f4', n, self.p).astype(float)
        self.p += 4 * n
        return v


def scan(d):
    c = Cdr(d)
    m = {'header': c.cabecera()}
    for k in ('angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max'):
        m[k] = c.f32()
    m['ranges'] = c.vector_f32()
    return m


def pose(c):
    p = [c.f64() for _ in range(3)]
    q = [c.f64() for _ in range(4)]   # x, y, z, w
    return p, q


def odom(d):
    c = Cdr(d)
    m = {'header': c.cabecera(), 'child_frame_id': c.texto()}
    m['position'], m['orientation'] = pose(c)
    return m


def tf(d):
    c = Cdr(d)
    lista = []
    for _ in range(c.u32()):
        h = c.cabecera()
        hijo = c.texto()
        p, q = pose(c)
        lista.append({'stamp': h['stamp'], 'parent': h['frame_id'], 'child': hijo, 'translation': p, 'rotation': q})
    return lista


DECODIFICAR = {'sensor_msgs/msg/LaserScan': scan, 'nav_msgs/msg/Odometry': odom, 'tf2_msgs/msg/TFMessage': tf}


def descomprimir(datos, metodo):
    if metodo == '':
        return datos
    if metodo == 'zstd':
        try:
            from compression import zstd        # Python >= 3.14
        except ImportError:
            import zstandard as zstd
        return zstd.decompress(datos)
    raise ValueError('compresión no soportada: ' + metodo)


def registros(datos):
    # Recorre los registros MCAP: opcode (1 byte), longitud (8 bytes), contenido.
    p = 0
    while p + 9 <= len(datos):
        op = datos[p]
        n = struct.unpack_from('<Q', datos, p + 1)[0]
        yield op, datos[p + 9:p + 9 + n]
        p += 9 + n


def leer(fichero, topics=None):
    with open(fichero, 'rb') as f:
        datos = f.read()
    assert datos[:8] == b'\x89MCAP0\r\n', 'no es un MCAP'
    esquemas, canales = {}, {}
    pendientes = [datos[8:]]
    while pendientes:
        for op, r in registros(pendientes.pop(0)):
            if op == 0x03:      # Schema: id, nombre (tipo del mensaje)
                sid, n = struct.unpack_from('<HI', r, 0)
                esquemas[sid] = r[6:6 + n].decode()
            elif op == 0x04:    # Channel: id, schema_id, topic
                cid, sid, n = struct.unpack_from('<HHI', r, 0)
                canales[cid] = (r[8:8 + n].decode(), esquemas.get(sid, ''))
            elif op == 0x05:    # Message: channel_id, sequence, log_time, publish_time, datos
                cid, _, t, _ = struct.unpack_from('<HIQQ', r, 0)
                topic, tipo = canales[cid]
                if (topics is None or topic in topics) and tipo in DECODIFICAR:
                    yield topic, t * 1e-9, DECODIFICAR[tipo](r[22:])
            elif op == 0x06:    # Chunk: tiempos, tamaño, crc, compresión, registros
                n = struct.unpack_from('<I', r, 28)[0]
                metodo = r[32:32 + n].decode()
                largo = struct.unpack_from('<Q', r, 32 + n)[0]
                bloque = r[40 + n:40 + n + largo]
                # Se procesa ya (antes que lo que viene detrás) para mantener el orden de los mensajes
                for item in leer_bloque(descomprimir(bloque, metodo), esquemas, canales, topics):
                    yield item


def leer_bloque(datos, esquemas, canales, topics):
    for op, r in registros(datos):
        if op == 0x03:
            sid, n = struct.unpack_from('<HI', r, 0)
            esquemas[sid] = r[6:6 + n].decode()
        elif op == 0x04:
            cid, sid, n = struct.unpack_from('<HHI', r, 0)
            canales[cid] = (r[8:8 + n].decode(), esquemas.get(sid, ''))
        elif op == 0x05:
            cid, _, t, _ = struct.unpack_from('<HIQQ', r, 0)
            topic, tipo = canales[cid]
            if (topics is None or topic in topics) and tipo in DECODIFICAR:
                yield topic, t * 1e-9, DECODIFICAR[tipo](r[22:])


if __name__ == '__main__':
    # Resumen: mensajes por topic
    cuenta = {}
    for topic, t, m in leer(sys.argv[1]):
        cuenta[topic] = cuenta.get(topic, 0) + 1
    print(cuenta)
