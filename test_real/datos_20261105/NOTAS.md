# Pruebas en el robot real (BOT 11), 05/11/2026

Horas locales (CET). Bags en `bags/` (comprimidos con zstd, ver abajo). Coordenadas en el sistema del enunciado.

## Montaje
- Había un Gazebo abierto en el mismo `ROS_DOMAIN_ID` (11): `/scan`, `/odom` e `/imu` con 2 publicadores y `/clock`.
  Cerrado antes de las pruebas. Comprobar siempre con `ros2 topic info -v` que hay 1 publicador.
- Frecuencias limpias: `/scan` 5.0 Hz, `/odom` 20 Hz, `/imu` 20 Hz.
- Al empezar el robot giraba solo a 0.021 rad/s sin nadie en `/cmd_vel`: la placa repetía el último comando.
  Se para publicando una velocidad cero.

## prueba_rumbo (12:48, 4 vueltas a izquierdas a 0.5 rad/s) — `prueba_rumbo.csv`
| Fuente | Giro | Error |
|---|---|---|
| Paredes (comprobar_pose antes/después: 0.97° -> 3.41°) | 1442.4° | — |
| `/odom` | 1441.1° | -1.3° |
| `/imu` integrado en el PC | 1440.5° | -1.9° |

El rumbo de `/odom` también sale del IMU (driver: `odometry.use_imu: true`, `turtlebot3_node/src/odometry.cpp`).
Al coger el robot con la mano (12:56:52-12:57:04) el rumbo de `/odom` saltó 131° con el robot girando 1° de verdad.
Decisión: `use_imu_yaw: false` (se integra en el robot, aguanta cortes de WiFi).

## Recta con teleop (13:01:01.5-13:01:30.0) — `odom_recta_teleop.csv`
Según `/odom`: 2.154 m, +3.4 cm lateral, +1.3°. Sin medida real (el robot se movió antes de medir).

## A->B nº 1 (13:12:32-13:13:07) — `mission_log_AB_1.csv`, `odom_AB_1.csv`, `foto_B_AB_1.jpg`
- Salida (comprobar_pose, sin canaleta): (0.667, 0.869), 1.38°. Llegada con cinta: (3.105, 1.68).
- Canaleta de cables de 4 cm en la pared del fondo y en la izquierda (8-25 cm del suelo): el LiDAR la ve a ella.
  Salida corregida: ~(0.707, 0.910).
- Dirección real 17.81° frente a 19.30° creída: -1.49°. Error real en B 4.2 cm (sobre todo en Y).
- Eventos: 56 OBSTACULO (ninguno paró el robot), sin BLOQUEO ni IMU_HUECO.

## Cambios tras la nº 1
- `maps/paredes.yaml`: caras de la pared del fondo y de la izquierda a 0.04 (canaleta).
- `comprobar_pose`: resta `lidar_yaw_offset` (0.026 primero).

## A->B nº 2, verificación (13:51:52.9-13:52:27) — `mission_log_AB_2_verificacion.csv`, `odom_AB_2_verificacion.csv`
- Deriva previa en A: -0.014° en 10 s (`odom_deriva_A.csv`).
- Salida: comprobar_pose (0.707, 0.904, 0.07°), cinta (0.71, 0.92). Llegada con cinta: (3.08, 1.75).
- Dirección real 19.30° frente a 18.84° creída: +0.46° (+2.0 cm en 2.55 m). Error real en B 3.6 cm (dX -2.2, dY +2.8).
  El robot se para al entrar en tol_mandatory (0.03): llega ~2.7 cm corto.
- Escala de las ruedas: real 2.511 m / creída 2.504 m = +0.3 % (despreciable).
- Eventos: sin BLOQUEO ni IMU_HUECO. Los OBSTACULO son casi todos del rato en FIN, con gente midiendo al lado.

## Cambios tras la nº 2
- `lidar_yaw_offset: 0.019` (pesa más la nº 2, con cinta en los dos extremos) y `tol_mandatory: 0.02`.

## Rumbo de cada pared por separado en A (`rumbo_por_pared.py`, scans del bag 13:51:00-13:51:50)
| Pared | Puntos | Rumbo del LiDAR |
|---|---|---|
| Fondo (cara X = 0.04) | 111 | 2.57° |
| Izquierda (cara Y = 0.04) | 87 | 0.80° |

No coinciden (1.8°): el desfase depende de las paredes, no solo del montaje del LiDAR.
Hipótesis: la esquina no es de 90°. PENDIENTE: triángulo 3-4-5 en la esquina (0.60 m por el fondo,
0.80 m por la izquierda, diagonal = 1.000 m si es recta) y comprobar desde otra salida (C o D).

## Ficheros
- `log_odom.py`: solo escucha `/odom` y escribe hora, x, y, yaw, v, w (para las horas de cada tramo).
- `rumbo_por_pared.py`: offline, lee `/scan` de un bag y saca el rumbo de cada pared con las funciones de comprobar_pose.

## Bags
- `bag_fase1_20261105_124803.tar.zst` (12:48-13:30 aprox.): prueba_rumbo, recta con teleop y A->B nº 1.
- `bag_fase1_20261105_133749.tar.zst` (13:37-14:09): A->B nº 2 (verificación).
- Descomprimir: `zstd -dc FICHERO.tar.zst | tar -xf -` y luego `ros2 bag info fase1_...`.
- Al final del log de la nº 2 (ya en FIN, al mover el robot para medir) el rumbo de /odom salta de 0.326 a -0.02 rad
  con x, y quietos: otra vez el IMU al manipular el robot. No afecta a la prueba.

## Ruta completa nº 1 (14:56:31-14:59:11) — `mission_log_ruta_completa_1.csv`, bag `fase1_20261105_145612`
- Salida en A (comprobar_pose con canaleta y lidar_yaw_offset 0.019): (0.710, 0.908, -0.08°). Deriva previa -0.006° en 10 s.
- B (14:57:03, 6.9 cm) y D (14:57:33, 6.9 cm) bien. A las 14:58:07, en (5.12, 5.11), a 1 m de E, una persona cruza:
  BLOQUEO y después SIN_CAMINO cada 5 s (E obligatorio tapado) hasta que se paró a mano a las 14:59:11.
- Causa (`serie_bloqueo.py`, `rayos_E.py` sobre el bag): de 14:58:13 a 14:58:30 la ruta estaba libre en el scan,
  pero el LDS da 0.0 (no inf) cuando el rayo no choca con nada hasta 3.5 m, y la regla de borrado solo aceptaba
  inf o una distancia mayor. Solo quedaba el olvido por tiempo (30 s), y la gente que pasaba lo reiniciaba.
  En Gazebo no se veía porque allí un rayo sin choque da inf.
- Arreglo: `rayos_libres` (0.0 = libre si el obstáculo está a > 0.5 m), 3 escaneos libres seguidos para borrar
  (un 0.0 suelto de ropa negra no basta) y obstacle_memory_time 8 s en real.yaml. Test: `test/test_borrado.py`.

## Prueba 3b en el robot real (pendiente)
Ruta completa desde A. Alguien se pone delante del robot a ~1 m unos 4 s y se aparta a más de 1 m.
Esperado: BLOQUEO, y RUTA_LIBRE a 1-2 s de apartarse, sin REPLANIFICACION ni SIN_CAMINO.
Quien acompañe al robot, detrás o a un lado a más de 0.5 m (a < 0.35 m de los próximos 1.6 m de ruta lo para).
