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

## Ruta con ecos (15:11:46-15:12:40) — `mission_log_ruta_ecos.csv`, bag `fase1_20261105_150953`
- 16 BLOQUEO / 7 RUTA_LIBRE / 8 REPLANIFICACION en un minuto: un "obstáculo" siempre a 0.16-0.21 m detrás a la
  izquierda del robot, que se movía con él. En el bag: lecturas a 0.12-0.17 m a 142-149°, intensidad alta (~5000),
  en el 25 % de los scans con el robot quieto en A (`cortas_ventana.py`, `rayo146.py`). Sin cables a esa altura.
- Tras limpiar la ventana del LiDAR: 0 lecturas a < 0.25 m en 100 scans (`vivo_cortas.py`). Diagnóstico: ventana sucia.
- Los RUTA_LIBRE rápidos confirman el borrado con 3 scans, pero allí los rayos daban distancias finitas (pared del
  fondo), no 0.0.

## Giróscopo descalibrado (15:29)
- Con el robot quieto, `/imu` marcaba 0.63 rad/s en z (y 0.22 en y) y el rumbo de /odom daba vueltas; las paredes
  (comprobar_pose) decían que el robot no giraba. La OpenCR calibra el giróscopo al encenderse: se encendió moviéndolo.
- Arreglo: apagar y encender la OpenCR con el robot quieto en el suelo y no tocarlo ~10 s; relanzar el bringup.
  Después `/imu` = 0.0000 (`imu_media.py`) y deriva -0.001° en 10 s. La comprobación de deriva previa lo detectó.

## Ruta completa nº 2 (15:36:18-15:38:21) — `mission_log_ruta_completa_2.csv`, bag `fase1_20261105_153312`
- A -> B (6.9 cm) -> D (6.9 cm) bien. En D->E mucha gente cruzando: 9 BLOQUEO, 6 RUTA_LIBRE, 2 REPLANIFICACION
  (rodeo por X ~ 4.83, ~35 cm a la izquierda de la ruta original: es el "desvío" que se vio). Parado a mano en
  (4.84, 5.47), a 0.7 m de E. Sin choques, sin SIN_CAMINO, sin ecos del robot (0 de 610 scans).
- Arreglo del 0.0 confirmado en el pasillo (`rayos_borrado.py`): RUTA_LIBRE 1.5-2.2 s después del BLOQUEO con rayos
  [0.0, 0.0, 3.789], [2.752, 0.0, 0.0] y [0.0, 0.0, 0.0] hacia el obstáculo (con el código viejo no se borraban).
- La 3b limpia (una persona, se aparta antes de 3 s, sin replanificar) sigue PENDIENTE: había demasiada gente.
- Obstáculos repetidos en X ~ 4.70, Y 5.2-6.2 (también en la ruta 1): ¿extremo de mesa o sillas fuera del mapa? Revisar.

## Cambios tras la ruta nº 2
- `obstacle_min_range: 0.20` (fase1.yaml): más cerca no entra en la memoria de obstáculos ni bloquea la ruta.
  La parada de emergencia no lo usa.
- comprobar_pose: si más del 5 % de los scans tiene lecturas a menos de obstacle_min_range, avisa (limpia la
  ventana del LiDAR) y no escribe la pose.
- real.yaml: max_linear_speed 0.12 (antes 0.08). Siguiente paso 0.15 si la ruta sale bien.

## Rumbo de cada pared en A (sin robot, `rumbo_por_pared.py`, bag `fase1_20261105_153312`)
| Ventana | Fondo (cara X) | Izquierda (cara Y) | Diferencia |
|---|---|---|---|
| 15:33:47 | 2.908° | 1.161° | 1.747° |
| 15:34:30 | 2.906° | 1.196° | 1.710° |
| 15:35:30 | 2.911° | 1.144° | 1.767° |
| 13:51 | 2.571° | 0.796° | 1.775° |
Sistemático: las paredes no son perpendiculares (vistas a la altura de la canaleta). Decisión: rumbo solo con la
pared izquierda (`pose_heading_axis: "Y"`) y `lidar_yaw_offset: 0`.

## Ruta completa nº 3 a 0.12 m/s (15:48:56-15:53:30) — `mission_log_ruta_completa_3.csv`, bag `fase1_20261105_153312`
- Salida: /imu 0, deriva +0.003° en 10 s, pose (0.684, 0.907, 0.23°).
- Rodeo cerca de B; D 6.7 cm; antes de E, BLOQUEO por algo en (5.29, 5.91) y 3 replanificaciones; E 1.9 cm (según el
  robot); rodeo después de E (el replan se saltó E-Door_1).
- 15:51:31: BLOQUEO delante de la puerta por "algo" en (6.20, 11.50) y SIN_CAMINO cada 5 s. Parado.
- Lanzado por Claude en segundo plano: el Ctrl+C (SIGINT) se ignoraba (`&` en un script no interactivo). Se paró con
  SIGTERM y velocidad cero publicada a mano. Desde ahora el robot real lo lanza Marc en su terminal.
- Diagnóstico (`puerta_vivo.py`, comprobar_pose en vivo, `reloc_bag.py`):
  - Pose real por las paredes (5.832, 11.004) frente a la creída (5.696, 11.017): 13.6 cm de error en X tras ~17 m.
  - El LiDAR ve pared hasta Y ~ 11.48 en el lado de la puerta con Y menor; el mapa pone el hueco desde 11.27. Hueco libre
    ~0.52 m: Door (Y 11.68) queda a 0.18-0.20 m del borde, menos de lo que pide el planificador. PENDIENTE: medir.
  - La relocalización (como en el nodo) sobre 3 ventanas del bag: ACEPTADA, dx +0.136, dy -0.013, dth +0.38°.

## Cambios tras la ruta nº 3
- comprobar_pose: rumbo solo con la pared izquierda; aviso del giróscopo obligatorio (|w| medio de /imu > 0.01 rad/s
  o deriva del rumbo de /odom > 0.2° en la ventana: "reinicia la OpenCR" y no escribe).
- fase1_node: estado RELOC en E-Door_1 (eventos RELOC / RELOC_RECHAZADA <motivo>); los puntos de relocalización se
  conservan al replanificar.
- Tests: test_reloc.py (scans sintéticos con las caras del mapa) y casos del giróscopo. 29 tests.

## Puerta medida (desde la cruz de G) y relocalización con el hueco
- Medidas del usuario: a la pared de la puerta 0.91 m (mapa 0.86), al lado cercano del hueco 0.34 (mapa 0.34), al
  lejano 1.12 (mapa 1.10). La puerta mide 0.76 m: el mapa está bien y no se toca. Con la profundidad de la sala
  (5.91 frente a 5.86), la pared de la puerta está ~5 cm más allá: X = 6.39 para relocalizar (planificador: 6.34).
- Ruta 3: el error estaba también en la Y del robot (el marco 11.27 se veía en ~11.50). La "cara Y = 12.37" que usaba
  la relocalización anterior seguramente era otra superficie (¿la hoja de la puerta abierta?).
- Relocalización nueva (`ajuste_puerta` en fase1_node): rumbo y X de la recta de la pared de la puerta a los dos
  lados del hueco (x = 6.39); Y de los extremos del hueco (salto mayor de la línea de pared), con el ancho (0.76 ±
  0.06) como comprobación. Salvaguardas: pocos_puntos, residuo (> 2 cm), ancho, dxy (> 0.30 m), dth (> 3°).
- Validación con el bag (`reloc_puerta_bag.py`, robot parado 0.6 m antes de E-Door_1, 15:52-15:53, 5 ventanas):
  dx +0.183 (13.7 cm + los 5 cm de la referencia 6.39), dth +0.8° a +1.35°, rms ~1 cm, pero dy -0.33 y hueco
  0.84-0.86 m -> RECHAZADA ancho las 5 veces. La salvaguarda funciona; falta probar desde E-Door_1 (de frente).

## Relocalización: Y del borde cercano y corrección parcial; zona de la hoja de la puerta
- La hoja de la puerta está abierta del todo hacia la sala, perpendicular al hueco, con las bisagras (según la figura)
  en el borde lejano (Y ~ 12.03): tapa el tramo de pared del lado lejano (por eso el hueco salía de 0.84-0.86 m).
- `ajuste_puerta`: la recta (rumbo y X) solo con el tramo de pared CERCANO (hasta 11.27 + 0.40): la hoja, perpendicular
  y a ~0.4 m de E-Door_1, torcía la recta (test sintético: 4.8°). Y del borde cercano (11.27). Ancho del hueco: solo
  aviso. Niveles: total (dx, dy, dth), parcial (dx, dth; motivos sin_borde, dy) o nada (pocos_puntos, residuo, dxy, dth).
- Bag de la ruta 3 (0.6 m antes de E-Door_1): PARCIAL dy en 3 ventanas: dx +0.185, dth +0.33°, rms ~0.65 cm; borde
  cercano en 11.55 (creído) -> dy -0.28 no aplicado (|dxy| 0.34 > 0.30).
- Hoja en el bag (`hoja_bag.py`, pose corregida): recta en Y 12.08-12.13 (mediana 12.10), X 5.59-6.34.
  Zona `hoja_puerta`: X 5.58-6.34, Y 12.03-12.13. Ruta regenerada (en este PC): E-Door_1 pasa de (5.965, 11.655) a
  (5.915, 11.505); E->Door 6.05 m; total 18.55 m; holgura mínima 0.34 m. Referencia de test_planificador actualizada.

## Calibración de la relocalización en la puerta (en vivo, robot en pose de cinta)
- Datos del usuario: en la pared de la puerta hay canaleta que acaba justo antes del hueco; desde el eje del robot en
  (5.96, 11.66, 90°): 0.37 m a la canaleta, 0.43 m al yeso. Bisagras de la hoja en el lado lejano.
- Vista en vivo (`ventana_vivo.py`): canaleta en X 6.32-6.34 (Y 10.07-11.14), escalón de yeso X 6.35-6.42 (Y 11.14-11.23),
  lateral del marco X 6.46-6.54 en Y ~11.24, hoja en Y 12.01-12.02 (X 5.59-6.37). Hueco visto 0.77 m.
- Primer intento: la "pasada a 2 cm" no quitaba el escalón (las pasadas de 10 y 5 cm torcían la recta: dth +2°, X 6.37).
  Segundo (recta hasta 15 cm antes del marco): bien, pero sin ver el marco una persona en el hueco desviaba dx 3.6 cm, y
  en vivo a veces el tramo se partía (pocos_puntos). Definitivo: recta de partida = la de más puntos a < 2 cm entre las
  que pasan por dos puntos (la canaleta), y de ella el tramo continuo más largo; el marco, al final de ese tramo.
- Calibración (`puerta_vivo2.py`, 3 x 10 scans): X de la canaleta 6.343-6.344 (rumbo de referencia = la pared, que el bag
  de la ruta 3 da casi paralela a Y; la colocación a mano tenía ~1° de error) y lateral del marco en Y 11.245-11.246.
  -> reloc_wall_x 6.344, reloc_edge_y 11.245. reloc_min_points 20 (el tramo visible da 22-33 puntos).
- Prueba (a) sin mover: total, |dx| <= 0.1 cm, |dy| <= 0.2 cm, dth +1.0° (la colocación); al repetir desde la pose
  corregida, <= 1 mm / 0.03°.
- Prueba (b), robot movido 10 cm hacia E (cinta), pose creída la de calibración: total, dy -0.102 a -0.105, dx ~ -0.3 a
  -0.7 cm, dth +2.3° a +2.6° (al moverlo a mano giró). 1 de 6 rechazada por dth (3.06° > 3°).
  Con la pose verdadera de (b): 5/5 total, dx -0.7 cm, dy -0.5 cm, rms 0.4-0.5 cm.

## Ruta regenerada con hoja_puerta (Y 12.01-12.06) y mesa_pequena (X 5.51-5.90, Y 5.57-6.58)
- D -> E con un intermedio (5.015, 6.005); E -> Door con (5.115, 7.005) y (5.915, 11.455); total 18.54 m; holgura mínima
  0.33 m. Distancia mínima de la ruta a la mesa pequeña 0.411 m. Holgura en E 0.43 (era 0.78).
- La relocalización se activa en el último waypoint antes de Door: ahora E-Door_2 (5.915, 11.455), rumbo de llegada
  79.8°, a 0.429 m de la canaleta y 0.21 m por encima del marco. Puntos de pared esperados por geometría: 36 (la misma
  cuenta da 22 y 27 en las posiciones de calibración y (b), donde en vivo hubo 24-25 y 30-33).
- test_planificador: referencias actualizadas y justificadas (holgura en E, D->E con intermedio, E->Door con dos).
