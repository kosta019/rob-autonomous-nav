# Proyecto ROB · Autonomous Navigation (TurtleBot3 Burger, ROS 2 Jazzy)

Este fichero da el contexto permanente del proyecto. La tarea concreta de ahora está en `TAREA_FASE1.md`.

## Cómo trabajar con Marc

- Marc estudia Ingeniería Informática (FIB-UPC), asignatura Robótica (ROB). Hace el proyecto en equipo y **quiere aprender**, no solo que funcione.
- **Antes de escribir o modificar código:** explica el plan y la lógica. Di por qué lo haces así y qué problemas evitas.
- **Antes de ejecutar un comando o una prueba:** di qué esperas que ocurra.
- **Estilo de código:**
  - Breve y sencillo, que lo entienda un principiante.
  - Sin estructuras elaboradas: nada de jerarquías de clases, patrones ni abstracciones que no hagan falta.
  - Cercano a la teoría de clase.
  - Que se vea que sigue la estructura de la plantilla `ref/go_stop.py`: una clase `Node`, publicadores y suscriptores en `__init__`, callbacks y `main()`.
  - Comentarios en español.
- **Si un dato no está aquí ni en `docs/enunciado_v2.pdf`, no lo inventes.** Pregunta, o déjalo como parámetro con un `# TODO` visible. Los datos marcados como *estimado* o *pendiente* lo son de verdad.
- **Pruebas que valen para el informe: solo Gazebo o el robot real.** Al profesor no le gustan los simuladores propios, así que **no construyas un simulador del robot**. Los tests offline del planificador sobre el mapa sí valen, porque comprueban un algoritmo, no simulan el robot.
- **El PC del laboratorio pierde el entorno al apagarse.** Todo debe poder reconstruirse con `git clone` + `colcon build`, sin `pip install`.
  - En los nodos ROS usa solo: `rclpy`, mensajes estándar, `numpy`, `yaml` (PyYAML) y la librería estándar. Comprueba con `python3 -c "import numpy, yaml"`.
  - `scipy` y `Pillow` solo en scripts offline que se ejecutan en el portátil, nunca en los nodos.

## Entorno ROS 2

- ROS 2 Jazzy, paquete `ament_python`. TurtleBot3 Burger (`export TURTLEBOT3_MODEL=burger`), real y en Gazebo.
- **Convenciones de la plantilla `ref/go_stop.py` (obligatorias):**
  - `/cmd_vel` es `geometry_msgs/msg/TwistStamped` (no `Twist`), con QoS RELIABLE y depth 10. Rellena `header.stamp`. Usa `self.get_clock().now().to_msg()` para que funcione `use_sim_time` en Gazebo.
  - `/scan` es `sensor_msgs/msg/LaserScan`, con QoS BEST_EFFORT y depth 10.
  - `/odom` es `nav_msgs/msg/Odometry`. Suscríbete con BEST_EFFORT: es compatible tanto con publicadores RELIABLE como BEST_EFFORT.
  - Al terminar (en el `finally`), publica una velocidad cero.
- **Fallos de la plantilla que NO hay que copiar:**
  - `if __name__ == '__main__': main()` está indentado dentro de `main()`, así que nunca se ejecuta. Con `ros2 run` y el `entry_point` funciona igual, pero hay que dejarlo bien.
  - `msg.ranges[0:15] + msg.ranges[345:359]` supone 360 muestras y se salta la 359. Calcula el ángulo de cada lectura como `angle_min + i * angle_increment`, normalízalo a (−π, π] y elige por ángulo.
- **Lecturas del LiDAR:** `0.0`, `inf`, `nan` o valores fuera de `[range_min, range_max]` significan "sin dato". Esto incluye los objetos más cerca que `range_min`, que no se ven: por eso la distancia de parada tiene que ser holgada.
- **Yaw a partir del cuaternión,** sin `tf_transformations` (en Jazzy da problemas):
  `yaw = atan2(2·(w·z + x·y), 1 − 2·(y² + z²))`.
- **Normaliza siempre los ángulos** con `atan2(sin(a), cos(a))`.
- **Datos del Burger** (aproximados; si necesitas precisión, compruébalos en el URDF de `turtlebot3_description`):
  - Huella de 138 × 178 mm; radio circunscrito ≈ 0.105 m.
  - Velocidad lineal máxima 0.22 m/s; angular máxima 2.84 rad/s.
  - LiDAR LDS-01 (0.12–3.5 m) o LDS-02 (0.16–8 m), según la unidad. Está casi en el centro, unos 3 cm por detrás del eje de las ruedas (frame `base_scan`).
- **En Gazebo:**
  - Lanzar con `ros2 launch turtlebot3_gazebo empty_world.launch.py`.
  - Los nodos, con `-p use_sim_time:=true`.
  - No hay mundo de Gazebo del laboratorio (ver el paso opcional en `TAREA_FASE1.md`).

## El proyecto (resumen de `docs/enunciado_v2.pdf`, versión del 26/10/2026)

**Fases** (el robot las hace sin ayuda externa):
1. **Fase I.** Desde una pose inicial asignada (xᵢ, yᵢ, θᵢ) en la zona 1, llegar a una pose final asignada (x_f, y_f, θ_f) en la zona 2, **pasando por E, Door y Q**. Puede haber obstáculos estáticos o dinámicos que no están en el mapa, así que hay que detectarlos y esquivarlos en tiempo real.
2. **Fase II.** Explorar el pasillo exterior (≤ 12 × 12 m), localizar una estación de carga (cuadrado de 40 × 40 cm sobre 4 pilares de ~5 cm de diámetro) y volver a P Base.
3. **Fase III.** Desde P Base, aparcar exactamente en el centro de la estación.

Durante las fases I y II el robot construye su propio mapa con SLAM, que es un entregable (`.yaml` + `.pgm`).

**Evaluación** (demo en el laboratorio con el robot real):
- Primero, completar la misión.
- Después:
  - seguridad, con **cero choques** (cualquier choque penaliza mucho);
  - distancia recorrida y tiempo;
  - precisión en los waypoints, la pose final y la estación;
  - detección de obstáculos y pilares;
  - calidad del código, metodología y documentación.

**Entregables:**
- El paquete ROS 2.
- El informe, con:
  - un diagrama de nodos y topics;
  - la lógica de las fases I y II;
  - la estrategia de evitación (cómo el LiDAR modifica la ruta prevista).
- El mapa generado por SLAM.
- El vídeo.
- La declaración de uso de IA generativa.
- El **Mission Log** (CSV/TXT), con:
  - `timestamp`;
  - `current_phase` (I, II o III);
  - `robot_x`, `robot_y`, `robot_yaw`;
  - las posiciones de los obstáculos detectados en el pasillo y la posición de la estación (sus pilares).

## Sistema de coordenadas del mundo (el del enunciado)

- **Origen:** la esquina superior izquierda de la sala S203 en la figura. X = 0 es la cara de la pared del fondo e Y = 0 la cara de la pared izquierda. Unidades en metros.
- **Ejes:** +X apunta hacia la sala S202 ("abajo" en la figura) e +Y hacia el pasillo exterior ("derecha").
- **Es dextrógiro, con z hacia arriba.** θ = 0 es mirar hacia +X, y θ positivo es antihorario visto desde arriba (de +X hacia +Y). Es un sistema 2D estándar, así que las fórmulas de transformación de la teoría valen tal cual.

## Puntos del enunciado (v2)

| Punto | (X, Y) | Punto | (X, Y) |
|---|---|---|---|
| A | (0.700, 0.920) | P Base | (3.340, 15.700) |
| B | (3.102, 1.722) | Q | (8.970, 14.497) |
| C | (2.078, 2.517) | R | (7.500, 16.170) |
| D | (5.285, 2.517) | S | (3.540, 14.497) |
| E | (5.082, 6.122) | T | (2.136, 15.300) |
| F | (5.679, 8.528) | G | (5.482, 10.930) |
| Door | (6.490, 11.682) | | |

Notas sobre la tabla:
- La tabla del PDF repite una fila "Punt S (3.675, 14.190)". Es un resto de la versión anterior; la buena es (3.540, 14.497).
- En D aparece "2,517" con coma; el valor es 2.517.
- D queda fuera de la zona 1 dibujada a propósito: es el paso entre las mesas y los bancos.
- F y G no son obligatorios. Door está unos 10 cm pasado el marco de la puerta, en el lado del vestíbulo.

## Mapa

- **Fichero `maps/mapa_1cm.png`:** 1782 × 1065 px, a 1 cm/px. La escala la hemos deducido porque coincide con las cotas de la figura: 12.65, 6.37, 3.94, 3.14 y 1.66 m. Los píxeles negros (< 128) están ocupados.
- **No hay `.yaml` oficial.** El enunciado avisa de que la celda (0, 0) de los mapas no coincide con el origen del mundo.
- **Georreferencia medida desde las paredes:**
  - De píxel a mundo: `X = (fila + 0.5 − 71) / 100`, `Y = (col + 0.5 − 2) / 100`.
  - De mundo a píxel: `fila = floor(100·X + 71)`, `col = floor(100·Y + 2)`.
  - La esquina superior izquierda de la imagen queda en (X −0.705, Y −0.015).
- **Hay una mota negra suelta de 1 px en (fila 502, col 1217).** Bórrala antes de inflar los obstáculos, o se convierte en un obstáculo de 50 cm.
- **Geometría medida en el mapa, útil para comprobar tu georreferencia:**
  - Pared entre la sala y el vestíbulo: X 6.34–6.39. El hueco de la puerta va de Y 11.27 a 12.02 (0.75 m de ancho; la figura dice 0.80, **sin medir en el laboratorio**).
  - Frente de los bancos junto a S202: X = 5.86, para Y entre 0.96 y 7.30. A partir de Y = 7.30 la pared de la sala está en X ≈ 6.35.
  - Saliente de la pared izquierda: para X ≥ 3.93, la sala empieza en Y = 0.96.
  - Pared derecha de la sala: Y = 12.65. Bloque junto a la puerta: Y 12.37–13.78, X 5.18–6.84.
  - Salida del vestíbulo al pasillo: X entre 6.84 y 8.48, a la altura de Y = 13.78.
  - Pasillo exterior: Y 13.78–16.90, X −0.55–9.53. Entre X 1.32 y 5.18 es más ancho: empieza en Y = 13.00.
- **Zonas prohibidas que NO están en el mapa y que el LiDAR no ve (o ve mal).** Van en la configuración como rectángulos, nunca en el código:

  | Zona | Rectángulo | Estado |
  |---|---|---|
  | Suelo irregular del vestíbulo | X 8.05–8.72, Y 11.79–12.95 | **Estimado** desde la figura |
  | Suelo irregular del pasillo | X −0.55–4.59, Y 15.85–16.90 | **Estimado.** P Base queda a solo 0.15 m |
  | Mesas entre B y C (zona 1), no se pueden sortear | — | **Sin medir (pendiente).** El LiDAR solo ve las patas, así que tienen que ir en el mapa |

## Estaciones de carga (medidas en el laboratorio el 06/11)

Hay **dos**: la buena es la del enunciado (cuadrada, 4 pilares); la otra, triangular, hay que descartarla.

| | Rectangular, en R (7.500, 16.170) | Triangular, en T (2.136, 15.300) |
|---|---|---|
| Patas | 4 cilindros de Ø 5 cm | 3 cilindros de Ø 5 cm |
| Lado, de extremo a extremo de las patas | 49 cm → **44 cm entre centros** (cuadrado; diagonal 62.2) | 49 cm → **44 cm entre centros** (equilátero) |
| Altura libre bajo los listones | 25 cm | 25 cm |
| Centro | sobre la cruz de R | sobre la cruz de T |

- 49 cm medido de extremo a extremo (caras exteriores): 44 entre centros, 39 de hueco libre. Orientación de cada
  estación respecto a las paredes: **sin medir**.
- El LiDAR (a ~18 cm) ve solo las patas: los listones empiezan a 25 cm. El Burger (~19 cm de alto, 17.8 de ancho) cabe
  debajo, con ~10.6 cm por lado entre patas (39 cm de hueco): la fase III es entrar y parar sobre la cruz.
- `range_max` del /scan: 3.5 m. Con ~1° entre rayos, una pata da 2-3 puntos a ≤ 1.2 m; a 2.2 m (desde Q), 0-1.
- Suelo irregular del pasillo (06/11): rectángulo de **cinta negra** en el suelo junto a la pared del fondo del pasillo.
  Medido: borde a 0.82 m de la cruz de P Base (Y 16.52), 4.85 m de largo (supuesto desde X −0.55: hasta X 4.30).
  P Base queda con 0.83 m de holgura. Fotos en `test_real/datos_20261106/foto_suelo_irregular_*.jpg`.

### Pasillo: las coordenadas del enunciado NO valen (medido con cinta el 06/11, desde la cruz de Q)

- **Cruz de R real: (7.17, 16.51)**, no (7.50, 16.17): de Q a R hay 1.80 m en X y 2.01 m en Y (cinta: R a la pared del
  fondo 2.40, Q a esa pared 0.60; R a la pared del extintor 0.64, Q a ella 2.65). El detector en el bag de aproximación
  la pone en (7.19, 16.48): coincide a ~3 cm. La odometría de Q a R, bien (~3 cm).
- **Pared del extintor (larga, Y alto del pasillo): Y ≈ 17.15** (mapa 16.90). Pared del fondo (delante de Q): X 9.57.
- R está **paralela a las paredes** y centrada en su cruz; T, centrada en su cruz, girada (sin medir).
- **P Base sí coincide**: a la pared del extintor 1.45 m (Y 15.70; enunciado 15.70) y a la pared del fondo 6.20 m
  (X 3.37; enunciado 3.34). No es todo el pasillo: es **la cruz de R** la que está desplazada. T sin medir.
  Los 15-20 cm de error al llegar a P Base (ensayo de las 16:37) son, entonces, de la pose: hace falta localizarse en el
  pasillo (SLAM). El detector no debe fiarse de las cruces (las estaciones se pueden mover).
- Suelo irregular: de Y 16.52 (0.82 m de P Base) a la pared del extintor (17.15): 0.63 m de ancho.

## Decisiones de diseño ya tomadas (no las reabras sin un motivo nuevo)

- **Arquitectura híbrida** (teoría, diapositiva 9): un plan global sobre el mapa más una capa reactiva con el LiDAR que lo corrige.
- **Planificador: Distance Transform con costes**, es decir, un Dijkstra lanzado desde el objetivo sobre un mapa de costes (diapositivas 16–18). Descartamos, después de comparar sobre este mapa:
  - **Bug2:** no usa el mapa, hace rodeos hasta el doble de largos y no ve el suelo irregular.
  - **Voronoi:** da rutas largas por el centro de las salas.
  - **PRM:** es aleatorio y no óptimo.
  - **D\* incremental:** con unas 75 000 celdas de 5 cm, replanificar desde cero tarda menos de un segundo. En el informe se menciona como alternativa estudiada.
- **Inflado de obstáculos de 0.25 m, nunca más de 0.30 m:** con 0.33 m la puerta se cierra en la rejilla de 5 cm.
- **Navegación propia, sin Nav2** (pendiente de confirmar con el profesor). El SLAM (slam_toolbox) entra más adelante y también está pendiente de confirmar.
- **Ninguna coordenada en el código.** Puntos, poses, zonas y parámetros van en `config/*.yaml`. El profesor ya cambió las coordenadas una vez.
- **Robot real (BOT 11), pruebas del 05/11/2026:**
  - **Rumbo de `/odom`** (`use_imu_yaw: false` en `real.yaml`): se integra en el robot, así que un corte de WiFi no pierde giro. El driver lo saca del IMU de la OpenCR (`odometry.use_imu: true`), no de las ruedas: no se toca el robot durante la ruta.
  - **Canaleta de cables** en la pared del fondo y en la izquierda: sobresale 4 cm y el LiDAR la ve a ella, no al yeso. Está en `maps/paredes.yaml` (caras a 0.04). El sistema de coordenadas sigue siendo el yeso.
  - **Rumbo inicial con la pared izquierda** (`pose_heading_axis: "Y"`), `lidar_yaw_offset: 0`. La pared del fondo y la izquierda difieren 1.76° (sistemático, ±0.06°); el rumbo de la pared izquierda coincide con el real de la A→B medida con cinta (±0.1°). El 0.019 anterior solo compensaba la mezcla de las dos paredes en A.
  - **Relocalización en la puerta** (`reloc_*` en `fase1.yaml`, `ajuste_puerta` en `fase1_node`): en el último waypoint antes de Door, parado. Rumbo y X de la canaleta de la pared de la puerta (la recta con más puntos alineados y su tramo continuo más largo); Y del lateral del marco del borde cercano. Referencias CALIBRADAS en vivo con el robot en pose de cinta (X 6.344, Y 11.245). Límites por eje (|dx| ≤ 0.30 rechaza; |dy| ≤ 0.45 y borde creíble, si no corrección parcial) y comprobación repitiendo el ajuste (`RELOC_INCONSISTENTE`). La hoja de la puerta (bisagras en el lado lejano) no se usa.
  - **En el robot real, `fase1_node` lo lanza Marc en su terminal** (nunca en segundo plano: un `&` en un script no interactivo ignora el Ctrl+C).
  - **Antes de cada salida:** 10 s de deriva de `/odom` (< 0.2°). La pose de salida (rumbo y posición con las paredes) y el giróscopo los comprueba el nodo al arrancar, parado (`start_pose_check`, estado `POSE_INICIAL`, 07/11); antes, `comprobar_pose --escribir` a mano. La OpenCR calibra el giróscopo al encenderse: si se enciende moviendo el robot, `/imu` marca un giro falso (05/11: 0.63 rad/s). Se arregla apagando y encendiendo con el robot quieto.
  - **Ventana del LiDAR limpia:** sucia da ecos a 12-17 cm que bloqueaban la ruta. `obstacle_min_range: 0.20` los ignora en la memoria de obstáculos (no en la parada de emergencia) y `comprobar_pose` avisa si los ve.
