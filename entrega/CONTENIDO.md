# Qué hay en el zip de la entrega

Lo genera `entrega/crear_zip.sh` desde el último commit, y antes de comprimir compila el paquete con lo que va dentro.
Esta guía es para nosotros y no va en el zip.

## Entregables (los dejamos a mano en `entrega/` antes de generar el zip)

| Fichero | Qué es |
|---|---|
| `memoria.pdf` | La memoria técnica (el Google Docs exportado a PDF) |
| `declaracion_IA.pdf` | Declaración de uso de IA generativa |
| `mapa/` | El mapa del SLAM de las fases I y II: un `.pgm` (la imagen) y un `.yaml` (resolución y origen), de `map_saver_cli` |
| `mission_log.csv` | El Mission Log de la ejecución del vídeo (renombrado desde `mission_log_<fecha>.csv`) |
| `video.mp4` | El vídeo de la misión completa |

## El paquete ROS 2: `autonomous_nav/`

### Raíz del paquete

| Fichero | Qué es |
|---|---|
| `README.md` | Instrucciones para el profesor: requisitos, compilar, lanzar, Mission Log y regenerar la ruta |
| `LICENSE` | Licencia MIT |
| `package.xml` | Manifiesto ROS 2: nombre, versión, licencia y dependencias (rclpy, mensajes, numpy, yaml, slam_toolbox) |
| `setup.py` | Instalación (ament_python): qué ficheros de `config/`, `maps/` y `launch/` se copian a `share/` y los ejecutables (`fase1_node`, `comprobar_pose`, `scan_slam`) |
| `setup.cfg` | Dónde instala colcon los ejecutables (lo pide `ros2 run`) |
| `resource/autonomous_nav` | Fichero vacío que registra el paquete en el índice de ament (obligatorio en ament_python) |

### Código (`autonomous_nav/autonomous_nav/`)

| Fichero | Qué es |
|---|---|
| `__init__.py` | Vacío: convierte la carpeta en un módulo de Python |
| `fase1_node.py` | **El nodo de la misión** (fases I, II y III). Máquina de estados (POSE_INICIAL, INICIO, GIRAR, AVANZAR, ESPERAR, RELOC, ORIENTAR, PAUSA, MEDIR3, ENTRAR, VERIFICAR3, FIN); pose por odometría corregida con el mapa, la pared del pasillo y las relocalizaciones de la puerta y Q; memoria de obstáculos, bloqueo y replanificación; parada de emergencia; detección de las estaciones; Mission Log |
| `planificador.py` | Planificador global: rejilla de holgura, mapa de costes con inflado, distance transform (Dijkstra desde el objetivo) y simplificación del camino. Lo usan el nodo (replanificar) y `planificar_fase1.py` |
| `pilares.py` | Detector de las patas de las estaciones: segmenta el scan, agrupa detecciones de varios scans y busca el cuadrado de 44 cm (o el triángulo de la descartada); centro de la cuadrada con 3 o 4 patas para la fase III |
| `comprobar_pose.py` | Estima la pose con el LiDAR contra las paredes del mapa. Lo usa el nodo al arrancar (POSE_INICIAL) y también se puede ejecutar a mano para comprobar la pose y el giróscopo antes de salir |
| `scan_slam.py` | Nodo que reenvía `/scan` a `/scan_slam` con el sello de tiempo −0.1 s (el retraso del LiDAR), para que el mapa del SLAM salga recto |

### Configuración (`autonomous_nav/config/`)

| Fichero | Qué es |
|---|---|
| `fase1.yaml` | Todos los parámetros del nodo con su valor por defecto: puntos del enunciado, poses, waypoints, tolerancias, velocidades, parada de emergencia, relocalizaciones, correcciones de pose, obstáculos, fases II y III, zonas prohibidas |
| `fase1_ruta.yaml` | La ruta de la fase I calculada offline por `planificar_fase1.py` (waypoints entre A, B, D, E, Door y Q). Generado: no se edita a mano |
| `real.yaml` | Los valores que cambian en el robot real respecto a `fase1.yaml` (velocidades, rumbo de `/odom`, paredes medidas con cinta en Q y en el pasillo, corrección de pose activada) |
| `salida_B.yaml`, `salida_C.yaml`, `salida_D.yaml` | Para empezar desde B, C o D: pose inicial, waypoints que se saltan y plan desde la salida (por los paneles) |
| `slam.yaml` | Parámetros de slam_toolbox (los de por defecto, leyendo `/scan_slam`) |

### Launch (`autonomous_nav/launch/`)

| Fichero | Qué es |
|---|---|
| `slam.launch.py` | Lanza `scan_slam` y slam_toolbox con `slam.yaml`, en un solo comando |

### Mapas (`autonomous_nav/maps/`)

| Fichero | Qué es |
|---|---|
| `mapa_1cm.png` | El mapa oficial del enunciado a 1 cm/píxel (negro = ocupado). De aquí sale todo lo demás |
| `paredes.yaml` | Las paredes del mapa como rectángulos en coordenadas del mundo, con la canaleta de cables. Lo usa `comprobar_pose` (pose de salida) |
| `holgura_5cm.npy` + `.yaml` | Rejilla de 5 cm con la distancia de cada celda al obstáculo más cercano (con las zonas prohibidas). La usa el planificador del nodo y la detección de obstáculos nuevos. Generada por `planificar_fase1.py` |
| `distancias_2cm.npy` + `.yaml` | Distancia a la pared más cercana y su gradiente, cada 2 cm, para la corrección continua de la pose en la sala. Generada por `generar_mapa_distancias.py` |

### Scripts offline (`autonomous_nav/scripts/`, en el portátil; usan scipy y Pillow, no los nodos)

| Fichero | Qué es |
|---|---|
| `planificar_fase1.py` | Calcula la ruta de la fase I sobre el mapa y escribe `fase1_ruta.yaml`, `holgura_5cm.*` y una figura de la ruta. Hay que volver a ejecutarlo si cambian los puntos o las zonas |
| `generar_mapa_distancias.py` | Escribe `distancias_2cm.*` a partir de `paredes.yaml`. Solo si cambian las paredes |

## Lo que se queda fuera (está en el repo)

Los tests (`autonomous_nav/test/`), Gazebo (`launch/laboratorio.launch.py` y `worlds/`), `prueba_rumbo.py`, los
`config/test_*.yaml`, los scripts de análisis, `test_real/` (pruebas, logs, bags y fotos), `docs/` y `docker/`.
