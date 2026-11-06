# Handoff para el siguiente agente (06/11/2026, fin de la sesión de laboratorio)

Léelo entero antes de tocar nada. Después, en este orden: `CLAUDE.md` (reglas del proyecto, datos medidos, decisiones
cerradas), `README_DEMO.md` (cómo se opera el robot: es lo que usará el equipo en la demo, sin agente) y
`test_real/datos_20261106/NOTAS.md` (todas las pruebas del 06/11, con el porqué de cada parámetro).

## Contexto en dos líneas

TurtleBot3 Burger (BOT 11, `ROS_DOMAIN_ID=11`), ROS 2 Jazzy, navegación propia sin Nav2. Misión: fase I (de A/B/C/D a la
pose final pasando por E, Door y Q), fase II (explorar el pasillo, identificar la estación de carga y volver a P Base,
con mapa SLAM como entregable), fase III (aparcar en el centro de la estación). El usuario (Marc, FIB-UPC) quiere
**aprender**, habla español y trabaja en el laboratorio con un compañero; el PC del laboratorio está **dentro** de la
sala y el pasillo (zona 2) **fuera**: desde el PC no se ve el robot en las fases II y III.

## Estado

| Fase | Estado |
|---|---|
| I | **Terminada.** Vídeo grabado (A → Q, 2:49 a 0.20 m/s, `mission_log_ruta_video.csv`). Probada desde A, B (hasta E) y C (hasta Q) entre paneles de madera. D sin probar (el usuario la da por buena). |
| II | **~30 %.** Prototipo de ruta de cobertura (`test_real/prueba_pasillo.yaml`), primer mapa SLAM, bags de las estaciones, posición real de R medida. Falta casi todo lo importante (abajo). |
| III | **~5 %.** Solo medidas y un bag de aproximación a R. |

## Cómo está hecho (lo esencial)

- `autonomous_nav/autonomous_nav/fase1_node.py`: un único nodo. Estados: `INICIO` (salida desde B/C/D: junta scans y
  planifica), `GIRAR`/`AVANZAR`, `ESPERAR` (bloqueo → replan), `RELOC` (tipo `puerta`: `ajuste_puerta`; tipo `Q`:
  `ajuste_esquina`), `ORIENTAR`, `FIN`. Pose = odometría (rumbo de /odom, que viene del IMU de la OpenCR) + corrección
  continua con el mapa de distancias (`ajuste_mapa`, solo hasta Y 10.2) + relocalizaciones en la puerta y en Q.
  `pose_mundo()` es la única función que sabe de dónde sale la pose: **ahí entra el SLAM**.
- `planificador.py`: Distance Transform con costes (Dijkstra desde el objetivo). Offline (`scripts/planificar_fase1.py`
  genera `config/fase1_ruta.yaml`, `maps/holgura_5cm.*`, `docs/ruta_fase1_plan.png`) y online (replan con obstáculos).
- Configuración: `config/fase1.yaml` (valores por defecto, Gazebo), `config/fase1_ruta.yaml` (GENERADO, no editar),
  `config/real.yaml` (robot real), y en `test_real/`: `salida_{B,C,D}.yaml`, `prueba_pasillo.yaml`,
  `prueba_estaciones.yaml`, `prueba_aproximacion_R.yaml` (último `--params-file`; ver README_DEMO).
- `comprobar_pose`: pose inicial con el LiDAR contra las paredes del mapa. En A va bien; en B y C solo ve paredes del
  eje Y (los paneles tapan el fondo) y hay que escribir la pose a mano (README_DEMO, 3.2).
- Tests: `cd autonomous_nav && python3 -m pytest -q test` con `install/setup.bash` cargado → **96 passed, 1 xfailed**.

## Datos clave medidos el 06/11 (detalle en CLAUDE.md)

- **Las coordenadas del enunciado no siempre valen.** La **cruz de R está en (7.17, 16.51)**, no en (7.50, 16.17)
  (cinta desde Q; el detector, con el bag de aproximación, la pone en (7.19, 16.48)). **P Base sí coincide** (3.37, 15.70).
  T sin medir. Pared del extintor (Y alto del pasillo) en Y ≈ 17.15 (mapa 16.90); pared del fondo X 9.57.
- El mapa está mal en Y ~0.3 m en el tramo del escalón a la puerta y en la pared de Q (13.997 real frente a 13.780).
- Estaciones: patas Ø 5 cm, **44 cm entre centros** (49 de extremo a extremo), 25 cm libres debajo (el robot cabe, con
  ~10 cm por lado). R cuadrada, paralela a las paredes. T triangular (la que hay que descartar), girada. **Se pueden
  mover**: el detector no debe fiarse de las cruces.
- Paneles de madera (~30 cm, el LiDAR los ve como paredes) alrededor de C, **movibles**: la salida se planifica con lo
  que se ve al arrancar. Necesita huecos ≥ 0.6 m.
- `/scan`: 5 Hz, `range_max` 3.5 m, ~1° por rayo: una pata da 2-3 puntos a ≤ 1.2 m.

## Lo que queda, por orden

1. **Detector de pilares (fase II).** Prototipo en `test_real/datos_20261106/scripts_tarde/pilares_scan.py`: segmentos
   del scan de 1-8 rayos, ancho ≤ 9 cm, aislados. Solo saca 1 de 4 patas en `bag_fase1_20261106_171005` (aproximación a R:
   en bruto se ven las 4 a 0.70/8°, 0.88/37°, 1.14/4°, 1.23/24°): el test de aislamiento falla cuando hay algo detrás
   cerca. Luego: agrupar en el mundo, identificar **4 en cuadrado de 44 cm** (diagonal 62.2) frente a **3 a 44 cm**, y
   escribir en el Mission Log la posición de cada pilar y de la estación. Verdad de tierra: R en (7.17, 16.51), paralela a
   las paredes. Otros bags: `bag_fase1_20261106_161359` (rodeo de R y T; ojo, R allí sale con el mismo desfase),
   `bag_fase1_20261106_163111` (ensayo con SLAM, dos pasadas por el pasillo).
2. **Localización con SLAM en el pasillo.** El usuario lo ha decidido. Comando que usan:
   `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false` (arrancado con el robot en A). En el bag de las
   16:31 (`bag_fase1_20261106_163111`) están /tf con map→odom y /map: se puede desarrollar offline. Hay que fijar la
   transformación mundo↔map con la pose de A al arrancar y que `pose_mundo()` la use (al menos en el pasillo). Sin esto
   se llega a P Base con 15-20 cm de error (24 m sin corrección).
3. **Mapa entregable.** El de `mapa_slam/mapa_fases_I_II_20261106.*` sale con el vestíbulo deformado (abanico) y el
   pasillo girado respecto a la sala; origen en el sistema de /odom, no en A. Ajustar slam_toolbox (resolución, loop
   closure, alcance 3.5 m) y volver a grabarlo en el laboratorio.
4. **Integrar la fase II en el nodo**: `current_phase` = II en el log (ahora siempre "I"), fases I y II en un solo
   lanzamiento (ahora dos con Ctrl+C entre medias), ruta de cobertura tras Q, vuelta a P Base, guardar el mapa.
5. **Pose final asignada por el profesor**: HECHO en código (06/11, al final): si `final_pose` queda a más de
   `final_plan_dist` (0.30 m) de Q, al pasar Q se planifica hasta ella (`PLAN_FINAL` / `FINAL_SIN_CAMINO`); antes iba en
   línea recta (al vestíbulo, p. ej. (7.5, 12.6), cruzaba una pared). En README_DEMO ("Pose final"). **Sin probar en el
   robot.** Sin corrección después de Q: el error crece con la distancia (lo arregla la localización por SLAM).
6. **Fase III**: de P Base a delante de la estación, alinearse con las patas en directo, entrar entre las dos de delante
   y parar sobre la cruz (con la pose relativa a los pilares, no la odometría). Desarrollar con
   `bag_fase1_20261106_171005`.
7. **Entregables**: vídeo de la misión completa, informe (diagrama de nodos y topics, lógica de las fases I y II,
   estrategia de evitación), declaración de uso de IA. Mucho material ya está en las NOTAS de cada día.

Pendiente de entender (no bloquea): 6-9° de rumbo acumulados entre la puerta y Q (la RELOC_Q lo corrige).

## Cosas que han costado y conviene saber

- **Nunca dos `fase1_node` a la vez** (pasó el 06/11: uno viejo en FIN publicaba v = 0). Comprobar `ros2 topic info
  /cmd_vel` → 0 antes de lanzar. El modo automático no deja matar procesos del usuario: pídeles el Ctrl+C.
- **Arrancar el bag antes del nodo** (se perdió media prueba el 06/11). `test_real/grabar.sh`.
- Con `pkill -f patrón` desde la herramienta Bash el patrón coincide con el propio comando y lo mata: usa
  `pkill -INT -f "ros2 bag recor[d]"`.
- A 0.20 m/s sin rampa el rumbo del IMU saltaba ~30° al arrancar: `max_linear_accel` 0.20 lo arregla.
- `obstacle_min_range` 0.20: lo que está a menos de 20 cm del LiDAR no entra en la memoria de obstáculos.
- Git: el autor no está configurado en el PC del laboratorio: `git -c user.name="rob" -c user.email="rob@c5s202pc109.fib.upc.es" commit ...`
  (el que se ha usado siempre). Subir los bags comprimidos (`tar | zstd -19`, < 50 MB) con `git add -f`.
  Al comprobar tests con grep, ojo: "xfailed" contiene "failed".
- En el PC del laboratorio puede haber un `gz sim` abierto en el dominio 11 (sin bridge no molesta; con bridge mezcla
  /scan y /odom). Y `~/Baixades/rob-autonomous-nav` es una copia vieja: el bueno es `~/ros2_ws/src/rob-autonomous-nav`.
- Forma de trabajar con el usuario: tras cada prueba en el robot, analizar el Mission Log y el bag, guardar log + bag +
  fotos en `test_real/datos_AAAAMMDD/`, apuntarlo en su `NOTAS.md` y hacer commit + push. Cambios pequeños y de uno en
  uno, con test (que falle con el código anterior) y comprobados con los bags antes de ir al robot.

## Preguntas abiertas para el profesor

- ¿Se puede usar slam_toolbox (lo asumimos) y Nav2 (no lo usamos)?
- ¿Dónde estará la pose final en la demo? ¿Dan θ en la salida?
- ¿Las estaciones y los paneles cambian de sitio en la demo? ¿Cuál es exactamente la "zona 2"?
