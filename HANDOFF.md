# Handoff para el siguiente agente (actualizado el 09/11/2026, antes del laboratorio)

Léelo entero antes de tocar nada. Después, en este orden: `CLAUDE.md` (reglas del proyecto, datos medidos, decisiones
cerradas), `README_DEMO.md` (cómo se opera el robot: es lo que usará el equipo en la demo, sin agente),
`entrega/PASOS_ENTREGA.md` (todos los pasos hasta la entrega) y `test_real/datos_20261106/NOTAS.md` (todas las pruebas
del 06/11, con el porqué de cada parámetro).

## Situación al acabar el laboratorio del 09/11 (LO PRIMERO: lo de la sección siguiente es de esta mañana)

- **Entrega: martes 10/11 a las 10:00.** Detalle de todas las pruebas de hoy: `test_real/datos_20261109/NOTAS.md`.
- **Misión de la entrega: la de las 16:39** (elegida por Marc: BOT 11, fases I + II + III desde A, 480 s, sin choques, fase I
  sin paradas): `mission_log_bot11_sin_paradas.csv`, `mapa_sin_paradas.*` y su bag, en esa carpeta. Lanzada con
  `-p phase2_enabled:=true -p phase3_enabled:=true -p reloc_pbase_times:=0`. El vídeo no está en el repo (lo tiene Marc). La de
  las 16:22 (429 s, aparcado verificado a 3.2 cm) queda de reserva: `mission_log_bot11_mision_final.csv`, `mapa_final.*`.
- **Ramas:** todo está en `main` (09/11 por la noche), incluido `phase1_pass_speed` (0.06 en `real.yaml`, 0 lo apaga). La etiqueta
  `viernes-06-11` marca el código del viernes. `memoria` sigue viva solo hasta entregar.
- **Decidido (Marc, 09/11):** `reloc_pbase_times: 0` en `real.yaml`, igual que en el vídeo. Ya no hace falta pasarlo con `-p`.
- **Hay que regenerar `entrega/mission_log.csv` y `entrega/mapa/`** (están en `.gitignore`):
  `python3 entrega/log_entrega.py test_real/datos_20261109/mission_log_bot11_sin_paradas.csv entrega/mission_log.csv`
  y `mkdir -p entrega/mapa && cp test_real/datos_20261109/mapa_sin_paradas.* entrega/mapa/`. Comprobar a mano que el mapa
  muestra las patas: a 5 cm no se ha podido confirmar.
- **Probado hoy en el robot, y funciona:** `POSE_INICIAL`, `CORR_PASILLO` (`activo`), `scan_slam`, fase II en el nodo, detector de
  estaciones (se mueven cada día), fase III (3 de 5 intentos entra; sin choques) y `FASE3_OTRO_LADO`.
- **Limitaciones conocidas:** el rumbo se tuerce en la fase II (3.6° a 12°, varía de una misión a otra). Sin relocalización,
  P Base queda con 17-27 cm de error en X (con ella, bien; pero con ella la fase III abandonó 2 de 2: `sin_patas`, causa sin
  averiguar). La corrección de rumbo con paredes no pasó la validación: `rumbo_paredes_no_validado.patch`, fuera del código.
- **BOT 04** (`ROS_DOMAIN_ID=4`), usado esta tarde al no estar el 11: su rumbo se desvía 17° en la fase II y su LiDAR podría
  estar girado 5-7° (sin confirmar). `real.yaml` está calibrado para el BOT 11 (`ROS_DOMAIN_ID=11`).
- **En este PC** `~/ros2_ws/src/rob-autonomous-nav` es un enlace a `~/Baixades/rob-autonomous-nav`, sin `--symlink-install` (tras
  cambiar código, `colcon build`). Sin paquetes de TurtleBot3 (no se puede lanzar Gazebo). `.bashrc` trae `ROS_DOMAIN_ID=04`.

## Situación el 09/11 por la mañana, antes del laboratorio

- **Entrega: martes 10/11/2026 a las 10:00.** Hoy es el último día de laboratorio.
- **Rama de trabajo: `arreglo-fase3-lado`.** `main` sigue con el código del viernes 06/11 y con un handoff de ese día: no
  sirve para el laboratorio. En el PC: `cd ~/ros2_ws/src/rob-autonomous-nav && git fetch && git checkout
  arreglo-fase3-lado && git pull`, y `colcon build` (README_DEMO 1).
- **Nada del código escrito desde el 07/11 se ha probado en el robot:** `POSE_INICIAL`, `CORR_PASILLO`, `scan_slam`, la
  fase II dentro del nodo, `THETA_HACIA_ESTACION`, la segunda cobertura y toda la fase III. Solo tests, bags (lazo
  abierto) y Gazebo.
- **Corrección de la fase III** (commit `2afc08d`, sin verificar en el robot): si un panel tapa el punto de delante de la
  estación, elige otro lado (`phase3_side_wait`, 10 s; `0.0` la desactiva). Cómo montar la prueba:
  `sim/fallos/fase3_lado_tapado/README.md`. No fusionarla en `main` hasta que Marc la verifique. Si falla,
  `git revert 2afc08d` (comprobado: sale sin conflictos).
- **Simulaciones en Gazebo** (`sim/`, 300 misiones por batería, con paneles, cajas y persona móvil): sin la corrección,
  293/300 y 0 choques; con ella, **300/300, 0 choques, 0 contactos y 0 atascos**. Fallos documentados en `sim/fallos/`.
- **Orden recomendado hoy:** preguntas al profesor → primera misión desde A **ya grabando vídeo**, con bag y SLAM,
  guardando el mapa al salir `FASE II COMPLETADA` → subir esos datos → fase III con panel → medidas con cinta → aplicar
  las medidas solo si sobra tiempo (y entonces misión de comprobación y vídeo nuevo). Asegurar primero una entrega válida.
  Si falta tiempo, se recorta en este orden: mesas entre B y C (solo anotar; `planificar_fase1.py` necesita scipy, que
  solo está en el portátil), paredes nuevas del pasillo, prueba del panel. Nunca el vídeo con bag y mapa.
- **Entrega:** `entrega/crear_zip.sh` (desde HEAD), `entrega/log_entrega.py` (Mission Log con los obstáculos del pasillo),
  `entrega/CONTENIDO.md`. El borrador de la declaración de IA está solo en el portátil de Marc, fuera de git a propósito.
- **Hoy no se hace limpieza de ramas ni se avanza `main`:** queda para después del laboratorio.
- **Reglas de hoy:** `fase1_node` lo lanza Marc en su terminal, nunca en segundo plano. No cambiar el nodo sin enseñarle
  antes los fallos con números.

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
| II | **Código hecho, sin probar en el robot.** Cobertura en el nodo, detector de pilares, corrección con la pared del extintor, segunda cobertura y mapa con `scan_slam`. Comprobado con bags y en Gazebo. |
| III | **Código hecho, sin probar en el robot.** `MEDIR3`, `ENTRAR`, `VERIFICAR3`. Con el bag de las 17:10, centro a 0.6-1.2 cm de la cinta; en Gazebo aparca. |

## Cómo está hecho (lo esencial)

- `autonomous_nav/autonomous_nav/fase1_node.py`: un único nodo. Estados: `INICIO` (salida desde B/C/D: junta scans y
  planifica), `GIRAR`/`AVANZAR`, `ESPERAR` (bloqueo → replan), `RELOC` (tipo `puerta`: `ajuste_puerta`; tipo `Q`:
  `ajuste_esquina`), `ORIENTAR`, `FIN`. Pose = odometría (rumbo de /odom, que viene del IMU de la OpenCR) + corrección
  continua con el mapa de distancias (`ajuste_mapa`, solo hasta Y 10.2) + relocalizaciones en la puerta y en Q.
  `pose_mundo()` es la única función que sabe de dónde sale la pose: **ahí entra el SLAM**.
- `planificador.py`: Distance Transform con costes (Dijkstra desde el objetivo). Offline (`scripts/planificar_fase1.py`
  genera `config/fase1_ruta.yaml`, `maps/holgura_5cm.*`, `docs/ruta_fase1_plan.png`) y online (replan con obstáculos).
- Configuración: `config/fase1.yaml` (valores por defecto, Gazebo), `config/fase1_ruta.yaml` (GENERADO, no editar),
  `config/real.yaml` (robot real), `config/salida_{B,C,D}.yaml` y en `test_real/`: `prueba_pasillo.yaml`,
  `prueba_estaciones.yaml`, `prueba_aproximacion_R.yaml` (último `--params-file`; ver README_DEMO).
- `comprobar_pose`: pose inicial con el LiDAR contra las paredes del mapa. En A va bien; en B y C solo ve paredes del
  eje Y (los paneles tapan el fondo) y hay que escribir la pose a mano (README_DEMO, 3.2).
- Tests: `cd autonomous_nav && python3 -m pytest -q test` con `install/setup.bash` cargado → **150 passed, 1 xfailed** en `arreglo-fase3-lado` (o `docker/tests.sh`).

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

## Decisiones del usuario (07/11) y plan para el lunes 09/11 (3-4 h de laboratorio: solo pruebas, ajustes y vídeo)

- **Diapositivas de clase en `docs/teoria/`** (fuera de git): Wheeled Mobile Robots, RM navigation, Sensors,
  Localization and mapping. **Antes de seguir, comprobar cada técnica contra ellas** (con número de diapositiva):
  detector de pilares (`pilares.py`: su comentario dice "teoría: extracción de características" SIN comprobar), la
  corrección con paredes del pasillo, la fase III y el SLAM. Si algo no encaja, proponer la alternativa de clase.
- El usuario revisa la rama `fase2-pilares` y la fusiona él en `main`.
- Vídeo definitivo **desde A**. Pasillo a **0.12 m/s** en el primer vídeo; si va bien, se subirá.
- Fase III: aparcado dado por bueno con el centro a **≤ 3 cm** de la cruz.
- Mission Log: las patas salen como `OBSTACULO` (todos los puntos) y hay que **indicar cuáles de ellos son las patas**
  (hoy, filas `PILAR` aparte con su posición; revisar que se pueda cruzar una cosa con la otra).
- **Rumbo inicial automático (confirmado: es el rumbo del robot en la salida):** el θ de salida puede variar; el nodo
  debe corregirlo solo al arrancar, parado, con el LiDAR contra las paredes (lo que hoy hace a mano
  `comprobar_pose --escribir`), antes de moverse.
- **Confirmado por el usuario (07/11), a hacer antes del lunes, en este orden:**
  1. **HECHO (07/11, sin probar en el robot):** localización en el pasillo con sus paredes medidas. No con
     `ajuste_mapa` (con fondo y pared de Q empeoraba): `ajuste_paredes` corrige solo la Y con la pared del extintor
     (Y 17.15, puntos a ≤ 2 m), activa en `real.yaml` (`corridor_*`), eventos `CORR_PASILLO`. En los bags: R igual
     (±0.3 cm) y en P Base habría corregido los ~15 cm. Detalle y medidas para el lunes (predicción: de la cruz de
     P Base a la pared del tramo ancho ≈ 2.41 m; el extremo X −0.55) en `docs/analisis_pasillo_20261107.md`. Las
     paredes que se midan se añaden en `real.yaml` (`corridor_wall_*`), sin tocar código.
  2. **HECHO (07/11, sin probar en el robot):** pose de salida automática. Estado `POSE_INICIAL` (parado, 25 scans):
     `comprobar_pose.estimar` contra las paredes del mapa; rumbo siempre y cada coordenada solo si ve paredes de su eje
     (`revisar_pose_inicial`); giróscopo comprobado. Si no vale (`POSE_INICIAL_RECHAZADA <motivo>`), no se mueve y
     reintenta. `start_pose_check: true` en `real.yaml` (falso en `fase1.yaml` y en las pruebas desde Q). Converge con el
     robot a ±10° del θ dado. Con los bags del 06/11, B y C dan lo mismo que a mano; en A, con alguien tapando paredes, la
     X sale 3.5 cm distinta (README_DEMO 3.1).
  3. Nodo que reenvía /scan a /scan_slam con el sello −0.1 s, para slam_toolbox (mapa entregable). Comprobar que el
     mapa muestra las patas de la estación (el enunciado lo pide: "charging station pillars"). **HECHO (07/11):** `scan_slam` +
     `config/slam.yaml` + `launch/slam.launch.py` (terminal 2 de README_DEMO). Con el bag de las 16:31: mapa recto y con
     las patas de R (cuadrado de 44 cm) y de T (`docs/analisis_slam_20261106.md`, sección del 07/11). Falta grabarlo en el
     laboratorio el lunes.
  4. Fase III: P Base → delante de R → alinearse con las patas (pose relativa a ellas) → entrar entre las dos de
     delante → parar con el centro a ≤ 3 cm de la cruz. La parada de emergencia no puede saltar por las patas
     laterales ya conocidas. Desarrollar con `bag_fase1_20261106_171005`. El usuario la prueba el lunes; si choca, no
     entra en el vídeo. **HECHO en código (07/11, sin probar en el robot):** `phase3_enabled`; estados MEDIR3 (centro
     relativo a las patas, `pilares.centro_cuadrada` con 3 o 4), ENTRAR (0.05 m/s, sin corrección de pose) y VERIFICAR3
     (error real del aparcado en el log). Con el bag de las 17:10: centro a 0.6-1.2 cm de la cinta (6/6). README_DEMO 6.
  5. **HECHO (07/11, sin probar en el robot):** θ en P Base mirando a la estación cuadrada confirmada
     (`THETA_HACIA_ESTACION`); sin ella, el de `phase2_final_pose`.
- **Descartado: acortar la fase II al encontrar R.** El enunciado: "Once the Corridor area is completely explored and
  the station is identified, the robot has to return to Punt Base", y el mapa debe salir "during the completion of the
  first two phases". Hay que explorar el pasillo entero, y el mapa entregable sale de esa misma ejecución.
- **Revisión contra el enunciado (07/11)**, a tener en cuenta:
  - *"without external assistance throughout all stages"*: la fase III debe seguir sola tras la II en el mismo
    lanzamiento (como la I→II), con `current_phase` = III. Y el rumbo inicial automático quita el paso manual de
    `comprobar_pose`.
  - *"completely explored and the station is identified"*: si al acabar la cobertura no se ha confirmado la cuadrada,
    hoy va igual a P Base y da la fase II por completada. **Decidido (07/11):** hacer una segunda cobertura (ruta en
    `config`, no en el código; p. ej. pasadas a otras Y para cubrir lo que la primera ve peor) y, si tampoco la
    encuentra, registrarlo en el Mission Log (p. ej. `ESTACION_NO_ENCONTRADA`) y volver a P Base. **HECHO (07/11, sin probar en
    el robot):** `phase2_second_*` (pasadas en Y 14.65 y 15.85), eventos `SEGUNDA_COBERTURA` y `ESTACION_NO_ENCONTRADA`.
  - El mapa entregable debe mostrar las patas de la estación (comprobar con el nodo del scan resellado).
  - Mission Log: pide obstáculos "within the passadis area"; hoy `OBSTACULO` se registra en toda la misión (con
    `current_phase`, se puede filtrar en el informe). Las patas de la cuadrada son `PILAR`; las de la triangular,
    `PILAR_DESCARTADA` (cambiado el 07/11).
  - Fase III frente a *"conservative clearance buffer"*: entrar a ~10 cm de las patas es la propia maniobra que pide el
    enunciado; justificarlo en el informe (velocidad baja, pose relativa a las patas, parada si algo inesperado).
- Plan del lunes: el orden está en "Situación el 09/11", al principio de este fichero. Prioridad: vídeo de fases I+II
  antes que la fase III.
- Preguntas al profesor (el lunes al llegar): slam_toolbox, pose final de la fase I y su θ, si se mueven estaciones y
  paneles, tolerancia del aparcado.

## Historial de tareas (hasta el 07/11)

Lo que queda de verdad está en `entrega/PASOS_ENTREGA.md`. Esta lista se conserva por el detalle técnico de cada parte.

1. **Detector de pilares (fase II): HECHO** (rama `fase2-pilares`, sin probar en el robot, pero sí con el nodo real
   reproduciendo los bags). `autonomous_nav/pilares.py` (solo numpy) + `Fase1Node.detectar_pilares` → filas
   `ESTACION_CUADRADA` / `ESTACION_TRIANGULAR` y `PILAR` en el Mission Log (README_DEMO, fase II). El prototipo perdía
   patas porque junto a ellas sale un rayo a 0.0 (sin dato) que tomaba por "algo más cerca". Resultados: bag 17:10 →
   cuadrada en (7.17, 16.50), −2.4° (cinta: 7.17, 16.51, paralela); bag 16:13 → R y T, cada una una vez; bag 16:31 →
   R y T, ninguna estación falsa. Las pasadas largas sitúan R a ~10 cm (deriva de la pose: lo arregla el SLAM).
   Herramientas sin ROS en el portátil: `scripts/leer_mcap.py`, `scripts/pilares_bag.py`; con Docker: `docker/tests.sh`
   (todos los tests) y `docker/reproducir_bag.sh` (el nodo con un bag grabado).
2. **Localización con SLAM en el pasillo: ANALIZADO, NO CONECTADO** (`docs/analisis_slam_20261106.md`). En su lugar,
   la pared del extintor (`ajuste_paredes`, 07/11: ver el punto 1 de las decisiones confirmadas). Pasando
   slam_toolbox otra vez por el bag de las 16:31 con varias configuraciones, su pose sitúa R a 0.39-0.83 m de la cinta;
   la del nodo, a 0.08. El scan llega ~0.1 s tarde (resellarlo mejora el SLAM, pero no basta). Pendiente de decidir
   (usuario): corregir la pose en el pasillo con sus paredes medidas (como `ajuste_mapa` en la sala) en vez del SLAM.
3. **Mapa entregable.** El del 06/11 sale con el pasillo girado. Offline salen mucho mejor con el scan resellado
   −0.1 s o sin emparejar scans (figura en el análisis). Falta decidir cuál y volver a grabarlo en el laboratorio.
4. **Integrar la fase II en el nodo: HECHO en parte** (rama `fase2-pilares`, sin probar en el robot). Con
   `-p phase2_enabled:=true`, al acabar la fase I: PAUSA (`phase2_start_delay`), `FASE II INICIO`, ruta de cobertura
   (`phase2_*` en `fase1.yaml`) y P Base; `current_phase` = II en el log. Solo se planifica el primer tramo (si la pose
   final de la fase I queda lejos de P1_e): planificar las pasadas las llevaría al centro del pasillo. Comprobado
   reproduciendo el bag de las 16:31 desde Q (`OFFSET=340 docker/reproducir_bag.sh ... docker/fase2_desde_Q.yaml`):
   P1_e, P1_o, P2_o y las dos estaciones en fase II; después el replan diverge del robot grabado (lazo abierto: una
   reproducción solo vale mientras el nodo decide lo mismo que en el laboratorio). Falta: guardar el mapa al llegar a
   P Base (ahora a mano, README_DEMO 5) y, si se quiere, acortar la ruta al encontrar la estación cuadrada.
5. **Pose final asignada por el profesor**: HECHO en código (06/11, al final): si `final_pose` queda a más de
   `final_plan_dist` (0.30 m) de Q, al pasar Q se planifica hasta ella (`PLAN_FINAL` / `FINAL_SIN_CAMINO`); antes iba en
   línea recta (al vestíbulo, p. ej. (7.5, 12.6), cruzaba una pared). En README_DEMO ("Pose final"). **Sin probar en el
   robot.** Sin corrección después de Q: el error crece con la distancia (lo arregla la localización por SLAM).
6. **Fase III: HECHO en código** (07/11, sin probar en el robot): ver el punto 4 de las decisiones confirmadas.
7. **Entregables**: ver `entrega/PASOS_ENTREGA.md`. El informe se redacta en Google Docs desde otra sesión
   (`HANDOFF_MEMORIA.md`).

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
