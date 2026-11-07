# Demo: guía paso a paso (sin Claude)

Todo lo que hay que hacer el día de la demo, desde un PC del laboratorio recién encendido. Probado el 06/11/2026 con
BOT 11. Las pruebas, los datos y el porqué de cada parámetro están en `test_real/datos_20261106/NOTAS.md`.

**Estado (07/11):** fase I completa y probada desde A (2:49 a 0.20 m/s) y desde C (hasta Q) y B (hasta E) entre paneles.
Fase II probada como prototipo el 06/11 (cobertura del pasillo y vuelta a P Base); el detector de estaciones, la segunda
cobertura, la corrección en el pasillo y el θ en P Base, **hechos en código (07/11), sin probar en el robot**. Fase III:
**hecha en código (07/11), sin probar** (sección 6). Pose de salida automática (07/11, sección 3.1).

---

## 0. Preparar el PC (una vez; el PC del laboratorio pierde todo al apagarse)

```bash
# Si el PC está recién instalado: el script del laboratorio pone ROS_DOMAIN_ID=30 en ~/.bashrc. BOT 11 es el 11:
grep ROS_DOMAIN_ID ~/.bashrc                       # debe decir 11; si no, edítalo
sudo apt install -y ros-jazzy-slam-toolbox         # si falta (si apt espera un bloqueo, esperar: no matar dpkg)

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/kosta019/rob-autonomous-nav.git
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
cd src/rob-autonomous-nav/autonomous_nav && python3 -m pytest -q test     # debe salir "passed" y 1 "xfailed"
```

En **cada terminal nueva** (hay que abrir tres):
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=11 TURTLEBOT3_MODEL=burger
cd ~/ros2_ws && CFG=src/rob-autonomous-nav/autonomous_nav/config
```

## 1. Robot

1. **Batería cargada** (≥ 12 V). Con la batería baja la OpenCR ha fallado (06/11: /odom saltó 11 km).
2. Encender, **bringup** en el robot y **no tocarlo 10 s** (calibra el giróscopo).
3. Comprobar desde el PC:
   ```bash
   ros2 topic hz /odom      # ~20 Hz      (Ctrl+C para salir)
   ros2 topic hz /scan      # ~5 Hz
   ros2 topic info /scan    # Publisher count: 1  (2 = hay un Gazebo con bridge en el dominio 11: cerrarlo)
   ros2 topic info /cmd_vel # Publisher count: 0  (1 = hay un fase1_node viejo vivo: Ctrl+C en su terminal)
   ros2 topic echo --once /battery_state --field voltage
   ```
4. **Seguridad:** el freno de emergencia es el **interruptor de la OpenCR**. Alguien siempre junto al robot. Nunca dos
   `fase1_node` a la vez: **Ctrl+C en cada nodo antes de lanzar el siguiente**.

## 2. Terminal 1: bag · Terminal 2: SLAM (con el robot ya en la salida)

```bash
# Terminal 1 (graba todo; parar con Ctrl+C al final). Comprueba que escribe "Grabando en ..."
src/rob-autonomous-nav/test_real/grabar.sh
# Terminal 2 (el mapa entregable se construye durante las fases I y II; arrancarlo con el robot en la salida).
# scan_slam reenvía /scan a /scan_slam con el sello -0.1 s y slam_toolbox lee ese (config/slam.yaml): mapa recto.
mkdir -p ~/ros2_ws/mapas && cd ~/ros2_ws/mapas && ros2 launch autonomous_nav slam.launch.py
```

## 3. Fase I (terminal 3)

### 3.1 Salida desde A

Robot sobre la cruz de A, mirando a +X (de espaldas a la pared del fondo, hacia B), **a ojo basta (±10°)**, nadie a
menos de 1 m ni delante de las paredes.
```bash
ros2 run autonomous_nav fase1_node --ros-args \
  --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml --params-file $CFG/real.yaml
```
**Pose de salida automática (07/11, `start_pose_check` en `real.yaml`):** antes de moverse, parado ~5 s, el nodo hace lo
que hacía `comprobar_pose --escribir`: corrige el rumbo y la posición con las paredes del mapa y comprueba el giróscopo.
- `POSE_INICIAL dx= dy= dth= ... paredes XY n= rms=`: corregida; sale. En A, ≈ (0.69, 0.91, −0.4°).
- `POSE_INICIAL_RECHAZADA <motivo>`: **no se mueve** y lo repite cada ~5 s. `giroscopo`: Ctrl+C, apagar y encender la
  OpenCR con el robot quieto. `grande`: el robot está a más de 10° o 15 cm de `initial_pose` (recolocarlo). `pocos_puntos`
  o `sin_paredes_rumbo`: algo tapa las paredes (apartarse). `residuo`: recolocarlo.
- 07/11, bag de las 15:47: con alguien cerca tapando parte de las paredes, la X salió 3.5 cm distinta (el rumbo y la Y,
  igual). Por eso: nadie junto al robot mientras está parado al principio.

`comprobar_pose` sigue valiendo para mirar la pose y los avisos antes de lanzar (sin `--escribir`). Si se pasa un
`initial_pose_corregida.yaml` antiguo como último `--params-file`, la corrección automática parte de esa pose.

### 3.2 Salida desde B, C o D

Robot sobre la cruz, **mirando a +X** (hacia D), a ojo (±10°). Si el profesor da otro ángulo θ, cambiarlo en
`initial_pose` del fichero de esa salida (`config/salida_B.yaml`, `salida_C.yaml` o `salida_D.yaml`, en radianes).
```bash
S=$CFG/salida_C.yaml          # o salida_B.yaml / salida_D.yaml
ros2 run autonomous_nav fase1_node --ros-args \
  --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml --params-file $CFG/real.yaml --params-file $S
```
La pose de salida se corrige sola, como en A. En B y C los paneles tapan la pared del fondo: corrige el rumbo y la Y, y la
X se queda la de la cruz (`(X sin paredes: la de initial_pose)` en `POSE_INICIAL`; antes se escribía a mano). Comprobado
con los bags del 06/11: B (3.102, 1.729, +1.77°) y C (2.078, 2.543, +1.54°), lo mismo que a mano. `AVISO_ECOS`: si hay un
panel pegado al robot es normal; si no, limpiar la ventana del LiDAR.
Se queda ~1 s parado mirando alrededor (`PLAN_SALIDA ... obstaculos=N`) y sale esquivando los paneles. Si sale
`SALIDA_SIN_CAMINO`: los huecos entre paneles son de menos de ~0.6 m; espera y reintenta solo cada 5 s.

### Pose final (la da el profesor; por defecto, Q)

Añadir al final del comando de `fase1_node` (θ en radianes; 0 = mirando a +X):
```bash
  -p final_pose:="[X, Y, THETA]"
```
Por ejemplo `-p final_pose:="[7.5, 12.6, 1.5708]"`. Si queda a más de 0.30 m de Q, al pasar Q planifica hasta ella
esquivando paredes y obstáculos (`PLAN_FINAL`; si no hay camino, `FINAL_SIN_CAMINO` y reintenta). Desde Q, el error
crece con la distancia (sin corrección después de Q): unos cm a 1-2 m, hasta ~15-20 cm a 10 m.
**Sin probar en el robot** (06/11): probarlo una vez antes de la demo con una pose final distinta de Q.

### 3.3 Qué debe pasar (y qué no)

Recorrido: (A →) B → D → E → (pasillo) → para ~2 s junto a la puerta y **relocaliza** → **Door** → **Q** (para, relocaliza
con la esquina, se acerca ~20 cm y repite) → gira a θ = 0 → `FASE I COMPLETADA`. A 0.20 m/s, unos 3 min desde A.

| Mensaje | Qué es | Qué hacer |
|---|---|---|
| `POSE_INICIAL ...` | pose de salida corregida con las paredes (parado, antes de moverse) | nada |
| `POSE_INICIAL_RECHAZADA <motivo>` | no se fía de la pose de salida: no se mueve y reintenta | ver 3.1 |
| `RELOC ... Y=canaleta`, `RELOC_Q ...` | relocalizaciones normales | nada |
| `RELOC_PARCIAL` + `SIN_CAMINO` en la puerta | alguien tapa el final de la canaleta | nada: `RELOC_REINTENTO` cada 5 s, en cuanto se aparta sigue |
| `BLOQUEO` / `REPLANIFICACION` | obstáculo en la ruta | nada |
| `OBLIGATORIO_TAPADO E` (o Q) | algo encima del punto: va al libre más cercano (< 0.40 m) | nada |
| `RODEO_LARGO` | el desvío sería muy largo: espera a que se vaya el obstáculo (30 s máx.) | nada |
| `RELOC_RECHAZADA`, `RELOC_INCONSISTENTE` | la relocalización de la puerta no cuadra | **Ctrl+C** antes de que entre en la puerta |
| Va pegado a una pared, o se acerca a < 30 cm de algo | — | **interruptor de la OpenCR** |
| Oscila parado (GIRAR/AVANZAR sin avanzar) | rumbo del IMU saltando (tirones) | Ctrl+C, apagar y encender la OpenCR, 10 s quieto |

Al acabar: **Ctrl+C en el nodo** (sin mover el robot si sigue la fase II). Con `phase2_enabled:=true` no hace
falta: sigue solo con la fase II.

## 4. Fase II

**Misión completa (fases I y II en un solo lanzamiento):** el comando de la fase I (3.1 o 3.2) con
`-p phase2_enabled:=true` al final. Al acabar la fase I (`FASE I COMPLETADA`) se para 3 s (`phase2_start_delay`),
escribe `FASE II INICIO` y sigue con la ruta de cobertura de `fase1.yaml` (`phase2_*`) hasta P Base
(`FASE II COMPLETADA`). Desde ahí, `current_phase` es `II` en el Mission Log. Si la pose final de la fase I no está junto
al primer punto de la ruta (P1_e), planifica hasta él (`PLAN_FASE2`; sin camino, `FASE2_SIN_CAMINO` y reintenta).
Comprobado reproduciendo el bag del ensayo del 06/11 (`docker/reproducir_bag.sh`); **sin probar en el robot**.
**Al acabar la cobertura (07/11, sin probar en el robot):** si no ha confirmado la estación cuadrada, hace una segunda
cobertura (`SEGUNDA_COBERTURA`, pasadas en Y 14.65 y 15.85: `phase2_second_*`) y, si tampoco, escribe
`ESTACION_NO_ENCONTRADA` y vuelve a P Base. En P Base se orienta **mirando a la estación cuadrada**
(`THETA_HACIA_ESTACION <grados>`, con su centro en `obstacle_x/obstacle_y`); sin estación, al θ de `phase2_final_pose`.

**Solo la fase II** (terminal 3, robot en Q donde acabó la fase I):
```bash
ros2 run autonomous_nav fase1_node --ros-args --params-file $CFG/fase1.yaml --params-file $CFG/real.yaml \
  --params-file src/rob-autonomous-nav/test_real/prueba_pasillo.yaml
```
Dos pasadas a lo largo de todo el pasillo (todo queda a ≤ 1.2 m del robot) y vuelta a **P Base**. ~5 min a 0.12 m/s.
Las estaciones y los obstáculos se esquivan; las patas quedan como `OBSTACULO` en el Mission Log.
**Estaciones:** el detector de pilares (`pilares.py`) las identifica solo, dentro de `pillar_zone` (el pasillo). Cuando
ha visto una en 5 scans, el Mission Log y la terminal dicen dónde está:
- `ESTACION_CUADRADA` (la buena) o `ESTACION_TRIANGULAR` (la que hay que descartar): centro en `obstacle_x/obstacle_y`,
  y ángulo de la estación (grados) en `error`;
- seguidas de una fila por pata, con su posición en `obstacle_x/obstacle_y`: `PILAR` (de la cuadrada, la estación de
  carga del enunciado) o `PILAR_DESCARTADA` (de la triangular).

La posición es la de la pose del robot en ese momento: hereda su error (en el pasillo, ~5-10 cm, sobre todo en X).

**Corrección en el pasillo (07/11, sin probar en el robot):** dentro de `corridor_corr_zone`, cada segundo y sin girar,
la Y se corrige con la pared del extintor medida con cinta (Y 17.15, `corridor_wall_*` en `real.yaml`): eventos
`CORR_PASILLO dx= dy= nx= ny=` (n = puntos de pared usados), como mucho 3 cm por paso. El 06/11 el robot llegó a P Base
con ~15 cm de error hacia esa pared (sin corrección); con ella, en los bags, lo habría visto y corregido. Las paredes que
se midan (p. ej. el extremo X −0.55) se añaden en `real.yaml`, sin tocar código. Para quitarla:
`-p corridor_corr_mode:=apagado` (o `sombra`: solo la registra). Análisis: `docs/analisis_pasillo_20261107.md`.

**Limitaciones:** con `prueba_pasillo.yaml` el log dice fase "I" (es otro lanzamiento de la fase I); la ruta no cambia al
encontrar la estación; la X en el pasillo solo la da la odometría (4-9 cm junto a R en los bags).

## 5. Guardar el mapa del SLAM (al llegar a P Base, con el SLAM aún en marcha)

Con la fase III encadenada (`phase3_enabled:=true`), el robot sigue solo tras `FASE II COMPLETADA`: guardar el mapa en
ese momento (es el de las fases I y II, el que pide el enunciado) mientras empieza la fase III; el SLAM sigue en marcha.

```bash
cd ~/ros2_ws/mapas && ros2 run nav2_map_server map_saver_cli -f mapa_demo --ros-args -p use_sim_time:=false
ls ~/ros2_ws/mapas      # mapa_demo.pgm + mapa_demo.yaml: el entregable
```
Después, Ctrl+C en el nodo, en el SLAM (terminal 2) y en el bag (terminal 1).

## 6. Fase III

**Hecha en código (07/11), sin probar en el robot.** Misión completa: el comando de la fase I con
`-p phase2_enabled:=true -p phase3_enabled:=true`. Tras `FASE II COMPLETADA` (en P Base, mirando a la estación), 3 s
parado y `FASE III INICIO` (`current_phase` III):
1. Va a un punto **0.80 m delante** de la estación cuadrada, perpendicular al lado más cercano con sitio (en R, el de P
   Base o el del pasillo; por el de la pared no), y se orienta al centro (`FASE3_DELANTE`).
2. **Parado ~4 s** (`MEDIR3`), mide las patas y el centro **relativo al robot**: `FASE3_CENTRO patas=N` (centro en
   `obstacle_x/y`). Con el bag del 06/11 a ~1 m de R: centro a 0.6-1.2 cm de la cinta en 6 de 6 medidas. Si no las ve o el
   centro está a más de 0.25 m del de la fase II: `FASE3_MEDIDA_FALLIDA`, y a la 3.ª `FASE_III_ABANDONADA` (**no entra**).
   Si desde ahí entraría torcido (más de 6° respecto a la perpendicular de un lado: el ángulo de la fase II venía mal), no
   entra: `FASE3_REALINEAR <grados>`, va al punto delante con el ángulo medido de cerca y vuelve a medir (cuenta como
   intento).
3. **Entra recto a 0.05 m/s** hasta el centro (`FASE3_DENTRO`). La parada de emergencia no para por las 4 patas medidas
   (con el robot centrado ni siquiera entran en el cono), pero sí por cualquier otra cosa.
4. **Parado dentro** (`VERIFICAR3`), mide las patas alrededor: `FASE III COMPLETADA patas=N` con la distancia del centro
   del robot al de la estación (m) en `error`. Más de 3 cm: además `APARCADO_FUERA_DE_TOLERANCIA`.

**Probarla el lunes con alguien junto a la estación (interruptor de la OpenCR).** Si roza una pata o el aparcado sale fuera
de tolerancia, no entra en el vídeo. Solo la fase III, con el robot en P Base: `-p initial_pose:="[3.34, 15.70, θ]"`
no basta (la estación se identifica en la fase II): hay que lanzar la misión completa.
Estación buena: la cuadrada, 4 patas Ø 5 cm a 44 cm entre centros, 25 cm libres debajo; el robot cabe entre las patas
con ~10 cm por lado. Medidas en `CLAUDE.md`.

## 7. Guardar todo en el repo

```bash
cd ~/ros2_ws/src/rob-autonomous-nav
R=test_real/datos_$(date +%Y%m%d); mkdir -p $R/mapa_slam
cp ~/ros2_ws/mission_log_*.csv $R/ && cp ~/ros2_ws/mapas/mapa_demo.* $R/mapa_slam/
BAG=$(ls -td bags/fase1_* | head -1); tar -cf - -C bags $(basename $BAG) | zstd -T0 -19 -o $R/bag_$(basename $BAG).tar.zst
git add -f $R && git -c user.name="..." -c user.email="..." commit -m "Demo: logs, bag y mapa" && git push
```

## Vídeo

Plano general del robot + el terminal del nodo a la vista (se leen los eventos). Al final de la fase I, plano cenital
sobre la cruz de Q. Si se cruza un robot u obstáculo, que se vea cómo espera o esquiva.
