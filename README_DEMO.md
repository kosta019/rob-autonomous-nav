# Demo: guía paso a paso (sin Claude)

Todo lo que hay que hacer el día de la demo, desde un PC del laboratorio recién encendido. Probado el 06/11/2026 con
BOT 11. Las pruebas, los datos y el porqué de cada parámetro están en `test_real/datos_20261106/NOTAS.md`.

**Estado (06/11):** fase I completa y probada desde A (2:49 a 0.20 m/s) y desde C (hasta Q) y B (hasta E) entre paneles. Fase II: solo un **prototipo**
(recorre todo el pasillo y vuelve a P Base; todavía no detecta la estación). Fase III: **sin hacer**.

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
# Terminal 2 (el mapa entregable se construye durante las fases I y II; arrancarlo con el robot en la salida)
mkdir -p ~/ros2_ws/mapas && cd ~/ros2_ws/mapas && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

## 3. Fase I (terminal 3)

### 3.1 Salida desde A

Robot sobre la cruz de A, mirando a +X (de espaldas a la pared del fondo, hacia B), nadie a menos de 1 m.
```bash
ros2 run autonomous_nav comprobar_pose $CFG/fase1.yaml $CFG/real.yaml --escribir
```
Tiene que acabar con `Escrito initial_pose_corregida.yaml` **sin ningún AVISO** (pose ≈ (0.70, 0.91, ~1°)). Luego:
```bash
ros2 run autonomous_nav fase1_node --ros-args \
  --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml --params-file $CFG/real.yaml \
  --params-file initial_pose_corregida.yaml
```

### 3.2 Salida desde B, C o D

Robot sobre la cruz, **mirando a +X** (hacia D). Si el profesor da otro ángulo θ, cambiarlo en `initial_pose` del
fichero de esa salida (`test_real/salida_B.yaml`, `salida_C.yaml` o `salida_D.yaml`, en radianes).
```bash
S=src/rob-autonomous-nav/test_real/salida_C.yaml          # o salida_B.yaml / salida_D.yaml
ros2 run autonomous_nav comprobar_pose $CFG/fase1.yaml $CFG/real.yaml $S --escribir
```
- **Sin AVISO:** ya está escrito `initial_pose_corregida.yaml`.
- **Con AVISO "solo hay paredes de un eje"** (típico en C: los paneles tapan la pared del fondo): no escribe el fichero.
  Escribirlo a mano con la X de la cruz y la Y y θ que dice `Pose estimada` (ejemplo del 06/11 en C):
  ```bash
  printf 'fase1_node:\n  ros__parameters:\n    initial_pose: [2.078, 2.544, 0.0263]\n' > initial_pose_corregida.yaml
  ```
  En B pasa lo mismo (solo ve las paredes del eje Y). 06/11: `[3.102, 1.729, 0.0314]`. Cruces: B (3.102, 1.722),
  C (2.078, 2.517), D (5.285, 2.517).
- **Con AVISO de ecos a < 0.20 m:** si hay un panel pegado al robot es normal; si no, limpiar la ventana del LiDAR.

```bash
ros2 run autonomous_nav fase1_node --ros-args \
  --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml --params-file $CFG/real.yaml \
  --params-file $S --params-file initial_pose_corregida.yaml
```
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
- seguidas de una fila `PILAR` por pata, con su posición en `obstacle_x/obstacle_y`.

La posición es la de la pose del robot en ese momento: hereda su error (en el pasillo, sin SLAM, ~10 cm).
**Limitaciones:** con `prueba_pasillo.yaml` el log dice fase "I" (es otro lanzamiento de la fase I); la ruta no cambia al
encontrar la estación; llega a P Base con ~15-20 cm de error (sin corrección en el pasillo).

## 5. Guardar el mapa del SLAM (al llegar a P Base, con el SLAM aún en marcha)

```bash
cd ~/ros2_ws/mapas && ros2 run nav2_map_server map_saver_cli -f mapa_demo --ros-args -p use_sim_time:=false
ls ~/ros2_ws/mapas      # mapa_demo.pgm + mapa_demo.yaml: el entregable
```
Después, Ctrl+C en el nodo, en el SLAM (terminal 2) y en el bag (terminal 1).

## 6. Fase III

**Sin hacer** (06/11). Estación buena: la cuadrada, 4 patas Ø 5 cm a 44 cm entre centros, 25 cm libres debajo; el robot
cabe entre las patas con ~10 cm por lado. Medidas en `CLAUDE.md`.

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
