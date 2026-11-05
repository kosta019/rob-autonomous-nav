# Guía: ruta completa de la fase I en el robot real (sin Claude)

Configuración actual (05/11): `real.yaml` a **0.08 m/s** y **0.4 rad/s**, relocalización en la puerta calibrada,
obligatorio tapado activo. Todos los comandos, en la terminal del PC del laboratorio.

## 0. Una vez por sesión
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11 TURTLEBOT3_MODEL=burger
cd ~/ros2_ws && git -C src/rob-autonomous-nav pull && colcon build --symlink-install && source install/setup.bash
pgrep -af "gz sim"            # NO debe salir nada (un Gazebo abierto mezcla /scan, /odom e /imu con el robot)
ros2 topic info /scan; ros2 topic info /odom; ros2 topic info /imu   # 1 publicador cada uno
ros2 topic info /cmd_vel      # 0 publicadores
```
- Si el robot gira solo sin nadie en /cmd_vel:
  `ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"`
- Si se enciende la OpenCR: **no tocar el robot ~10 s** (calibra el giróscopo).

## 1. Bag (otra terminal)
Si hay un bag viejo grabando, páralo con Ctrl+C. Después:
```bash
cd ~/ros2_ws/src/rob-autonomous-nav && source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=11 && test_real/grabar.sh
```

## 2. Robot en A, quieto, mirando a +X, nadie a menos de 1 m
```bash
cd ~/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=11
CFG=src/rob-autonomous-nav/autonomous_nav/config
ros2 run autonomous_nav comprobar_pose $CFG/fase1.yaml $CFG/real.yaml --escribir
```
Debe terminar con `Escrito initial_pose_corregida.yaml` y **sin ningún AVISO**. Pose esperada ~ (0.70, 0.91, ~1°).
Si sale un AVISO, no escribe la pose:
| AVISO | Qué hacer |
|---|---|
| giróscopo (/imu marca giro o el rumbo cambia con el robot quieto) | apagar y encender la OpenCR con el robot quieto en el suelo, no tocarlo 10 s, relanzar el bringup |
| ecos a menos de 0.20 m | limpiar la ventana del LiDAR (paño seco) y apartar lo que haya junto al robot |
| solo paredes de un eje / pocos puntos / residuo | mal colocado o algo tapando las paredes: recolocar en A y repetir |

## 3. Ruta (en la terminal donde tienes el Ctrl+C)
```bash
cd ~/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=11 TURTLEBOT3_MODEL=burger
CFG=src/rob-autonomous-nav/autonomous_nav/config
ros2 run autonomous_nav fase1_node --ros-args \
  --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml --params-file $CFG/real.yaml \
  --params-file initial_pose_corregida.yaml
```
- Recorrido: A → B → D → D-E_1 → **E** → E-Door_1 → E-Door_2 (para ~2 s y relocaliza) → **Door** → Door-Q_1 →
  Door-Q_2 → **Q** → gira a θ = 0 → `FASE I COMPLETADA` → **Ctrl+C**. A 0.08 m/s, unos 4-5 min.
- Qué mirar y apuntar: los giros en D y en E; si va pegado a la pared derecha antes de la puerta (= error de
  rumbo: pararlo); qué dice la relocalización (`Relocalización: dx dy dth` / `parcial` / `rechazada` /
  `inconsistente`); si sale `OBLIGATORIO_TAPADO`.
- Seguridad: nadie delante de E ni en la puerta; quien acompañe, detrás o a un lado a más de 0.5 m; freno de
  emergencia = interruptor de la OpenCR; no tocar el robot.
- Al terminar: Ctrl+C en el nodo y en el bag. Si se puede, medir con cinta la pose final en Q.

## 4. Análisis
Eventos del Mission Log (el último de `~/ros2_ws`):
```bash
cd ~/ros2_ws && python3 - $(ls -t mission_log_*.csv | head -1) <<'EOF'
import csv, sys, time
for a in csv.DictReader(open(sys.argv[1])):
    if a['event'] and a['event'] != 'OBSTACULO':
        print(time.strftime('%H:%M:%S', time.localtime(float(a['timestamp']))), a['state'], a['event'], a['error'],
              'pose', a['robot_x'], a['robot_y'], a['robot_yaw'])
EOF
```
Error de rumbo y de posición a lo largo de la ruta, con las paredes (scripts en `test_real/datos_20261105/`;
**tienen la fecha 2026-11-05 escrita dentro: si es otro día, cámbiala en la línea con `'2026-11-05 '`**):
```bash
cd ~/ros2_ws && source install/setup.bash
D=src/rob-autonomous-nav/test_real/datos_20261105
BAG=$(ls -td src/rob-autonomous-nav/bags/fase1_* | head -1); MCAP=$(ls $BAG/*.mcap)
LOG=$(ls -t mission_log_*.csv | head -1)
# A -> D -> E: horas de inicio, fin y LLEGADA A D (sale en los eventos: WAYPOINT D)
python3 $D/error_en_ruta.py $MCAP $LOG HH:MM:SS HH:MM:SS HH:MM:SS
# De D a la puerta (pared de la derecha): horas de inicio y fin del tramo
python3 $D/rumbo_en_ruta.py $MCAP $LOG HH:MM:SS HH:MM:SS
```
- "error rumbo" = rumbo creído − real. **Objetivo: ≤ ~1° en toda la ruta, también después de los giros de D y E.**
  Si es así, 0.08 m/s es la configuración de la demo.
- "error Y" (A→D, pared izquierda) / "error X" (D→E, bancos): error de posición en esa coordenada.
- En la ruta 6 (0.12 m/s) salió: rumbo +1.5° en B, +3° en D, +4.6-6.6° tras el giro de D.

## 5. Guardar en el repo
```bash
cd ~/ros2_ws/src/rob-autonomous-nav
R=test_real/datos_$(date +%Y%m%d); mkdir -p $R
cp ~/ros2_ws/$(ls -t ~/ros2_ws | grep mission_log | head -1) $R/
BAG=$(ls -td bags/fase1_* | head -1); tar -cf - -C bags $(basename $BAG) | zstd -T0 -19 -o $R/bag_$(basename $BAG).tar.zst
ls -la $R                       # cada .tar.zst debe pesar < 50 MB
git add -f $R && git commit -m "Ruta completa real a 0.08 m/s: Mission Log y bag" && git push
```
(El autor de git en el PC del laboratorio no está configurado: `git -c user.name="..." -c user.email="..." commit ...`.)

## Si algo va mal
| Síntoma | Causa probable |
|---|---|
| /scan, /odom o /imu con 2 publicadores, aparece /clock | Gazebo abierto en el mismo dominio: cerrarlo |
| BLOQUEO/RUTA_LIBRE continuos sin nada delante | ventana del LiDAR sucia (ecos a 12-17 cm): limpiarla |
| SIN_CAMINO cerca de E | algo en el pasillo junto a E (p. ej. una pata de silla): a los 5 s va a un punto a < 0.40 m (OBLIGATORIO_TAPADO); si no, quitarlo |
| Va pegado a la pared derecha antes de la puerta, se para | error de rumbo acumulado: parar (Ctrl+C), analizar con el paso 4 |
| Relocalización rechazada dx / dth | error mayor de 0.30 m o 3° al llegar a la puerta |
| Replanifica hacia atrás | pose muy equivocada: parar |
