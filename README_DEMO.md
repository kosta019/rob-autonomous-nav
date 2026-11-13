# Demo: guía paso a paso (sin Claude)

Desde un PC del laboratorio recién encendido. Pruebas y porqués en `test_real/datos_20261106/NOTAS.md` y
`test_real/datos_20261109/NOTAS.md`.

**Estado (09/11, BOT 11):** misión completa I + II + III desde A sin choques (16:39: fase I a 159 s, II a 432 s, III a
480 s). Fase III: entró en 3 de 5 intentos; los otros 2 abandonaron sin entrar, también sin choques.

## 0. Preparar el PC (una vez; el PC del laboratorio pierde todo al apagarse)

El código se baja como zip, sin git. Si el `wget` falla, en GitHub **Code → Download ZIP** y moverlo a
`~/rob-autonomous-nav-main.zip` (o traerlo en un USB).
```bash
grep ROS_DOMAIN_ID ~/.bashrc                 # debe ser 11 (BOT 11); si no, editarlo
sudo apt install -y ros-jazzy-slam-toolbox   # si falta
wget -O ~/rob-autonomous-nav-main.zip https://github.com/kosta019/rob-autonomous-nav/archive/refs/heads/main.zip
mkdir -p ~/ros2_ws/src && unzip -q ~/rob-autonomous-nav-main.zip -d ~/ros2_ws/src
mv ~/ros2_ws/src/rob-autonomous-nav-main ~/ros2_ws/src/rob-autonomous-nav   # el resto de la guía usa este nombre
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

En **cada una de las 3 terminales**:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=11 TURTLEBOT3_MODEL=burger
cd ~/ros2_ws && CFG=src/rob-autonomous-nav/autonomous_nav/config
```

## 1. Robot y seguridad

1. Batería ≥ 12 V (con poca batería la OpenCR falla).
2. Encender, bringup y **no tocarlo 10 s** (calibra el giróscopo).
3. `ros2 topic hz /odom` (~20 Hz) y `ros2 topic hz /scan` (~5 Hz): el robot responde.
4. `ros2 topic info /scan` → 1 publisher · `ros2 topic info /cmd_vel` → 0 publishers. Si no, hay un Gazebo o un
   `fase1_node` viejo vivo: cerrarlo.
5. Freno de emergencia: **interruptor de la OpenCR**, siempre alguien junto al robot. Nunca dos `fase1_node` a la vez.

## 2. Bag (terminal 1) y SLAM (terminal 2), con el robot en la salida

```bash
src/rob-autonomous-nav/test_real/grabar.sh                                          # T1: debe decir "Grabando en ..."
mkdir -p ~/ros2_ws/mapas && cd ~/ros2_ws/mapas && ros2 launch autonomous_nav slam.launch.py   # T2
```

## 3. Misión completa (terminal 3)

Robot sobre la cruz de salida, **mirando a +X a ojo (±10°)**. Nadie a menos de 1 m ni delante de las paredes mientras
está parado al principio.
```bash
ros2 run autonomous_nav fase1_node --ros-args --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml \
  --params-file $CFG/real.yaml -p phase2_enabled:=true -p phase3_enabled:=true
```
- **Desde B, C o D:** añadir `--params-file $CFG/salida_C.yaml` (o `salida_B` / `salida_D`) justo después de `real.yaml`.
  Si el profesor da otro θ, cambiar `initial_pose` en ese fichero (radianes).
- **Pose final de la fase I** (por defecto, Q): añadir `-p final_pose:="[X, Y, THETA]"`. Sin probar en el robot con una
  pose distinta de Q.
- **Solo la fase I:** quitar los dos `-p phase*_enabled`.

**Al arrancar** se queda ~5 s parado corrigiendo la pose con las paredes:
- `POSE_INICIAL ...`: corregida, sale. En B y C la X se queda la de la cruz, porque los paneles tapan el fondo.
- `POSE_INICIAL_RECHAZADA <motivo>`: no se mueve y reintenta cada ~5 s.
  - `giroscopo`: reiniciar la OpenCR con el robot quieto.
  - `grande` o `residuo`: recolocar el robot.
  - `pocos_puntos` o `sin_paredes_rumbo`: apartarse de las paredes.

**Recorrido:**
1. **Fase I:** B → D → E → relocaliza junto a la puerta → Door → Q (relocaliza con la esquina) → `FASE I COMPLETADA`.
2. **Fase II:** cobertura del pasillo → `ESTACION_CUADRADA` / `ESTACION_TRIANGULAR` → P Base mirando a la estación
   → `FASE II COMPLETADA`.
3. **Fase III:** se para 0.80 m delante de la estación, mide las patas, entra recto a 0.05 m/s → `FASE III COMPLETADA`.

**Mensajes normales, no hacer nada:** `RELOC*`, `CORR_PASILLO`, `BLOQUEO`, `REPLANIFICACION`, `OBLIGATORIO_TAPADO`,
`RODEO_LARGO` (espera hasta 30 s), `SEGUNDA_COBERTURA`, `FASE3_REALINEAR`, `FASE3_OTRO_LADO`, `SALIDA_SIN_CAMINO`
(reintenta cada 5 s).

| Si pasa esto | Hacer |
|---|---|
| `RELOC_RECHAZADA` o `RELOC_INCONSISTENTE` | **Ctrl+C** antes de que cruce la puerta |
| Va pegado a una pared, a < 30 cm de algo o rozando una pata | **Interruptor de la OpenCR** |
| Oscila parado (GIRAR/AVANZAR sin avanzar) | Ctrl+C, reiniciar la OpenCR y dejarla 10 s quieta |
| `FASE_III_ABANDONADA` | Nada: no entra en la estación. Las fases I y II valen igual |

**Limitaciones conocidas:**
- El rumbo se tuerce en la fase II (3.6° a 12°).
- P Base queda con 17-27 cm de error en X.

## 4. Guardar el mapa (en cuanto salga `FASE II COMPLETADA`)

El mapa entregable es el de las fases I y II. Se guarda mientras empieza la fase III, con el SLAM aún en marcha:
```bash
cd ~/ros2_ws/mapas && ros2 run nav2_map_server map_saver_cli -f mapa_demo --ros-args -p use_sim_time:=false
```
Debe dejar `mapa_demo.pgm` y `mapa_demo.yaml`. Al acabar la misión: Ctrl+C en el nodo, el SLAM y el bag.

## 5. Llevarse los datos (antes de apagar el PC)

Sin git en el PC del laboratorio: se empaqueta todo y se sube al repo desde el portátil.
```bash
cd ~/ros2_ws/src/rob-autonomous-nav && R=datos_$(date +%Y%m%d) && mkdir -p ~/$R/mapa_slam
cp ~/ros2_ws/mission_log_*.csv ~/$R/ && cp ~/ros2_ws/mapas/mapa_demo.* ~/$R/mapa_slam/
BAG=$(ls -td bags/fase1_* | head -1); tar -cf - -C bags $(basename $BAG) | zstd -T0 -19 -o ~/$R/bag_$(basename $BAG).tar.zst
cd ~ && zip -qr $R.zip $R && ls -lh ~/$R.zip     # copiarlo a un USB o subirlo al Drive
```
En el portátil: descomprimirlo en `test_real/`, y luego `git add -f test_real/datos_<fecha>`, commit y push.

**Vídeo:** plano general del robot con el terminal del nodo a la vista. Plano cenital en Q y al aparcar en la estación.
