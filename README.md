# ROB · Autonomous Navigation (TurtleBot3 Burger, ROS 2 Jazzy)

Contexto del proyecto en `CLAUDE.md`; tarea actual en `TAREA_FASE1.md`. El paquete ROS 2 es `autonomous_nav/`.

## Montar el workspace (PC del laboratorio, desde cero)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:kosta019/rob-autonomous-nav.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Con `--symlink-install`, editar `autonomous_nav/config/*.yaml` no requiere recompilar.

## Fase I

```bash
export TURTLEBOT3_MODEL=burger
CFG=~/ros2_ws/src/rob-autonomous-nav/autonomous_nav/config

# Robot real
ros2 run autonomous_nav fase1_node --ros-args --params-file $CFG/fase1.yaml

# Gazebo (otra terminal: ros2 launch turtlebot3_gazebo empty_world.launch.py)
ros2 run autonomous_nav fase1_node --ros-args --params-file $CFG/fase1.yaml \
    --params-file $CFG/test_1a.yaml -p use_sim_time:=true
```

El Mission Log (`mission_log_<fecha>.csv`) se escribe en la carpeta desde la que se lanza el nodo.

Con la ruta generada por el planificador (paso 2), añade `--params-file $CFG/fase1_ruta.yaml` después de `fase1.yaml`.

## Planificador (paso 2, offline en el portátil)

Necesita scipy, Pillow y matplotlib, así que **no** se ejecuta en el PC del laboratorio: sus salidas están en el repo.

```bash
python3 autonomous_nav/scripts/planificar_fase1.py          # genera config/fase1_ruta.yaml, maps/holgura_5cm.*, docs/ruta_fase1_plan.png
python3 -m pytest autonomous_nav/test/test_planificador.py  # compara con los valores de referencia de TAREA_FASE1.md
```

## Pruebas en Gazebo sin ventana

```bash
test_gazebo/prueba.sh p1a 100 config/test_1a.yaml                      # cuadrado de 1 m
CAJA="1.0 0.0 30" test_gazebo/prueba.sh p1c 100 config/test_1a.yaml    # caja delante, se quita a los 30 s
test_gazebo/prueba.sh ruta 300 config/fase1_ruta.yaml                  # ruta completa de la fase I
```

Resultados en `test_gazebo/resultados/<nombre>/` (Mission Log, `node.log`, pose final en Gazebo).
