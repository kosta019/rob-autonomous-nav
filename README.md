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

Necesita scipy, Pillow y matplotlib (no los nodos ROS). En el PC del laboratorio están instalados como paquetes del sistema
(comprobado el 05/11: scipy 1.11.4, Pillow 10.2.0), así que también se puede ejecutar allí. Sus salidas están en el repo.

```bash
python3 autonomous_nav/scripts/planificar_fase1.py          # genera config/fase1_ruta.yaml, maps/holgura_5cm.*, docs/ruta_fase1_plan.png
python3 -m pytest autonomous_nav/test/test_planificador.py  # compara con los valores de referencia de TAREA_FASE1.md
```

Tests sin robot de la regla de borrado, de los avisos de comprobar_pose (ecos, giróscopo) y de la relocalización (sin robot, en el PC del laboratorio, con ROS cargado):

```bash
python3 -m pytest autonomous_nav/test/test_borrado.py autonomous_nav/test/test_comprobar_pose.py autonomous_nav/test/test_reloc.py -v
```

## Mundo de Gazebo del laboratorio

Generado desde el mapa (`python3 autonomous_nav/scripts/generar_mundo_lab.py` -> `worlds/laboratorio.sdf`).
El origen de Gazebo es el del enunciado: la pose real del robot sale en coordenadas del enunciado.

```bash
ros2 launch autonomous_nav laboratorio.launch.py            # robot en initial_pose de fase1.yaml (gui:=false sin ventana)
ros2 run autonomous_nav fase1_node --ros-args --params-file $CFG/fase1.yaml \
    --params-file $CFG/fase1_ruta.yaml -p use_sim_time:=true
```

## Pruebas en Gazebo sin ventana

```bash
test_gazebo/prueba.sh p1a 100 config/test_1a.yaml                      # cuadrado de 1 m
CAJA="1.0 0.0 30" test_gazebo/prueba.sh p1c 100 config/test_1a.yaml    # caja delante, se quita a los 30 s
test_gazebo/prueba.sh ruta 300 config/fase1_ruta.yaml                  # ruta completa de la fase I

# En el mundo del laboratorio (MUNDO=lab), con la pose real y su análisis:
MUNDO=lab test_gazebo/prueba.sh lab_ruta 260 config/fase1_ruta.yaml
MUNDO=lab CAJAS="5.18 4.32 0" test_gazebo/prueba.sh p3a 320 config/fase1_ruta.yaml               # 3a: caja en D->E
MUNDO=lab CAJAS="5.18 4.32 B" test_gazebo/prueba.sh p3b 320 config/fase1_ruta.yaml               # 3b: se quita durante la espera
MUNDO=lab CAJAS="4.9 4.32 0; 5.5 4.32 0" test_gazebo/prueba.sh p3c 340 config/fase1_ruta.yaml    # 3c: paso estrecho
MUNDO=lab CAJAS="6.36 11.65 150" test_gazebo/prueba.sh p3d 380 config/fase1_ruta.yaml            # sin camino: caja en la puerta
python3 test_gazebo/analizar.py test_gazebo/resultados/p3a config/fase1_ruta.yaml                # choques, zonas, precisión real
```

Dentro del toolbox las variables se pasan con `env`: `toolbox run -c ros-jazzy env MUNDO=lab bash test_gazebo/prueba.sh ...`.
Resultados en `test_gazebo/resultados/<nombre>/` (Mission Log, `node.log`, `final.txt`, `real.csv`).
