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
