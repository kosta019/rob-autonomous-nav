#!/bin/bash
# Opcional: abre un escenario en Gazebo CON ventana (X11) para verlo; las personas se mueven. El robot no se mueve
# (para ver la misión con ventana, lanza fase1_node a mano en otra terminal del contenedor). SIN PROBAR con ventana
# en esta máquina (render por software, puede ir lento).
# Uso:  sim/ver.sh <carpeta del escenario>
cd "$(dirname "$0")/.." || exit 1
ESC=$(realpath "$1")
[ -d sim/ws/install ] || sim/construir.sh || exit 1
xhost +local:docker > /dev/null 2>&1
docker run --rm -it --network none -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD/sim":/sim:ro -v "$PWD/sim/ws/install":/ws/install:ro -v "$ESC":/esc:ro rob-gz bash -c '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export TURTLEBOT3_MODEL=burger ROS_LOG_DIR=/tmp/roslog
  read X Y YAW <<< "$(python3 -c "import yaml; e = yaml.safe_load(open(\"/esc/escenario.yaml\")); print(*e[\"pose_salida\"])")"
  N=$(python3 -c "import yaml; print(len(yaml.safe_load(open(\"/esc/escenario.yaml\"))[\"personas\"]))")
  ARGS="/world/laboratorio/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
  for k in $(seq 0 $((N - 1))); do ARGS="$ARGS /model/persona_$k/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"; done
  ros2 run ros_gz_bridge parameter_bridge $ARGS > /dev/null 2>&1 &
  (sleep 15; python3 /sim/nodos_sim.py /esc/escenario.yaml /tmp/real.csv /sim/sim.yaml > /dev/null 2>&1) &
  ros2 launch autonomous_nav laboratorio.launch.py gui:=true x:=$X y:=$Y yaw:=$YAW world:=/esc/mundo.sdf'
