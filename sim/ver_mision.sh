#!/bin/bash
# Abre un escenario en Gazebo CON ventana y lanza la misión completa (fases I, II y III) para verla.
# Uso:  sim/ver_mision.sh <carpeta del escenario>   (Mission Log y salida del nodo en esa carpeta)
cd "$(dirname "$0")/.." || exit 1
ESC=$(realpath "$1")
[ -d sim/ws/install ] || sim/construir.sh || exit 1
xhost +local:docker > /dev/null 2>&1
docker run --rm --network none -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD/sim":/sim:ro -v "$PWD/sim/ws/install":/ws/install:ro -v "$ESC":/esc rob-gz bash -c '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export TURTLEBOT3_MODEL=burger ROS_LOG_DIR=/tmp/roslog
  CFG=/ws/install/autonomous_nav/share/autonomous_nav/config
  read SALIDA X Y YAW <<< "$(python3 -c "import yaml; e = yaml.safe_load(open(\"/esc/escenario.yaml\")); print(e[\"salida\"], *e[\"pose_salida\"])")"
  N=$(python3 -c "import yaml; print(len(yaml.safe_load(open(\"/esc/escenario.yaml\"))[\"personas\"]))")
  EXTRA=""; [ "$SALIDA" != "A" ] && EXTRA="--params-file $CFG/salida_$SALIDA.yaml"
  ARGS="/world/laboratorio/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
  for k in $(seq 0 $((N - 1))); do ARGS="$ARGS /model/persona_$k/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"; done
  ros2 run ros_gz_bridge parameter_bridge $ARGS > /dev/null 2>&1 &
  (sleep 15; python3 /sim/nodos_sim.py /esc/escenario.yaml /tmp/real.csv /sim/sim.yaml > /dev/null 2>&1) &
  (sleep 20; /ws/install/autonomous_nav/lib/autonomous_nav/fase1_node --ros-args --params-file $CFG/fase1.yaml \
     --params-file $CFG/fase1_ruta.yaml $EXTRA -p phase2_enabled:=true -p phase3_enabled:=true -p use_sim_time:=true \
     -p log_file:=/esc/mission_log.csv > /esc/nodo.log 2>&1) &
  ros2 launch autonomous_nav laboratorio.launch.py gui:=true x:=$X y:=$Y yaw:=$YAW world:=/esc/mundo.sdf'
