#!/bin/bash
# Reproduce un bag grabado en el robot y lanza fase1_node con él (datos reales, no un simulador), dentro de rob-jazzy.
# Sirve para comprobar el nodo entero (parámetros, detector, Mission Log) sin robot. El nodo publica /cmd_vel, pero
# nadie lo escucha: su pose sale de la /odom grabada. Mission Log en bags/replay/.
# Uso:  docker/reproducir_bag.sh <carpeta del bag en bags/> <yaml (desde la raíz del repo)> [-p param:=valor ...]
# OFFSET=<s> (variable de entorno): empezar el bag a los <s> segundos.
# Ej.:  docker/reproducir_bag.sh fase1_20261106_171005 test_real/prueba_aproximacion_R.yaml
cd "$(dirname "$0")/.." || exit 1
mkdir -p bags/replay
docker run --rm --network none -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v "$PWD":/ws/src/rob-autonomous-nav:ro -v "$PWD/bags/replay":/replay rob-jazzy bash -c "
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  colcon build --packages-select autonomous_nav > /tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
  source install/setup.bash
  CFG=install/autonomous_nav/share/autonomous_nav/config
  # El bag en segundo plano y el nodo en primer plano: un proceso con & en un script ignora el Ctrl+C (SIGINT).
  # timeout le manda SIGINT al acabar el bag (+5 s) para que cierre como en el robot.
  DUR=\$(python3 -c \"import yaml; m=yaml.safe_load(open('src/rob-autonomous-nav/bags/$1/metadata.yaml'));
print(int(m['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9) + 8 - ${OFFSET:-0})\")
  (sleep 3; ros2 bag play src/rob-autonomous-nav/bags/$1 --clock --start-offset ${OFFSET:-0} --topics /scan /odom /imu > /dev/null) &
  timeout -s INT -k 5 \$DUR ros2 run autonomous_nav fase1_node --ros-args --params-file \$CFG/fase1.yaml \
    --params-file \$CFG/real.yaml --params-file src/rob-autonomous-nav/$2 -p use_sim_time:=true ${*:3} \
    -p log_file:=/replay/mission_log.csv
  echo Mission Log: bags/replay/\$(ls -t /replay | head -1)
"
