#!/bin/bash
# Pasa un bag grabado en el robot por launch/slam.launch.py (scan_slam + slam_toolbox con config/slam.yaml), tal cual se
# usará en el laboratorio, y graba /map y /tf en bags/slam/<nombre>_<hora>/ (evaluar_slam.py, mapa_slam.py).
# filtro_tf.py quita del /tf grabado el map->odom del SLAM de aquel día y reenvía /scan_bag a /scan (sin retraso: el
# resellado lo hace scan_slam).
# Uso:  docker/slam/mapa_bag.sh <carpeta del bag en bags/> <nombre>
cd "$(dirname "$0")/../.." || exit 1
NOMBRE=$2_$(date +%H%M%S)
mkdir -p bags/slam
docker run --rm --network none -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v "$PWD":/ws/src/rob-autonomous-nav:ro \
  -v "$PWD/bags/slam":/salida rob-jazzy bash -c "
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  colcon build --packages-select autonomous_nav > /tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
  source install/setup.bash
  B=src/rob-autonomous-nav/bags/$1
  DUR=\$(python3 -c \"import yaml; m=yaml.safe_load(open('\$B/metadata.yaml'));
print(int(m['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9) + 5)\")
  python3 src/rob-autonomous-nav/docker/slam/filtro_tf.py --ros-args -p use_sim_time:=true &
  ros2 launch autonomous_nav slam.launch.py use_sim_time:=true > /salida/$NOMBRE.log 2>&1 &
  ros2 bag record -o /salida/$NOMBRE /tf /map > /dev/null 2>&1 &
  REC=\$!
  sleep 8
  timeout \$DUR ros2 bag play \$B --clock --topics /scan /tf /tf_static --remap /tf:=/tf_bag /scan:=/scan_bag > /dev/null
  kill -TERM \$REC; sleep 3
"
echo "bags/slam/$NOMBRE"
