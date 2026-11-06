#!/bin/bash
# Vuelve a pasar slam_toolbox (online async, como en el laboratorio) por un bag grabado, con un fichero de parámetros.
# Graba /tf y /map del SLAM nuevo en bags/slam/<nombre>_<hora>/ para evaluarlo (autonomous_nav/scripts/evaluar_slam.py).
# Cada contenedor con --network none: si no, DDS junta los de varias ejecuciones en paralelo.
# Uso:  docker/slam/slam_bag.sh <carpeta del bag en bags/> <params.yaml en docker/slam/> <nombre> [segundos (0: todo)] [retraso del scan, s]
cd "$(dirname "$0")/../.." || exit 1
NOMBRE=$3_$(date +%H%M%S)   # carpeta nueva en cada ejecución
LIMITE=${4:-0}
RETRASO=${5:-0.0}   # s que se restan al sello de /scan
mkdir -p bags/slam
docker run --rm --network none -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e NOMBRE=$NOMBRE -v "$PWD":/r:ro -v "$PWD/bags/slam":/salida rob-jazzy bash -c "
  source /opt/ros/jazzy/setup.bash
  DUR=\$(python3 -c \"import yaml; m=yaml.safe_load(open('/r/bags/$1/metadata.yaml'));
print(int(m['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9) + 5)\")
  [ $LIMITE -gt 0 ] && DUR=$LIMITE   # opcional: solo los primeros segundos (pruebas rápidas)
  python3 /r/docker/slam/filtro_tf.py --ros-args -p use_sim_time:=true -p retraso_scan:=$RETRASO &
  ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /r/docker/slam/$2 -p use_sim_time:=true \
    > /salida/\$NOMBRE.log 2>&1 &
  ros2 bag record -o /salida/\$NOMBRE /tf /map > /dev/null 2>&1 &
  REC=\$!
  sleep 4
  # En Jazzy slam_toolbox es un nodo lifecycle: el launch lo configura y activa; con ros2 run hay que hacerlo a mano
  for i in \$(seq 20); do ros2 lifecycle set /slam_toolbox configure > /dev/null 2>&1 && break; sleep 1; done
  ros2 lifecycle set /slam_toolbox activate || { echo 'slam_toolbox no arranca'; exit 1; }
  timeout \$DUR ros2 bag play /r/bags/$1 --clock --topics /scan /tf /tf_static --remap /tf:=/tf_bag /scan:=/scan_bag > /dev/null
  kill -TERM \$REC; sleep 3
"
echo "bags/slam/$NOMBRE"
