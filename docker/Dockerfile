# Entorno ROS 2 Jazzy para pasar los tests y analizar bags en un portátil sin ROS (no hace falta en el laboratorio).
# Construir:  docker build -t rob-jazzy docker/
# Usar:       docker/tests.sh
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends python3-pytest python3-scipy python3-pil ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*
