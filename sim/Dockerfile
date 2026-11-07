# Imagen para simular en Gazebo (Harmonic) el TurtleBot3 Burger sin ventana. Parte de rob-jazzy.
# Construir:  docker build -t rob-gz sim/      (rob-jazzy antes: docker build -t rob-jazzy docker/)
FROM rob-jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-turtlebot3-gazebo ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
        ros-jazzy-turtlebot3-description python3-matplotlib python3-numpy python3-yaml \
        libgl1-mesa-dri libegl1 mesa-utils xvfb \
    && rm -rf /var/lib/apt/lists/*
ENV TURTLEBOT3_MODEL=burger
