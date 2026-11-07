#!/bin/bash
# Compila el paquete autonomous_nav una vez dentro de rob-gz; build/ e install/ quedan en sim/ws/ (no va a git)
# y todas las misiones lo reutilizan. Se vuelve a ejecutar solo si cambia el paquete.
cd "$(dirname "$0")/.." || exit 1
mkdir -p sim/ws
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/ws/src/rob-autonomous-nav:ro -v "$PWD/sim/ws":/ws/out -w /ws/out rob-gz bash -c '
  source /opt/ros/jazzy/setup.bash &&
  colcon build --base-paths /ws/src/rob-autonomous-nav/autonomous_nav --packages-select autonomous_nav --build-base /ws/out/build --install-base /ws/out/install > /ws/out/build.log 2>&1 \
    || { cat /ws/out/build.log; exit 1; }'
