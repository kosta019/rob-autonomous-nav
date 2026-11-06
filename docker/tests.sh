#!/bin/bash
# Compila el paquete y pasa todos los tests dentro de la imagen rob-jazzy (ver Dockerfile), como en el laboratorio.
# El repo se monta en /ws/src; build/ e install/ quedan dentro del contenedor y se borran al salir.
cd "$(dirname "$0")/.." || exit 1
docker run --rm -v "$PWD":/ws/src/rob-autonomous-nav:ro rob-jazzy bash -c '
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  colcon build --packages-select autonomous_nav > /tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
  source install/setup.bash && cd src/rob-autonomous-nav/autonomous_nav &&
  python3 -m pytest -q -p no:cacheprovider test "$@"' -- "$@"
