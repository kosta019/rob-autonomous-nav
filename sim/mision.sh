#!/bin/bash
# Una misión completa (fases I, II y III) en Gazebo sin ventana, en un contenedor aislado (como docker/).
# Uso:  sim/mision.sh <carpeta del escenario>   (la que crea sim/escenarios.py: escenario.yaml + mundo.sdf)
# Deja en la carpeta: mission_log_*.csv, nodo.log, gz.log, real.csv (pose real de Gazebo), fin.txt, tiempos.txt.
cd "$(dirname "$0")/.." || exit 1
ESC=$(realpath "$1")
[ -f "$ESC/escenario.yaml" ] || { echo "No hay escenario.yaml en $ESC"; exit 1; }
[ -d sim/ws/install ] || sim/construir.sh || exit 1
NOMBRE="rob-gz-$(basename "$(dirname "$ESC")")-$(basename "$ESC")-$$"
trap 'docker kill "$NOMBRE" > /dev/null 2>&1' INT TERM
# --network none + LOCALHOST: varias misiones en paralelo sin mezclar sus topics
docker run --rm --name "$NOMBRE" --network none -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD/sim":/sim:ro -v "$PWD/sim/ws/install":/ws/install:ro -v "$ESC":/esc \
    rob-gz bash /sim/dentro.sh > "$ESC/contenedor.log" 2>&1 &
wait $!
cat "$ESC/fin.txt" "$ESC/tiempos.txt" 2> /dev/null | tr '\n' ' '; echo
