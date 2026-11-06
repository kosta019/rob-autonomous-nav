#!/bin/bash
# Graba un rosbag con los topics de una prueba real. Parar con Ctrl+C.
# Uso:  test_real/grabar.sh
# Guarda en bags/fase1_<fecha>/ de la raíz del repo, se lance desde donde se lance (bags/ está en .gitignore).
# /joint_states: las ruedas, para comparar su rumbo con el del giróscopo. /map: si el SLAM está en marcha.
cd "$(dirname "$0")/.." || exit 1
BAG=bags/fase1_$(date +%Y%m%d_%H%M%S)
echo "Grabando en $PWD/$BAG"
exec ros2 bag record -o "$BAG" /scan /odom /imu /tf /tf_static /cmd_vel /joint_states /map
