#!/bin/bash
# Graba un rosbag con los topics de una prueba real. Parar con Ctrl+C.
# Uso (desde la raíz del repo):  test_real/grabar.sh
# Guarda en bags/fase1_<fecha>/ (la carpeta bags/ está en .gitignore).
ros2 bag record -o bags/fase1_$(date +%Y%m%d_%H%M%S) /scan /odom /imu /tf /tf_static /cmd_vel
