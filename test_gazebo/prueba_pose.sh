#!/bin/bash
# Aceptación de comprobar_pose en Gazebo: el robot aparece desplazado de la pose nominal y
# comprobar_pose debe estimar su pose REAL (la de Gazebo) con poco error.
# Opcional: EXTRA_ARGS=--escribir para probar también el fichero corregido.
#
# Uso:  test_gazebo/prueba_pose.sh <nombre> <x> <y> <yaw> <dx> <dy> <dyaw_grados>
#   (x, y, yaw) = initial_pose nominal; el robot aparece en (x+dx, y+dy, yaw+dyaw).
# Resultados en test_gazebo/resultados/<nombre>/ (salida.txt, gz.log, nominal.yaml).
DIR=$(cd "$(dirname "$0")" && pwd)
PKG=$DIR/../autonomous_nav
NOMBRE=$1; X=$2; Y=$3; YAW=$4; DX=$5; DY=$6; DYAW=$7

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

limpiar() {
    # Cierra la simulación (gz sim a veces no atiende a SIGTERM: SIGKILL a los 3 s)
    for s in TERM KILL; do
        pkill -$s -f "gz sim"; pkill -$s -f parameter_bridge; pkill -$s -f robot_state_publisher
        pkill -$s -f "ros2 launch"
        sleep 3
    done
}
limpiar 2> /dev/null

OUT=$DIR/resultados/$NOMBRE; rm -rf $OUT; mkdir -p $OUT; cd $OUT
# Posición real inicial y yaml con la pose nominal
read RX RY RYAW <<< $(awk -v x=$X -v y=$Y -v w=$YAW -v dx=$DX -v dy=$DY -v dw=$DYAW \
    'BEGIN {printf "%.6f %.6f %.6f", x+dx, y+dy, w+dw*3.14159265358979/180}')
printf 'fase1_node:\n  ros__parameters:\n    initial_pose: [%.6f, %.6f, %.6f]\n' $X $Y $YAW > nominal.yaml

ros2 launch autonomous_nav laboratorio.launch.py gui:=false x:=$RX y:=$RY yaw:=$RYAW > gz.log 2>&1 &
for i in $(seq 60); do ros2 topic list 2>/dev/null | grep -q '^/scan$' && break; sleep 1; done
sleep 3

# En primer plano y con timeout (en un bash no interactivo, lo lanzado con & ignora SIGINT)
timeout -s INT 60 ros2 run autonomous_nav comprobar_pose $PKG/config/fase1.yaml $OUT/nominal.yaml ${EXTRA_ARGS:-} \
    --ros-args -p use_sim_time:=true > salida.txt 2>&1

# Pose real: tras "Pose", línea XYZ y línea RPY (el yaw es el 3er valor RPY)
REAL=$(gz model -m burger -p 2>/dev/null | grep -A2 "Pose" | tail -2 | tr -d '[]' | tr '\n' ' ')
limpiar 2> /dev/null

cat salida.txt
EST=$(grep '^POSE_ESTIMADA' salida.txt)
echo "REAL (Gazebo): $REAL"
echo "$EST $REAL" | awk -v nombre=$NOMBRE '{
    if (NF < 10) { print "RESULTADO " nombre ": FALLO (sin estimación o sin pose real)"; exit }
    dth = ($4 - $10) * 180 / 3.14159265358979
    while (dth > 180) dth -= 360; while (dth < -180) dth += 360
    dpos = sqrt(($2 - $5)^2 + ($3 - $6)^2) * 100
    if (dth < 0) dth = -dth
    printf "RESULTADO %s: error de rumbo %.3f grados, error de posición %.2f cm  -> %s\n", nombre, dth, dpos, (dth < 0.3 && dpos < 1) ? "OK" : "NO CUMPLE"
}' | tee -a salida.txt
