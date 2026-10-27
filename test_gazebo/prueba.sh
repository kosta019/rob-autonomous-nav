#!/bin/bash
# Lanza una prueba de fase1_node en Gazebo (empty_world, sin ventana) y guarda los resultados.
#
# Uso:  test_gazebo/prueba.sh <nombre> <segundos> [fichero.yaml ...]
#   - Los .yaml se pasan después de config/fase1.yaml y lo sobrescriben (p. ej. config/test_1a.yaml).
#     Rutas relativas a autonomous_nav/, o absolutas.
#   - Opcional: CAJA="x y t" pone una caja de 20 cm en (x, y) de Gazebo y la quita a los t segundos.
#   - A los <segundos> el nodo recibe SIGINT (como un Ctrl+C); final.txt dice si el robot queda quieto.
# Resultados en test_gazebo/resultados/<nombre>/: CSV del Mission Log, node.log y pose final.
# Ejemplo: CAJA="1.0 0.0 30" test_gazebo/prueba.sh p1c 90 config/test_1a.yaml
DIR=$(cd "$(dirname "$0")" && pwd)
PKG=$DIR/../autonomous_nav
NOMBRE=$1; T=$2; shift 2
EXTRA=""
for f in "$@"; do
    case $f in /*) ;; *) f=$PKG/$f ;; esac
    EXTRA="$EXTRA --params-file $(realpath $f)"
done

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}   # aislado de otros robots de la red

OUT=$DIR/resultados/$NOMBRE; rm -rf $OUT; mkdir -p $OUT; cd $OUT
ros2 launch $DIR/empty_headless.launch.py > gz.log 2>&1 &
for i in $(seq 60); do ros2 topic list 2>/dev/null | grep -q '^/scan$' && break; sleep 1; done
sleep 3

if [ -n "${CAJA:-}" ]; then
    read CX CY CT <<< "$CAJA"
    SDF='<sdf version="1.8"><model name="caja"><static>true</static><link name="l"><collision name="c"><geometry><box><size>0.2 0.2 0.5</size></box></geometry></collision><visual name="v"><geometry><box><size>0.2 0.2 0.5</size></box></geometry></visual></link></model></sdf>'
    ros2 run ros_gz_sim create -world default -string "$SDF" -name caja -x $CX -y $CY -z 0.25 > caja.log 2>&1
    ( sleep $CT
      gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
          --timeout 3000 --req 'name: "caja" type: MODEL' >> caja.log 2>&1
      echo "caja quitada a los $CT s" >> caja.log ) &
fi

# El nodo va en primer plano: en un bash no interactivo, lo lanzado con & ignora SIGINT (Ctrl+C)
timeout -s INT $T ~/ros2_ws/install/autonomous_nav/lib/autonomous_nav/fase1_node --ros-args \
    --params-file $PKG/config/fase1.yaml $EXTRA -p use_sim_time:=true > node.log 2>&1
sleep 2
{
    echo "--- pose final en Gazebo (/odom de Gazebo, que empieza en 0,0,0) ---"
    timeout 10 ros2 topic echo /odom --once --qos-reliability best_effort --field pose.pose
    echo "--- velocidad 2 s después de parar el nodo ---"
    timeout 10 ros2 topic echo /odom --once --qos-reliability best_effort --field twist.twist.linear
} > final.txt 2>&1

pkill -f "gz sim"; pkill -f parameter_bridge; pkill -f robot_state_publisher; pkill -f "ros2 launch"
cat final.txt
grep -v "Mission Log" node.log | cut -d']' -f3-
