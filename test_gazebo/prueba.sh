#!/bin/bash
# Lanza una prueba de fase1_node en Gazebo (sin ventana) y guarda los resultados.
#
# Uso:  test_gazebo/prueba.sh <nombre> <segundos> [fichero.yaml ...]
#   - Los .yaml se pasan después de config/fase1.yaml y lo sobrescriben (p. ej. config/test_1a.yaml).
#     Rutas relativas a autonomous_nav/, o absolutas.
#   - MUNDO=lab: mundo del laboratorio (worlds/laboratorio.sdf, robot en initial_pose). Si no, empty_world.
#   - CAJAS="x y t; x y t": cajas de 20 cm en (x, y) de Gazebo; cada una se quita a los t s (t = 0: nunca;
#     t = B: 1 s después del primer BLOQUEO del Mission Log).
#   - A los <segundos> el nodo recibe SIGINT (como un Ctrl+C); final.txt dice si el robot queda quieto.
# Resultados en test_gazebo/resultados/<nombre>/: Mission Log, node.log, final.txt y,
# en el mundo del laboratorio, real.csv (pose REAL de Gazebo, en coordenadas del enunciado).
# Ejemplo: MUNDO=lab CAJAS="5.18 4.3 0" test_gazebo/prueba.sh p3a 300 config/fase1_ruta.yaml
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

limpiar() {
    # Cierra la simulación. A veces gz sim no atiende a SIGTERM: si sigue vivo a los 3 s, SIGKILL.
    # (Un Gazebo que se queda vivo bloquea el siguiente: el nuevo nunca publica /clock.)
    for s in TERM KILL; do
        pkill -$s -f "gz sim"; pkill -$s -f parameter_bridge; pkill -$s -f robot_state_publisher
        pkill -$s -f "ros2 launch"; pkill -$s -f "lib/autonomous_nav/fase1_node"
        sleep 3
    done
}
limpiar 2> /dev/null   # por si quedó algo de una prueba anterior

OUT=$DIR/resultados/$NOMBRE; rm -rf $OUT; mkdir -p $OUT; cd $OUT
if [ "${MUNDO:-}" = "lab" ]; then
    MUNDO_GZ=laboratorio
    ros2 launch autonomous_nav laboratorio.launch.py gui:=false > gz.log 2>&1 &
else
    MUNDO_GZ=default
    ros2 launch $DIR/empty_headless.launch.py > gz.log 2>&1 &
fi
for i in $(seq 60); do ros2 topic list 2>/dev/null | grep -q '^/scan$' && break; sleep 1; done
sleep 3

# Cajas (obstáculos que no están en el mapa)
SDF='<sdf version="1.8"><model name="NOMBRE"><static>true</static><link name="l"><collision name="c"><geometry><box><size>0.2 0.2 0.5</size></box></geometry></collision><visual name="v"><geometry><box><size>0.2 0.2 0.5</size></box></geometry><material><diffuse>0.7 0.3 0.1 1</diffuse></material></visual></link></model></sdf>'
k=0
IFS=';' read -ra LISTA <<< "${CAJAS:-}"
for c in "${LISTA[@]}"; do
    read CX CY CT <<< "$c"; [ -z "$CX" ] && continue
    k=$((k+1))
    ros2 run ros_gz_sim create -world $MUNDO_GZ -string "${SDF/NOMBRE/caja$k}" -name caja$k -x $CX -y $CY -z 0.25 >> cajas.log 2>&1
    echo "caja$k en ($CX, $CY)" >> cajas.log
    if [ "${CT:-0}" = "B" ]; then
        # Quitarla 1 s después del primer BLOQUEO del Mission Log (prueba 3b: durante la espera)
        ( until grep -qs ",BLOQUEO," mission_log_*.csv; do sleep 0.2; done; sleep 1
          gz service -s /world/$MUNDO_GZ/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
              --timeout 3000 --req "name: \"caja$k\" type: MODEL" > /dev/null 2>&1
          echo "caja$k quitada 1 s después del bloqueo" >> cajas.log ) &
    elif [ "${CT:-0}" != "0" ]; then
        ( sleep $CT
          gz service -s /world/$MUNDO_GZ/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
              --timeout 3000 --req "name: \"caja$k\" type: MODEL" > /dev/null 2>&1
          echo "caja$k quitada a los $CT s" >> cajas.log ) &
    fi
done

# Pose real del robot (solo tiene sentido en el mundo del laboratorio: origen = el del enunciado)
if [ "$MUNDO_GZ" = "laboratorio" ]; then
    echo "t_reloj,x,y,yaw" > real.csv
    ( while true; do
        gz model -m burger -p 2>/dev/null | grep -A2 "Pose" | tail -2 | tr -d '[]' | tr '\n' ' ' |
            awk -v t=$(date +%s.%N) 'NF==6 {printf "%.2f,%s,%s,%s\n", t, $1, $2, $6}' >> real.csv
      done ) &
    GT=$!
fi

# El nodo va en primer plano: en un bash no interactivo, lo lanzado con & ignora SIGINT (Ctrl+C)
timeout -s INT $T ~/ros2_ws/install/autonomous_nav/lib/autonomous_nav/fase1_node --ros-args \
    --params-file $PKG/config/fase1.yaml $EXTRA -p use_sim_time:=true > node.log 2>&1
sleep 2
[ -n "${GT:-}" ] && kill $GT
{
    echo "--- pose final según /odom de Gazebo (empieza en 0,0,0 donde aparece el robot) ---"
    timeout 10 ros2 topic echo /odom --once --qos-reliability best_effort --field pose.pose.position
    echo "--- velocidad 2 s después de parar el nodo ---"
    timeout 10 ros2 topic echo /odom --once --qos-reliability best_effort --field twist.twist.linear
} > final.txt 2>&1

limpiar 2> /dev/null
cat final.txt
grep -v "Mission Log" node.log | cut -d']' -f3-
