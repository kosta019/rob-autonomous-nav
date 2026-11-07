#!/bin/bash
# Se ejecuta DENTRO del contenedor de la misión (lo lanza mision.sh). Carpeta del escenario montada en /esc.
# Gazebo sin ventana + bridges + grabación de la pose real + fase1_node (misión completa). Termina por evento del
# Mission Log o por tiempo y cierra el nodo con SIGINT.
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_LOG_DIR=/tmp/roslog
CFG=/ws/install/autonomous_nav/share/autonomous_nav/config
cd /esc || exit 1
rm -f mission_log*.csv real.csv fin.txt tiempos.txt

read SALIDA X Y YAW <<< "$(python3 -c "import yaml; e = yaml.safe_load(open('escenario.yaml')); print(e['salida'], *e['pose_salida'])")"
read LIM_SIM LIM_RELOJ <<< "$(python3 -c "import yaml; s = yaml.safe_load(open('/sim/sim.yaml')); print(s['tiempo_limite'], s['tiempo_limite_reloj'])")"
FIN=$(python3 -c "import yaml; print('|'.join(yaml.safe_load(open('/sim/sim.yaml'))['fin_eventos']))")
NPERS=$(python3 -c "import yaml; print(len(yaml.safe_load(open('escenario.yaml'))['personas']))")
EXTRA=""
[ "$SALIDA" != "A" ] && EXTRA="--params-file $CFG/salida_$SALIDA.yaml"   # desde A no hace falta

# Gazebo (sin ventana) con el robot en la salida
ros2 launch autonomous_nav laboratorio.launch.py gui:=false x:=$X y:=$Y yaw:=$YAW world:=/esc/mundo.sdf > gz.log 2>&1 &
for i in $(seq 120); do ros2 topic list 2>/dev/null | grep -q '^/scan$' && break; sleep 1; done
timeout 30 ros2 topic echo --once /scan --qos-reliability best_effort > /dev/null 2>&1

# Pose real (todas las poses dinámicas) y velocidad de cada persona
ARGS="/world/laboratorio/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
for k in $(seq 0 $((NPERS - 1))); do ARGS="$ARGS /model/persona_$k/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"; done
ros2 run ros_gz_bridge parameter_bridge $ARGS > puentes.log 2>&1 &
sleep 3
python3 /sim/nodos_sim.py escenario.yaml real.csv /sim/sim.yaml > nodos_sim.log 2>&1 &
sleep 3

# Vigilante: cierra el nodo (SIGINT) al aparecer un evento final en el Mission Log o al pasar el tiempo simulado.
# El nodo va en primer plano: lo lanzado con & en un script no interactivo ignora SIGINT.
( while true; do
    sleep 2
    L=$(ls mission_log*.csv 2> /dev/null | head -1)
    [ -n "$L" ] || continue
    if grep -qE "$FIN" "$L"; then
        echo "EVENTO $(grep -oE "$FIN" "$L" | head -1)" > fin.txt; sleep 4
        pkill -INT -f lib/autonomous_nav/fase1_node; break
    fi
    if awk -F, -v lim=$LIM_SIM 'NR == 2 {a = $1} END {exit !(NR > 2 && $1 - a > lim)}' "$L"; then
        echo "TIEMPO_LIMITE" > fin.txt
        pkill -INT -f lib/autonomous_nav/fase1_node; break
    fi
  done ) &

T0=$(date +%s)
timeout -s INT -k 15 $LIM_RELOJ /ws/install/autonomous_nav/lib/autonomous_nav/fase1_node --ros-args \
    --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml $EXTRA \
    -p phase2_enabled:=true -p phase3_enabled:=true -p use_sim_time:=true -p log_file:=/esc/mission_log.csv > nodo.log 2>&1
T1=$(date +%s)
[ -f fin.txt ] || echo "RELOJ_LIMITE_O_NODO_TERMINADO" > fin.txt

# Tiempos: reloj, simulado y factor de tiempo real
L=$(ls mission_log*.csv | head -1)
python3 - "$L" $((T1 - T0)) << 'PY' > tiempos.txt
import csv, sys
filas = list(csv.reader(open(sys.argv[1])))[1:]
sim = float(filas[-1][0]) - float(filas[0][0]) if len(filas) > 1 else 0.0
reloj = float(sys.argv[2])
print(f'reloj_s={reloj:.0f} sim_s={sim:.1f} rtf={sim / reloj if reloj else 0:.2f}')
PY
pkill -f nodos_sim.py; pkill -f parameter_bridge; pkill -f "gz sim"; sleep 2
exit 0
