#!/bin/bash
# Monta el zip de la entrega: el paquete ROS 2 (solo lo esencial, tal como está en el último commit) y los entregables
# que se dejan en esta carpeta. Antes de comprimir, compila el paquete con colcon (imagen Docker rob-jazzy) para
# comprobar que funciona solo con lo que va en el zip.
# Uso (desde la raíz del repo):  entrega/crear_zip.sh
# Entregables que hay que dejar en entrega/ antes:
#   memoria.pdf, declaracion_IA.pdf, mission_log.csv, video.mp4 y mapa/ (el .pgm y el .yaml del SLAM)
cd "$(dirname "$0")/.." || exit 1
DIR=$(mktemp -d)
P=autonomous_nav
PAQUETE="$P/package.xml $P/setup.py $P/setup.cfg $P/resource
  $P/autonomous_nav/__init__.py $P/autonomous_nav/fase1_node.py $P/autonomous_nav/planificador.py
  $P/autonomous_nav/pilares.py $P/autonomous_nav/comprobar_pose.py $P/autonomous_nav/scan_slam.py
  $P/config/fase1.yaml $P/config/fase1_ruta.yaml $P/config/real.yaml $P/config/slam.yaml
  $P/config/salida_B.yaml $P/config/salida_C.yaml $P/config/salida_D.yaml
  $P/launch/slam.launch.py $P/maps
  $P/scripts/planificar_fase1.py $P/scripts/generar_mapa_distancias.py"
git diff --quiet HEAD -- $PAQUETE || echo "AVISO: hay cambios sin commit en el paquete; el zip lleva lo del último commit"
git archive HEAD $PAQUETE | tar -x -C "$DIR" || exit 1

# Entregables (no están en git)
FALTAN=""
for f in memoria.pdf declaracion_IA.pdf mission_log.csv video.mp4 mapa; do
  if [ -e "entrega/$f" ]; then cp -r "entrega/$f" "$DIR/"; else FALTAN="$FALTAN $f"; fi
done
[ -n "$FALTAN" ] && echo "AVISO: faltan en entrega/:$FALTAN"

# El paquete tiene que compilar y dar sus ejecutables solo con lo que va en el zip
docker run --rm --network none -v "$DIR/$P":/ws/src/$P:ro rob-jazzy bash -c '
  source /opt/ros/jazzy/setup.bash && cd /ws && colcon build > /tmp/b.log 2>&1 || { cat /tmp/b.log; exit 1; }
  source install/setup.bash && ros2 pkg executables autonomous_nav &&
  ls install/autonomous_nav/share/autonomous_nav/config install/autonomous_nav/share/autonomous_nav/maps' || {
  echo "ERROR: el paquete no compila con lo que va en el zip"; exit 1; }

rm -f entrega/entrega_ROB.zip
(cd "$DIR" && zip -qr - .) > entrega/entrega_ROB.zip
echo "entrega/entrega_ROB.zip: $(du -h entrega/entrega_ROB.zip | cut -f1)"
unzip -l entrega/entrega_ROB.zip | tail -1
rm -rf "$DIR"
