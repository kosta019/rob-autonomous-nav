#!/bin/bash
# Batería de misiones en Gazebo con obstáculos aleatorios.
# Uso:  sim/bateria.sh N [paralelo]      (paralelo: misiones a la vez; por defecto nproc/5, entre 1 y 3)
#       SEMILLA=<n> (variable de entorno): primera semilla (por defecto 1000). Las semillas son consecutivas; las que
#       no tienen camino posible se saltan.
#       FORZAR="tranquila:A exigente:C:persona" (prueba): un escenario por cada dificultad:salida[:persona].
# Resultados en sim/resultados/<fecha_hora>/<semilla>/ y progreso.txt en esa carpeta. Después: sim/analizar.py <carpeta>.
# Un escenario que falle no para la batería. Ctrl+C la para y mata los contenedores.
cd "$(dirname "$0")/.." || exit 1
N=${1:?Uso: sim/bateria.sh N [paralelo]}
P=${2:-$(( $(nproc) / 5 ))}
[ "$P" -lt 1 ] && P=1
[ "$P" -gt 3 ] && P=3
OUT=sim/resultados/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
OUT=$(realpath "$OUT")
LIM=$(python3 -c "import yaml; print(int(yaml.safe_load(open('sim/sim.yaml'))['tiempo_limite_reloj']) + 300)")

sim/construir.sh || exit 1
if [ -n "$FORZAR" ]; then
    # Para pruebas: un escenario por cada dificultad:salida[:persona] de FORZAR (N se ignora)
    N=0
    for f in $FORZAR; do
        python3 sim/escenarios.py --n 1 --semilla "$((${SEMILLA:-1000} + N * 1000))" --salida "$OUT" --forzar "$f" \
            2>> "$OUT/generacion.log" >> "$OUT/escenarios.txt" || exit 1
        N=$((N + 1))
    done
else
    python3 sim/escenarios.py --n "$N" --semilla "${SEMILLA:-1000}" --salida "$OUT" 2> "$OUT/generacion.log" > "$OUT/escenarios.txt" || exit 1
fi
echo "Batería en $OUT: $N misiones, $P en paralelo"
echo "$(date +%T) inicio: $N misiones, $P en paralelo" > "$OUT/progreso.txt"

export OUT N LIM
una() {
    # Una misión; cualquier fallo se anota y no se propaga
    d=$1
    timeout -k 20 "$LIM" sim/mision.sh "$d" > "$d/mision.log" 2>&1
    hechas=$(grep -c "^[0-9:]* fin" "$OUT/progreso.txt")
    echo "$(date +%T) fin $(basename "$d") ($((hechas + 1))/$N): $(tail -1 "$d/mision.log")" >> "$OUT/progreso.txt"
}
export -f una
ls -d "$OUT"/*/ | sed 's#/$##' | xargs -P "$P" -I{} bash -c 'una {}' || true
echo "$(date +%T) batería terminada" >> "$OUT/progreso.txt"
echo "Hecho. Análisis:  python3 sim/analizar.py $OUT"
