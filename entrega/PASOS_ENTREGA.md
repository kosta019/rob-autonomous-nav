# Pasos hasta la entrega

Marcar cada paso al hacerlo. Rama para el laboratorio: `arreglo-fase3-lado` (= `fase2-pilares` + corrección de la fase III
+ simulación). Si la corrección diera problemas en el robot: `-p phase3_side_wait:=0.0`.

## Antes del lunes (domingo)

- [ ] Revisar y fusionar `fase2-pilares` en `main` (todo lo de la semana menos la corrección de la fase III).
- [ ] Añadir el nombre del compañero en `autonomous_nav/LICENSE` (ahora solo "Marc Costa").
- [ ] Borrar `entrega/entrega_ROB/` (zip descomprimido de prueba; colcon veía el paquete dos veces).
- [ ] Memoria: que la sesión de la memoria tenga ya los números de las dos baterías de Gazebo y deje bien marcados los
      `[PENDIENTE 09/11]`.
- [ ] Preguntar la fecha y la hora límite de la entrega y dónde se sube (Atenea).

## Lunes en el laboratorio

0. [ ] Preparar el PC (README_DEMO 1): `git pull`, `git checkout arreglo-fase3-lado`, `colcon build`, `ROS_DOMAIN_ID=11`.
       Comprobar que están slam_toolbox y nav2_map_server. `docker/tests.sh` no hace falta allí.
1. [ ] Preguntas al profesor al llegar: si vale slam_toolbox, la pose final de la fase I y su θ, si moverá las
       estaciones y los paneles, la tolerancia del aparcado y si el vídeo puede ir como enlace.
2. [ ] Misión completa con bag y SLAM (fases I, II y III). Comprobar en el Mission Log `POSE_INICIAL` (≈ lo que daba
       `comprobar_pose`), `CORR_PASILLO` solo en la fase II, la estación, `THETA_HACIA_ESTACION` y la fase III.
3. [ ] Medidas con cinta:
       - de la cruz de P Base a la pared del tramo ancho (predicción ≈ 2.41 m) → en `real.yaml`, `corridor_wall_*`;
       - el extremo X −0.55 (distancia de P1_o o de P Base a esa pared) → otra pared `x` en `corridor_wall_*`;
       - las mesas entre B y C (cuatro esquinas respecto a las paredes o a las cruces de B y C) → zona prohibida en
         `fase1.yaml` y, en el portátil (necesita scipy), `python3 scripts/planificar_fase1.py`.
4. [ ] Fase III, sola o en una misión completa: entrada normal y con un panel delante del lado de R que da a P Base
       (`sim/fallos/fase3_lado_tapado/README.md`). Alguien junto a la estación con el interruptor de la OpenCR.
5. [ ] Si se han cambiado yaml con las medidas: una misión de comprobación antes del vídeo.
6. [ ] **Vídeo definitivo** desde A, misión completa, con bag y SLAM en marcha (pasillo a 0.12 m/s). Al salir
       `FASE II COMPLETADA`: guardar el mapa con `map_saver_cli -f mapa_fases_I_II`.
7. [ ] Guardar todo (README_DEMO 7): Mission Logs, bags comprimidos, mapa y fotos en `test_real/datos_20261109/`, con
       una `NOTAS.md` corta. Commit y push.
8. [ ] Si la corrección de la fase III ha ido bien: fusionarla en `main`. Si no: anotarlo y entregar sin ella (`main`).

## Después del laboratorio

9. [ ] Elegir la ejecución buena (la del vídeo) y copiar a `entrega/`:
       - `mission_log.csv` (el `mission_log_<fecha>.csv` de esa ejecución, renombrado);
       - `mapa/mapa_fases_I_II.pgm` y `mapa/mapa_fases_I_II.yaml`;
       - `video.mp4` (recortado si hace falta; si pesa demasiado para Atenea, subirlo y poner el enlace en la memoria).
10. [ ] Memoria:
       - rellenar los `[PENDIENTE 09/11]` con los resultados del lunes (fase III, corrección del pasillo, pose de salida,
         medidas nuevas);
       - añadir la figura del mapa SLAM de la ejecución buena;
       - revisión del compañero en el Google Docs y resolver sus comentarios;
       - comprobar que tiene lo que pide el enunciado: diagrama de nodos y topics, lógica de las fases I y II, estrategia de
         evitación;
       - exportar a PDF → `entrega/memoria.pdf`.
11. [ ] Declaración de uso de IA generativa → `entrega/declaracion_IA.pdf` (qué herramientas, para qué y qué ha revisado
        el equipo; firmada si lo piden).
12. [ ] Código final en `main` (con las medidas nuevas si las hay), tests en el portátil (`docker/tests.sh`), commit y push.
13. [ ] `entrega/crear_zip.sh` desde `main`: no debe dar AVISOS ni ERROR (compila el paquete solo con lo que va en el zip).
14. [ ] Revisar el zip: `unzip -l entrega/entrega_ROB.zip` (paquete, memoria, declaración, mapa, Mission Log, vídeo) y
        abrir la memoria y el vídeo desde dentro del zip.
15. [ ] Renombrar el zip si el profesor pide un formato (grupo, nombres) y subirlo a Atenea antes de la hora límite.
16. [ ] `git tag entrega && git push --tags` para tener marcado exactamente lo entregado.
