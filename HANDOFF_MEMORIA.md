# Handoff: redactar la memoria técnica (sesión nueva; actualizado el 09/11/2026 al acabar el laboratorio)

Tu tarea: **redactar entera la memoria técnica del proyecto en un Google Docs**, en castellano. Marc y su compañero la
revisarán en el documento y reescribirán lo que crean necesario. Léete este fichero entero antes de empezar.

## Resultados del laboratorio del 09/11 (mandan sobre lo que se diga abajo; entrega: martes 10/11 a las 10:00)

Detalle y números: `test_real/datos_20261109/NOTAS.md`. Rama: `arreglo-fase3-lado`. Lo probado en el robot ya no va con hueco.

**La misión de la entrega es la de las 16:39** (BOT 11, fases I + II + III desde A, sin choques):
`mission_log_bot11_sin_paradas.csv`, `mapa_sin_paradas.*`. Fase I completada a 159.0 s, fase II a 432.5 s y fase III a 480.2 s.
En la fase II, 5 `BLOQUEO` por otros robots o personas y una `PARADA_EMERGENCIA` (a los 321 s, obstáculo a unos 9 cm). Estación
cuadrada en (3.42, 14.21), triangular en (4.96, 16.45). Se lanzó con `-p phase2_enabled:=true -p phase3_enabled:=true
-p reloc_pbase_times:=0`.

Qué cambia respecto al índice de abajo:
- **Localización:** en el pasillo, `CORR_PASILLO` activo con tres paredes medidas: Y 17.15 (extintor), Y 13.31 (tramo ancho, 2.39 m
  de cinta desde P Base) y una pared x = 9.54 en la franja Y 15.0-16.1 (corrige X cerca de `P2_e`).
- **Relocalización en P Base (`RELOC_P`):** implementada y probada (dos veces: la llevó a la cruz, con correcciones de hasta
  58 cm y 12°), pero **apagada en la misión de la entrega**: con ella, la fase III abandonó 2 de 2 veces (`sin_patas`). La causa no se
  ha averiguado; se descartó que fuera la estación guardada. Sin ella, P Base queda con error en X: 17-27 cm en las misiones
  medidas (no medido en la de las 16:39).
- **Rumbo:** se tuerce en la fase II entre 3.6° y 12° según la misión (BOT 11). La corrección de rumbo con paredes **no** pasó la
  validación con los bags (`rumbo_paredes_no_validado.patch`): está fuera del código. Es una limitación que hay que contar.
- **Fase I:** `phase1_pass_speed` (0.06 m/s): no para en cada punto y no se arrastra al acercarse (siguen los giros de más de 30° y las
  relocalizaciones). 12.3 s menos que la misión de las 16:22 (159.0 frente a 171.3 s).
- **Fase II:** a 0.20 m/s, no a 0.12. Las estaciones se recogen cada día y se colocan en otro sitio: el detector no se fía de las cruces.
- **Fase III:** de 5 intentos en el robot, 3 entraron y 2 abandonaron sin entrar, sin choques. En la misión de la entrega acabó
  `FASE III COMPLETADA sin_verificar patas=2` (error del aparcado **no medido**), tras dos `sin_patas` y `FASE3_CENTRO patas=3`. En la
  de las 16:22 se midieron 3.2 cm (`APARCADO_FUERA_DE_TOLERANCIA`, umbral propio de 3 cm). Se probó `FASE3_OTRO_LADO`, que elige otro
  lado cuando el primero está tapado.
- **Mapa SLAM:** `test_real/datos_20261109/mapa_sin_paradas.*` (233 × 354 celdas, 5 cm), guardado al acabar la fase II. Coherente,
  pero girado respecto a los ejes del enunciado. **No se ha podido confirmar que se vean las 4 patas:** míralo antes de afirmarlo. No
  hay figura hecha: genérala desde el `.pgm`. **Limitación que hay que contar en la memoria (decisión de Marc: se entrega tal cual):**
  la sala sale recta (pared larga a +0.3°), pero el pasillo se curva al alejarse de ella: entre −2° y +6° en el tramo cercano y entre
  +8° y +14° en la mitad lejana (medido sobre el `.pgm`, aproximado por los escalones reales de esa pared). Causa probable, sin
  comprobar: el rumbo que se tuerce en la fase II y las pocas referencias del pasillo para un LiDAR de 3.5 m. No afecta a la
  navegación, que no usa este mapa.
- **Robots:** BOT 11 (`ROS_DOMAIN_ID=11`). Parte de la tarde se usó el BOT 04, y con él el rumbo se desvió 17° en la fase II.

**Sigue como hueco** (`[PENDIENTE]` en amarillo): el error del aparcado de la misión de la entrega. **Decidido (Marc, 09/11):** el
`real.yaml` entregado deja `reloc_pbase_times` en 0, como en el vídeo; se llega a P Base con 17-27 cm de error en X (limitación).

## Qué pide el enunciado (`docs/enunciado_v2.pdf`, apartado 5, "Deliverables")

"Technical Report":
- **System Architecture Diagram:** un diagrama de bloques sencillo con los nodos y los topics de ROS 2.
- **Algorithm Description:** explicación breve de la lógica de las fases I y II.
- **Obstacle Avoidance Strategy:** cómo los datos del LiDAR modifican la ruta prevista.

Lo demás va aparte y **no** forma parte de la memoria: el paquete, el mapa (.yaml + .pgm), el Mission Log, el vídeo y la
declaración de uso de IA generativa. Puedes referenciarlos.

## Reglas de Marc (obligatorias)

1. **Fiel a la asignatura.** Antes de escribir, lee **todas** las diapositivas de `docs/teoria/` (cuatro PDF, fuera de
   git: `2_Wheeled_Mobile_Robots`, `4_RM_navigation`, `5_Sensors` y `localization_and_mapping`; usa `pdftotext -layout`) y
   la ficha de la asignatura: https://www.fib.upc.edu/ca/graus/grau-en-enginyeria-informatica/pla-destudis/assignatures/ROB
   - Explica cada técnica con los conceptos y el vocabulario de clase, por ejemplo: arquitectura híbrida
     (deliberativa + reactiva), dead reckoning, actualización del filtro de Kalman con un mapa conocido, distance
     transform, extracción de características del láser, SLAM.
   - **Sin muletillas** como "como dice la teoría", "como vimos en clase" o "según la diapositiva X". La explicación tiene
     que salir arraigada a la teoría de forma natural.
   - **No te salgas del alcance de la asignatura.** Nada de detalles internos de slam_toolbox (Ceres, grafos de poses),
     ni de Nav2, ni de técnicas que no se vean en el curso. Si algo del proyecto no está en las diapositivas (p. ej.
     RANSAC para ajustar rectas a la pared), explícalo en una frase sencilla, sin presentarlo como materia del curso.
   - Comprueba cada afirmación técnica contra las diapositivas. Los comentarios del código citan alguna diapositiva;
     **compruébalas tú**, no te fíes.
2. **Lo más breve posible sin dejar nada fuera.** Estimación: 8-10 páginas A4 (≈ 3 500-4 500 palabras), con 6-8 figuras
   y 3-4 tablas, sin portada. **Antes de crear el documento, preséntale a Marc el índice con las páginas estimadas por
   apartado y espera su visto bueno.**
3. **Google Docs**, para que el compañero lo revise. Carga la skill `anthropic-skills:google-workspace` antes de crear o
   editar el documento. Títulos con estilos de encabezado (para el índice) y figuras numeradas con pie.
4. **No inventes datos.** Todos los números salen de los ficheros de abajo. Lo que siga sin medir (ver "Sigue como hueco",
   arriba) va con un hueco visible, **resaltado en amarillo**: `[PENDIENTE: …]`. Si otro fichero dice "sin probar en el robot",
   manda lo de la sección del 09/11.
5. Redacción en primera persona del plural ("hemos elegido…"), técnica y directa. Marc estudia en la FIB: quiere que se
   vea que entienden lo que han hecho.

## Fuentes (por orden de importancia)

| Fichero | Qué tiene |
|---|---|
| `CLAUDE.md` | Contexto del proyecto, sistema de coordenadas, puntos, datos medidos con cinta, decisiones de diseño cerradas (y por qué: planificador elegido frente a Bug2, Voronoi, PRM y D*; inflado de 0.25 m; navegación propia sin Nav2) |
| `HANDOFF.md` | Estado de cada fase y de cada tarea, decisiones del 07/11 y revisión contra el enunciado |
| `README_DEMO.md` | Cómo se opera el robot y qué significa cada evento del Mission Log (sirve para describir la lógica de cada fase) |
| `test_real/datos_20261105/NOTAS.md`, `test_real/datos_20261106/NOTAS.md`, `test_real/datos_20261109/NOTAS.md` | Todas las pruebas en el robot real: qué falló, cómo se arregló y con qué números (es la "metodología"). La del 09/11 tiene las misiones completas |
| `docs/analisis_slam_20261106.md` | Por qué el SLAM no se usa para navegar (tabla), el retraso de 0.1 s del scan y el mapa entregable (sección del 07/11) |
| `docs/analisis_pasillo_20261107.md` | La corrección de la pose en el pasillo con la pared medida (tabla de variantes) |
| `autonomous_nav/autonomous_nav/fase1_node.py` | El nodo principal: máquina de estados, memoria de obstáculos, replanificación, parada de emergencia, relocalizaciones, fases II y III |
| `planificador.py`, `pilares.py`, `comprobar_pose.py`, `scan_slam.py`; `launch/slam.launch.py`; `config/*.yaml` | Planificador (distance transform con costes), detector de pilares, pose de salida, scan para el SLAM, parámetros |
| `TAREA_FASE1.md` | La tarea original de la fase I (historia del diseño) |

Figuras ya hechas en `docs/`: `ruta_fase1_plan.png` y `ruta_fase1.png` (plan de la fase I), `mapa_zonas.png`,
`pasillo_pared_20261107.png`, `mapa_slam_scan_slam_20261107.png` y `mapa_slam_estaciones_20261107.png` (mapa
entregable y patas de las estaciones), `mapas_slam_20261106.png`. Si hace falta una nueva (p. ej. el diagrama de
arquitectura o el de estados), créala con un script en `docs/` (matplotlib está en el portátil; nada de pip install) y
súbela al documento.

## Índice propuesto (ajústalo con Marc)

1. **Introducción** (½ pág.): la misión en tres fases, el robot (TurtleBot3 Burger, LDS de 3.5 m) y el enfoque: arquitectura
   híbrida, con un plan global sobre el mapa y una capa reactiva con el LiDAR que lo corrige.
2. **Arquitectura del sistema** (1 pág.): **diagrama de nodos y topics**:
   - driver del TurtleBot3: `/scan`, `/odom`, `/imu` y `/tf`, y se suscribe a `/cmd_vel`;
   - `fase1_node`: `/scan`, `/odom`, `/imu` → `/cmd_vel` (TwistStamped), más el Mission Log en CSV, los yaml de
     configuración y los mapas;
   - `scan_slam`: `/scan` → `/scan_slam`;
   - `slam_toolbox`: `/scan_slam` y `/tf` → `/map` y el `tf` map→odom;
   - `map_saver_cli`: `/map` → .pgm + .yaml.

   Ni Nav2 ni coordenadas en el código: todo en `config/`.
3. **Localización** (1-1½ pág.):
   - dead reckoning (distancia de `/odom`, rumbo del IMU);
   - pose de salida con el LiDAR contra las paredes del mapa;
   - corrección continua con el mapa de distancias (actualización tipo Kalman) en la sala;
   - relocalizaciones en la puerta y en la esquina de Q;
   - en el pasillo, corrección de la Y con la pared medida (Kalman 1D).

   Con los números de las NOTAS: el error que había y el que queda.
4. **Fase I: planificación y seguimiento** (1½ pág.):
   - mapa de ocupación → inflado → mapa de costes → distance transform (Dijkstra desde el objetivo) por tramos E → Door → Q;
   - comparación con las alternativas descartadas (CLAUDE.md);
   - simplificación del camino;
   - máquina de estados GIRAR/AVANZAR, con su control proporcional (v, w).

   Resultado: la misión de la entrega (fase I en 159.0 s, ver la sección del 09/11), más las salidas desde B y C.
5. **Evitación de obstáculos** (1 pág., parte obligatoria):
   - memoria de obstáculos (lo que el LiDAR ve y el mapa no explica, y cómo se olvida);
   - detección de bloqueo de la ruta → espera → replanificación con el obstáculo en el mapa de costes;
   - rodeo largo, punto obligatorio tapado y parada de emergencia del cono frontal.

   Ejemplos reales de las NOTAS (otros robots en la ruta, paneles alrededor de C, silla junto a E).
6. **Fase II: exploración del pasillo** (1 pág.):
   - ruta de cobertura (97 % del pasillo a ≤ 1.2 m) y segunda cobertura;
   - detector de pilares (segmentación del scan por saltos de distancia → patas → cuadrado de 44 cm o triángulo) y
     estación descartada;
   - vuelta a P Base mirando a la estación;
   - Mission Log (OBSTACULO, PILAR, PILAR_DESCARTADA).
7. **Fase III: aparcado** (½-1 pág.): delante de la estación, centro medido con las patas relativo al robot, entrada recta
   y verificación (resultados en la sección del 09/11; el error de la misión de la entrega, `[PENDIENTE]`).
8. **Mapa SLAM** (½ pág.): slam_toolbox con el scan resellado (por qué) y la figura del mapa con las patas.
9. **Pruebas y resultados** (1 pág.): tabla resumen de las pruebas en el robot real (fechas, qué se probó, resultado) y
   conclusiones.

## Al acabar

Pásale a Marc el enlace del documento y una lista corta con: los huecos `[PENDIENTE]`, lo que no has podido
comprobar contra las diapositivas y lo que has dejado fuera por brevedad. Si queda contexto, ofrece un borrador de la
declaración de uso de IA generativa (aparte, 1 página).
