# Tarea: código de la fase I

Lee antes `CLAUDE.md`, que tiene el contexto, los datos y las reglas. La figura `docs/ruta_fase1.png` muestra el mapa con la ruta.

**Cómo avanzar:**
- Haz los pasos en orden. Cada paso acaba con algo que funciona en Gazebo.
- No empieces el siguiente hasta que Marc confirme que el anterior pasa sus pruebas.
- En cada paso: primero explica el plan, después escribe el código y al final propón las pruebas diciendo qué esperas ver.

**Lo que decides tú** (y se lo explicas a Marc con su porqué): la estructura interna de las funciones, las ganancias del controlador, los detalles de la máquina de estados y la política de memoria de obstáculos del paso 3. Los valores de `config/fase1.yaml` son puntos de partida, no verdades.

## Estructura del paquete (propuesta; el nombre se puede cambiar)

```
autonomous_nav/                  # ros2 pkg create --build-type ament_python autonomous_nav \
│                                #   --dependencies rclpy geometry_msgs sensor_msgs nav_msgs
├── autonomous_nav/
│   ├── fase1_node.py            # pasos 1 y 3
│   └── planificador.py          # funciones del Distance Transform (paso 2, reutilizadas en el 3)
├── scripts/planificar_fase1.py  # paso 2, offline: genera config/fase1_ruta.yaml (+ .npy para el paso 3)
├── config/fase1.yaml            # parámetros (ya está hecho, ver abajo)
├── maps/mapa_1cm.png
├── package.xml, setup.py, setup.cfg, resource/
```

`setup.py` tiene que instalar `config/` y `maps/` en `share/autonomous_nav/` (con `data_files`) y declarar el `entry_point` `fase1_node = autonomous_nav.fase1_node:main`.

Para ejecutar:
`ros2 run autonomous_nav fase1_node --ros-args --params-file config/fase1.yaml [--params-file config/fase1_ruta.yaml] [-p use_sim_time:=true]`
(si pasas dos ficheros de parámetros, el segundo sobrescribe al primero).

## Contexto de la fase I

**Secuencia de la ruta**, saliendo desde A:
- A → B → D → E → Door → Q → pose final.
- B y D no son obligatorios: sirven para rodear las mesas entre B y C, que no se pueden sortear. Los obligatorios son E, Door y Q, **en ese orden**.

**Las poses inicial y final las asigna el profesor en la demo.** Son parámetros. Mientras no se conozcan, usa como provisionales A (θ = 0) y Q.

**Ruta ya calculada** sobre el mapa (inflado 0.25 m, coste de holgura), en coordenadas (X, Y):
- A (0.70, 0.92) → B (3.102, 1.722) → D (5.285, 2.517) → E (5.082, 6.122)
- → pre_door (5.96, 11.66) → **Door** (6.490, 11.682) → post_door_1 (7.42, 12.36) → post_door_2 (7.96, 14.16) → **Q** (8.970, 14.497)
- Total: 18.7 m. Holgura mínima: 0.34 m, en la puerta.
- Ya está en `config/fase1.yaml`.

**Si la salida no es A, la lista cambia:**
- Desde B: quitar B.
- Desde D: quitar B y D.
- Desde C: pendiente, hasta medir las mesas.

La solución definitiva es el paso 2 con las mesas dibujadas como zona prohibida.

---

## Paso 1 · `fase1_node`: seguir waypoints, parada de emergencia y log

**Objetivo:** completar la fase I si no hay obstáculos nuevos, y pararse sin chocar si aparece uno. Todavía no rodea obstáculos; eso es el paso 3.

**Entradas y salidas**
- Suscripciones: `/odom` y `/scan`.
- Publicación: `/cmd_vel` (TwistStamped).
- Parámetros: `config/fase1.yaml`.
- Fichero: el CSV del Mission Log.
- Bucle de control en un **timer a 10 Hz**. Los callbacks solo guardan el último mensaje; no publican nada, a diferencia de la plantilla.
  - **Por qué:** así el control va a un ritmo fijo, independiente de los sensores, y se puede detectar que un sensor ha dejado de llegar.

**Requisitos**

1. **Pose en el sistema del enunciado.**
   - No supongas que `/odom` empieza en (0, 0, 0): en el robot real puede no estar a cero si se ha movido después del bringup.
   - Guarda la primera lectura (x₀, y₀, θ₀) y calcula la pose relativa a ella: el desplazamiento (dx, dy) girado −θ₀, y θ_rel = θ − θ₀.
   - Después pásala al mundo con la pose inicial (xᵢ, yᵢ, θᵢ):
     - x = xᵢ + cos θᵢ·x_rel − sin θᵢ·y_rel
     - y = yᵢ + sin θᵢ·x_rel + cos θᵢ·y_rel
     - θ = θᵢ + θ_rel (normalizado)
   - Pon todo esto en **una sola función** (p. ej. `pose_mundo()`): más adelante la pose vendrá del SLAM y solo cambiará esa función.
2. **Máquina de estados.**
   - GIRAR: girar sobre sí mismo hasta que el error de ángulo hacia el waypoint sea menor que `turn_in_place_angle`.
   - AVANZAR: control proporcional. La velocidad lineal es proporcional a la distancia, limitada a `max_linear_speed`, y corrige el ángulo. Si el error de ángulo pasa de `reorient_angle`, vuelve a GIRAR.
   - LLEGADO: cuando la distancia es menor que la tolerancia, pasar al siguiente waypoint. La tolerancia es `tol_mandatory` para E, Door y Q, y `tol_waypoint` para el resto.
   - Al acabar la lista, ir a la posición de `final_pose`. Después ORIENTAR, hasta que |θ − θ_f| < `tol_final_yaw`. Después FIN: velocidad cero, log "FASE I COMPLETADA".
   - **Por qué girar primero:** el TB3 es diferencial y no puede ir de lado. Si avanza con error de ángulo, traza arcos y se sale del tramo calculado, que es donde se midió la holgura. En la puerta solo hay 0.35 m.
3. **Parada de emergencia,** por encima de cualquier estado.
   - Si una lectura válida con |ángulo| ≤ `stop_half_angle_deg` está a menos de `stop_distance`: velocidad lineal **cero**. Se permite girar sobre sí mismo.
     - **Por qué solo la lineal:** si se bloquea también el giro y el robot queda mirando a una pared cercana, nunca podría girarse para salir.
   - Cuando el cono queda libre, el robot sigue.
   - Si no llega `/scan` durante `scan_timeout` segundos, velocidad cero total.
   - Si no llega `/odom`, lo mismo.
4. **Mission Log CSV** (`log_file`).
   - Cabecera con, como mínimo: `timestamp, current_phase, robot_x, robot_y, robot_yaw`.
   - Añade una columna de evento y otras para la posición de un obstáculo (`obstacle_x, obstacle_y`), que se usarán en la fase II.
   - Una fila cada 0.5 s, y una más en cada evento: waypoint alcanzado (con el error de posición), parada de emergencia (con la posición en el mundo de la lectura más cercana) y fin de fase.
   - `current_phase = I`. El tiempo sale del reloj del nodo, así que en Gazebo es tiempo simulado.
5. **Velocidad cero al salir**, también con Ctrl+C.

**Pruebas en Gazebo** (`empty_world`, robot en el origen de Gazebo). Para cada una, di antes qué esperas que pase:

| # | Configuración | Resultado esperado |
|---|---|---|
| 1a | `initial_pose: [0,0,0]`, waypoints (1,0), (1,1), (0,1), (0,0), pose final (0, 0, 0) | Hace un cuadrado de 1 m, pasa cada punto dentro de la tolerancia, acaba con θ ≈ 0 y escribe el CSV |
| 1b | `initial_pose: [1.0, 2.0, 1.5708]`, un único waypoint (2.0, 2.0) | En el mundo, ese punto está 1 m en +X. El robot mira a +Y, así que **gira 90° a su derecha** y avanza 1 m. En Gazebo termina en **(0, −1)**. Es la prueba de la transformación: si gira a la izquierda, hay un error de signo |
| 1c | Como 1a, con una caja insertada a 1 m delante | Se para con la lectura en ≈ 0.30 m y se queda parado; al quitar la caja, sigue |
| 1d | Robot real, en el laboratorio | Primero un tramo corto (A → B) a velocidad reducida, y después la ruta completa |

**Limitación conocida:** con solo `/odom`, a lo largo de 18.7 m la deriva puede sumar decenas de centímetros. Lo arreglará la pose del SLAM; no es tarea de este paso.

---

## Paso 2 · Planificador offline (`scripts/planificar_fase1.py` + `autonomous_nav/planificador.py`)

**Objetivo:** que la lista de waypoints salga del mapa y no de nosotros. El nodo no cambia: solo lee `config/fase1_ruta.yaml`, que genera este script.

**Algoritmo** (concreto, para poder comparar con los valores de referencia):

1. **Leer el mapa.** Leer `maps/mapa_1cm.png` (Pillow está permitido porque es offline). Negro (< 128) es ocupado. Borrar la mota de (502, 1217). Pintar como ocupadas las zonas prohibidas de la configuración (rectángulos en coordenadas del mundo).
2. **Holgura a 1 cm.** Distancia euclídea de cada píxel libre al ocupado más cercano, en metros (`scipy.ndimage.distance_transform_edt`).
3. **Rejilla de 5 cm.** La holgura de cada celda es **el mínimo** de sus 5 × 5 píxeles, que es lo conservador.
4. **Coste de cada celda.**
   - `1 + clearance_cost_gain · clip((clearance_target − c) / (clearance_target − inflation_radius), 0, 1)`.
   - Infinito si c ≤ `inflation_radius`.
   - Con los valores por defecto: `1 + 3·clip((0.60 − c)/0.35, 0, 1)`.
   - **Por qué:** el camino más corto puro va pegado al borde inflado (0.25 m del centro del robot). Este coste lo aleja de las paredes cuando hay sitio.
5. **Distance Transform con costes.**
   - Un Dijkstra (`heapq`) **desde el objetivo**, con 8 vecinos.
   - El coste de un paso es el coste de la celda destino por la longitud del paso (0.05 o 0.05·√2).
   - El resultado es la distancia al objetivo desde cada celda.
   - El camino se saca bajando desde la celda de inicio, siempre hacia el vecino con menor distancia.
6. **Inicio dentro de la zona inflada.** Si el inicio cae dentro (puede pasar: D estaba a 0.18 m de los bancos en una versión anterior), arrancar desde la celda libre más cercana.
7. **Planificar tramo a tramo** entre puntos consecutivos de la secuencia, para que E, Door y Q queden **exactos** en la ruta.
8. **Simplificar** la escalera de celdas a pocos waypoints.
   - Sustituir el primer y el último punto por las coordenadas exactas.
   - Desde el punto i, quedarse con el j más lejano tal que el segmento i–j tenga una holgura mínima (muestreada cada 1 cm en el mapa de 1 cm, con las zonas prohibidas incluidas) mayor o igual que `min(0.45, holgura mínima del camino original entre i y j) − 0.02`.
   - Repetir desde j.
9. **Salidas.**
   - `config/fase1_ruta.yaml` con `waypoint_names`, `waypoints_x` y `waypoints_y`, para el nodo `fase1_node`.
   - Un PNG de la ruta sobre el mapa, para el informe.
   - Para el paso 3: la holgura estática de la rejilla de 5 cm en `.npy`, más los metadatos necesarios (resolución y origen).

**Valores de referencia** (calculados por separado sobre el mismo mapa y con los mismos parámetros). Tu implementación debe acercarse a ellos; si se desvía, investiga antes de seguir:

| Comprobación | Valor esperado |
|---|---|
| Holgura a 1 cm (con zonas prohibidas) en A, B, C, D, E, F, G | 0.71, 1.13, 2.08, 0.58, 0.78, 0.68, 0.86 m. **Desde el 05/11, con `mesa_pequena`: E 0.43** |
| Holgura en Door, Q, R, S, T, P Base | 0.36, 0.56, 0.73, 1.36, 0.55, **0.15** m (con el suelo irregular estimado, P Base queda bloqueado con un inflado de 0.20 o más: es un problema de la fase II, no de la I) |
| Inflado 0.30 | Todos los puntos de la fase I libres |
| Inflado 0.33 | **Door bloqueado** |
| Tramos A→B, B→D, D→E | 2.53, 2.32 y 3.61 m, rectos, sin puntos intermedios. **Desde el 05/11: D→E 3.63 m con un intermedio en (5.015, 6.005), que rodea la mesa pequeña** |
| Tramo E→Door | 6.13 m, con un punto intermedio cerca de (5.96, 11.66). **Desde el 05/11, con `hoja_puerta` y `mesa_pequena`: 6.02 m, intermedios (5.115, 7.005) y (5.915, 11.455)** |
| Tramo Door→Q | 4.09 m, con puntos cerca de (7.42, 12.36) y (7.96, 14.16) |
| Total A→Q | **18.7 m**, holgura mínima ≈ 0.34 m (en la puerta) |
| Ruta directa A→E→Door→Q, sin pasar por B y D | 17.0 m (cruza las mesas; solo sirve para comparar) |
| Tiempo de un plan a 5 cm en Python | Del orden de un segundo o menos |

Tolerancia razonable frente a estas cifras: ±0.3 m en longitudes y ±0.15 m en la posición de los puntos intermedios. Las diferencias pequeñas vienen de empates entre caminos de igual coste.

Haz un test (un script o `pytest`) que compruebe estas cifras. No es una simulación del robot: es la verificación del algoritmo.

---

## Paso 3 · Replanificación con el LiDAR (dentro de `fase1_node`)

**Objetivo:** cumplir el requisito de *"real-time obstacle detection and avoidance"*. Es lo que el informe llama *"how the LiDAR data is used to override the predefined path"*.

**Requisitos**

- **En el nodo, solo `numpy`** (nada de scipy). Carga la holgura estática precalculada en el paso 2.
- **Detección.**
  - En cada ciclo, pasa las lecturas válidas a coordenadas del mundo, con la pose y el desfase aproximado del LiDAR.
  - Una lectura cuenta como **obstáculo nuevo** si en ese punto la holgura estática es mayor que un margen, p. ej. 0.15 m. Es decir: no la explica una pared del mapa ni una zona prohibida.
  - El margen hace falta porque, con la deriva de `/odom`, las paredes aparecen desplazadas.
- **Bloqueo.** El camino está bloqueado si algún obstáculo nuevo queda a menos de `inflation_radius` + un margen de la parte de la ruta que queda por delante, dentro de unos 1.5 m.
- **Reacción**, en este orden:
  1. Parar.
  2. Esperar unos 3 s, por si es un obstáculo dinámico, como una persona.
  3. Si sigue bloqueado: añadir esos puntos a una capa de obstáculos y replanificar desde la pose actual hasta el siguiente punto obligatorio, y desde ahí el resto de la secuencia.
  4. Seguir.
  - **Replanifica solo con el robot parado.** Presupuesto: menos de unos 2 s.
- **La memoria de obstáculos la decides tú,** con este problema en mente. Si solo usas el último escaneo, el robot oscila: ve la caja, replanifica, al girar deja de verla y vuelve al plan original, que la atraviesa, y así sin parar. Pero tampoco puede ser permanente, o una persona que ya se fue dejaría un obstáculo fantasma. Una opción es conservar los puntos hasta alcanzar el siguiente punto obligatorio o durante N segundos. Explica a Marc qué eliges y por qué.
- **Si no hay camino:** reintentar con un inflado menor (0.20, nunca por debajo de 0.18). Si sigue sin haber camino, esperar y reintentar cada pocos segundos. Registrarlo todo en el log.
- **La parada de emergencia del paso 1 se queda** como última capa de seguridad, independiente del planificador.
- **Log:** las posiciones en el mundo de los obstáculos nuevos, y los eventos de replanificación.

**Pruebas en Gazebo:**

| # | Escenario | Resultado esperado |
|---|---|---|
| 3a | `empty_world`, caja en mitad de un tramo | El robot se para, espera, replanifica, la rodea con holgura y llega a los waypoints |
| 3b | Igual, pero quitando la caja durante la espera | Sigue por la ruta original sin replanificar |
| 3c | Dos cajas que dejan un paso estrecho | Pasa solo si cabe con el inflado; si no, rodea o espera, pero no choca |

---

## Paso opcional · Mundo de Gazebo del laboratorio, generado desde el PNG

Solo si Marc lo pide. Hazlo con un script que genere un `.sdf` (de la versión de Gazebo que use su instalación de Jazzy):

- Las paredes como bloques estáticos, a partir de los píxeles negros agrupados en rectángulos, a 5 cm. Altura de unos 0.5 m.
- **El origen del mundo de Gazebo, igual al del enunciado:** cada bloque se coloca en sus coordenadas (X, Y). Así la pose real que da Gazebo está directamente en coordenadas del enunciado, y se puede medir la precisión.
- El suelo irregular, como parches visuales sin colisión, para ver si el robot los pisa.
- Las mesas, solo cuando estén medidas: patas como cilindros.
- Hacer aparecer el robot en la pose inicial.

Sigue siendo Gazebo, así que vale como prueba para el informe.

---

## Datos pendientes: NO los inventes

| Dato | Qué hacer mientras tanto |
|---|---|
| Poses inicial y final (se dan en la demo) | Parámetros con valores provisionales |
| Rectángulo de las mesas entre B y C | `# TODO` en la configuración. Sin ellas, la salida desde C no está resuelta |
| Medida real del suelo irregular del vestíbulo y del pasillo | Usar las estimaciones de `CLAUDE.md`, marcadas como tales |
| Ancho real de la puerta (0.75 en el mapa, 0.80 en la figura) | Usar el mapa |
| Comprobación del origen en el laboratorio (de la pared del fondo a la puerta, ≈ 6.34 m) | Dar la georreferencia por buena |
| Si se permite Nav2 o slam_toolbox | No usar Nav2 |
