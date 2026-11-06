# Pruebas reales del 06/11/2026 (BOT 11): corrección con el mapa, obligatorio tapado y puerta

Parche `correccion_mapa.patch` aplicado (095a07d) y dos arreglos tras las pruebas (e858ebf, cdb4f32).
`real.yaml` queda en `map_corr_mode: "activo"`. Todo a 0.08 m/s.

## Para retomar (otro PC del laboratorio)

1. Instalación: `install_ros2_turtlebot.sh` escribe `ROS_DOMAIN_ID=30` en `~/.bashrc`. BOT 11 es el **11**: cambiarlo.
   No instala `slam_toolbox`: `sudo apt install -y ros-jazzy-slam-toolbox`. Si `apt` espera un bloqueo, son las
   actualizaciones automáticas de Ubuntu: esperar (no matar `dpkg`).
2. `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone https://github.com/kosta019/rob-autonomous-nav.git`,
   `cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash`. Tests: 76 passed, 1 xfailed
   (con el paquete instalado y cargado; sin `source install/setup.bash` fallan 2 de `test_comprobar_pose.py`).
3. **Batería y OpenCR antes de nada** (ver "Fallo de la OpenCR"). Bringup, 10 s sin tocar el robot.
4. Guía `test_real/GUIA_RUTA_COMPLETA.md` tal cual.

## Medidas con cinta (Marc, desde la cara de la canaleta, que está en toda la pared de abajo)

- **Escalón de la pared de abajo → final de la canaleta junto a la puerta: 4.15 m.** Mapa: 3.84 m (7.30 → 11.14).
  LiDAR: ~4.2 m. **El mapa está mal ~0.31 m en Y en ese tramo.** Es el error de Y que llega a la puerta (ver abajo).
  Sin decidir qué extremo está mal en el mapa; el robot, por odometría desde A, ve la puerta ~0.3-0.37 m más allá en
  Y de lo que dice el mapa, así que lo probable es que el tramo del escalón a la puerta esté corto en el mapa.
- **Mesas que no están en el mapa** (bordes estimados, X):
  - entre D y E: 155 cm desde la canaleta (X ≈ 5.82) → X ≈ 4.28;
  - delante de E: 123 cm desde el saliente (la `mesa_pequena`, borde X 5.51) → X ≈ 4.28;
  - del escalón a la puerta (cubren el hueco, hasta la pared derecha): 147 cm desde la canaleta (X ≈ 6.34) → X ≈ 4.87.
  Quedan a ≥ 0.3 m de la ruta: no la bloquean.
- `reloc_duct_end_y` (11.14) **no se ha calibrado hoy**: el valor ya salía de la vista en vivo del 05/11 con el robot
  en pose de cinta en (5.96, 11.66, 90°) (`datos_20261105/NOTAS.md`, línea 173), la misma pose que pedía la calibración.

## Rutas (bag `bag_fase1_20261106_131026.tar.zst`, uno solo para las tres; /scan /odom /imu /tf /cmd_vel /joint_states)

`comprobar_pose --escribir` antes de cada una, sin avisos: (0.709, 0.915, 0.57°), (0.705, 0.920, 1.04°),
(0.704, 0.915, 1.65°).

### 1. 13:15, modo sombra, A → E (`mission_log_ruta_sombra.csv`)

Cumple el criterio del encargo para pasar a activo (n > 40 y evolución gradual):

| Punto | Encargo (bags 05/11) | 06/11 |
|---|---|---|
| Cerca de B | dy −0.03/−0.06, dth 0/−0.5°, n ~80 | dy −0.05, dth +0.2°, n 93 |
| Llegando a D | dy −0.10/−0.17, dth −0.4/−2.2°, n ~100 | dy −0.09/−0.12, dth −0.5/−0.8°, n ~102 |
| Tras D, Y ≈ 4 | dy −0.13/−0.19, dth −0.9/−3.5°, n ~86 | dy −0.05/−0.09, dth −1.5/−1.8°, n 81-86 |

- Sin corregir, el rumbo deriva de D a E hasta −3.7° y la X 14 cm (lo mismo que el 05/11).
- dy = 0.000 desde Y ≈ 4.7: solo se ve la pared de abajo (normal en X); lo no observado no se toca, como se diseñó.
- Al salir de A, dth +1.1/+1.4° que baja a ~0 en B: la diferencia de 1.76° entre la pared del fondo y la izquierda.
- **Una silla ocupada junto a E** (`foto_E_silla.jpg`, pata a 0.15 m de la cruz): bucle BLOQUEO/REPLANIFICACION sin
  fin, robot parado (seguro, pero no acaba). Causa y arreglo: e858ebf (abajo). Ctrl+C.

### 2. 13:40, modo activo, A → puerta (`mission_log_ruta_activo_silla_puerta.csv`)

- **Corrección con el mapa activa: funciona.** |dx|, |dy| ≲ 3 cm y |dth| < 1° casi siempre; sin deriva de rumbo de D a
  E y sin bloqueos contra la pared derecha. n 80-120 lejos de E, 36-50 junto a E (la silla y la profe tapan).
- **Obligatorio tapado con el arreglo e858ebf: funciona.** BLOQUEO → SIN_CAMINO → a los 5 s E~ → `OBLIGATORIO_TAPADO E
  0.383` → replan con inflado 0.25 y rodeo de la silla por el lado de las mesas.
- **Bloqueado 0.21 m antes del último waypoint antes de Door** (`foto_puerta_bloqueo.jpg`): BLOQUEO en (6.323, 11.512)
  y SIN_CAMINO sin fin. La Y llega con **+0.37 m**: con la pose creída, la canaleta acaba en Y ≈ 11.50 (calibrada
  11.14) y la hoja de la puerta sale en ~12.33 (12.01-12.06). El lateral del marco cae dentro del hueco del mapa
  (holgura estática 0.22 > `obstacle_static_margin` 0.20) y cuenta como obstáculo; el hueco se estrecha y no hay
  camino a Door (que no se sustituye). La relocalización, que lo corrige, nunca llegaba. Arreglo: cdb4f32 (abajo).
- Con los scans de ese momento, `ajuste_puerta` da `total` (Y=canaleta): dy −0.36/−0.38, dx +0.013, dth −0.7°, y ~0
  al repetir, en 5 de 5 ventanas (`reloc_bloqueo.py`). El respaldo de la canaleta del parche funciona desde ahí.

### 3. 13:55, modo activo, con la relocalización adelantada (`mission_log_ruta_reloc_adelantada_fallo_odom.csv`)

- Hasta E, como la 2. La silla seguía: E~ → `OBLIGATORIO_TAPADO E 0.352`. Entre la silla y la mesa pequeña hay ~0.29
  m (no cabe con el inflado): el rodeo va por el otro lado de la profe, largo (hasta X ≈ 4.07). `PARADA_EMERGENCIA`
  correcta junto a la mesa pequeña (5.38, 5.82).
- **13:58:18 fallo de la OpenCR**: /odom salta 11 km ((9.08, 21.86) → (2440.9, −11175.6)) y 0.8 s después /odom e
  /imu dejan de publicarse; /scan sigue a 5 Hz. El nodo: `TIMEOUT_SENSOR`, velocidad cero. Robot en (4.77, 6.78),
  aún rodeando. **La relocalización adelantada no llegó a probarse en el robot.** (`salto_odom.py`)
- Sospecha: batería baja (casi 1 h de pruebas) o cable USB Raspberry-OpenCR. Sin comprobar.

## Cambios en el código

- **e858ebf `ruta_bloqueada` solo hasta el siguiente obligatorio** (o su sustituto, o la pose final). Antes, con el
  robot a 8 cm de E~, los 0.40 m de `block_lookahead` entraban en el tramo E~ → E-Door_1, sin replanificar, que pasaba
  a 0.236 m de la silla; el replan solo llega al obligatorio, así que nunca cambiaba. Reconstruido con el bag
  (`bucle_E_tiempo.py`: mismo inflado 0.20 y mismo obstáculo que el log). Desde E~ hay camino a Door con inflado 0.25,
  a 0.31 m de la silla (`rodeo_E.py`). Comprobado en la ruta 2.
- **cdb4f32 relocalización adelantada**: un BLOQUEO yendo al último waypoint antes de `reloc_before`, a menos de
  `reloc_on_block_dist` (0.50 m) de él, relocaliza ahí (una vez) en vez de esperar; la relocalización ya borra los
  obstáculos. Evento `RELOC_ADELANTADA`. **Sin probar en el robot.**
- Tests nuevos en `test/test_bloqueo.py` (el primero falla con el código anterior).

## Pendiente, por orden

1. Batería/OpenCR; comprobar /odom e /imu (`ros2 topic hz`) y 10 s de deriva antes de salir.
2. **Ruta completa con E libre** (pedir que nadie esté junto a E): ver `RELOC_ADELANTADA` + `RELOC ... dy≈-0.3/-0.4
   Y=canaleta`, una segunda RELOC ~0 en el último waypoint, y **Door → Q por primera vez**. Ctrl+C antes de la puerta
   si `RELOC_PARCIAL`, `RELOC_RECHAZADA` o `RELOC_INCONSISTENTE`.
3. Si va bien: `max_linear_speed` 0.12 (solo ese cambio) y otra ruta completa. Vigilar vueltas en los obligatorios
   (`tol_mandatory` 0.02 → 0.03 si pasa).
4. Arreglar el mapa (~0.31 m en Y entre el escalón y la puerta) y regenerar holgura y distancias. Después de la demo:
   la relocalización ya lo compensa.
5. El rodeo de un obligatorio tapado puede ser muy largo si el sustituto queda encajonado (silla + mesa pequeña).
   Aceptable; en la demo nadie se sienta en E.

## Scripts (desde la raíz del repo, con ROS cargado; los tiempos van en segundos epoch dentro de cada script)

Descomprimir: `mkdir -p bags && tar -C bags -I zstd -xf test_real/datos_20261106/bag_fase1_20261106_131026.tar.zst`.
MCAP=`bags/fase1_20261106_131026/fase1_20261106_131026_0.mcap`.

- `bucle_E_tiempo.py LOG MCAP` (LOG = `mission_log_ruta_sombra.csv`): memoria de obstáculos y decisiones del bucle en E.
- `rodeo_E.py MCAP`: camino de E~ a Door con los obstáculos del bag.
- `puerta_amplia.py MCAP`: rejilla de puntos del LiDAR junto a la puerta con la pose creída (ruta 2).
- `reloc_bloqueo.py MCAP`: `ajuste_puerta` desde donde se bloqueó la ruta 2.
- `salto_odom.py MCAP`: /odom, /scan e /imu alrededor del fallo de la OpenCR (ruta 3).
- `../corr_mapa_log.py LOG`: resumen de las correcciones con el mapa.

## Tarde (otro PC, batería nueva): primeras rutas completas A → Q

Batería cambiada y robot reiniciado; /odom e /imu a 20 Hz, /scan a 5 Hz, 12.06 V. `comprobar_pose` sin avisos en
todas: (0.682, 0.919, 0.16°) y (0.681, 0.919, 0.68°). E libre salvo donde se dice.

| Ruta | Hora | Velocidad | Resultado | Log | Bag |
|---|---|---|---|---|---|
| 4 | 14:25 | 0.08 | **A → Q en 4:39**, `FASE I COMPLETADA` | `mission_log_ruta_completa_008.csv` | `bag_fase1_20261106_142217` |
| 5 | 14:34 | 0.12 | obstáculo en E esquivado; **SIN_CAMINO en la puerta** (`RELOC_PARCIAL sin_borde`) | `mission_log_ruta_012_sin_borde.csv` | `bag_fase1_20261106_143325` |
| 6 | 14:42 | 0.12 | **A → Q en 3:36** con el arreglo del escalón | `mission_log_ruta_completa_012.csv` | `bag_fase1_20261106_144143` |

(La ruta 5 usó la pose de `comprobar_pose` de las 14:24, sin repetirla.)

- **Relocalización adelantada: funciona.** Ruta 4: `RELOC dy=-0.442 Y=canaleta`; ruta 6: `dy=-0.462 Y=canaleta`. La
  segunda RELOC en E-Door_2 queda en ~3 cm (dx +0.019, dy +0.031/+0.036) en las dos.
- **Ruta 5, `sin_borde`:** el respaldo de la canaleta pedía ≥ 2 puntos del escalón de yeso detrás de la recta. De lado,
  el escalón da 2-3 rayos y el primero (0.7 cm detrás) entra en la recta: en la ruta 4 quedaron 2 (+3.1, +3.3 cm), en
  la 5 solo 1 (+2.7 cm). Sin dy, la Y siguió +0.47 m mal y SIN_CAMINO sin fin (como la ruta 2). **Arreglo: basta 1
  punto** (detrás de la recta solo puede estar la pared; el scan es la mediana de 10). Con los bags: ruta 4 igual
  (dy −0.442), ruta 5 `total canaleta` dy −0.471. Test `test_escalon_de_un_solo_punto` (falla con el código anterior).
  Comprobado en el robot en la ruta 6.
- Corrección con el mapa a 0.12 (ruta 6): 113 correcciones, mediana |dx| 0.9 cm, |dy| 0.7 cm, |dth| 0.3°. Tras B,
  tres seguidas de dx ≈ −0.07 (la X llega 7 cm por delante tras el giro de B: a más velocidad crecerá).
- Obstáculos: junto a D-E_1/E bloqueo + replan, llega a E a 2 cm (rutas 5 y 6). Tras Door-Q_1, en (7.55, 12.9), bloqueo
  y replan en las dos rutas completas, ~25 s: algo fijo que no está en el mapa (sin mirar).
- `TIMEOUT_SENSOR` momentáneo en la ruta 4 (14:26:06, entre B y D), recuperado en el mismo segundo; en el bag ningún
  hueco > 0.47 s (umbral 0.5). Vigilar.
- `real.yaml`: `max_linear_speed` 0.12.

### Error en Q (~20 cm, sistemático)

Ruta 6, cinta + `foto_Q_ruta_012.jpg` (robot mirando a +X): del eje de las ruedas a la cruz **20 cm**, la cruz delante a
la derecha → real ≈ **15-17 cm por detrás en X y 8-10 cm hacia +Y** de Q, ligeramente girado. El robot cree estar en Q a
2 cm. Ruta 4 (`foto_Q_ruta_008.jpg`, sin cinta): la misma posición respecto a la cruz. Desde la puerta no hay corrección
(`map_corr_y_max` 10.2; la última RELOC es en E-Door_2): ~3.5 m de odometría con dos giros. Como se repite, lo probable
es que Q (o el mapa detrás de la puerta) no coincida con la cruz, no deriva.

## Pendiente (actualizado)

1. Reintentar la relocalización en cada SIN_CAMINO junto a la puerta mientras no haya salido `total` (alguien tapando el
   final de la canaleta deja ahora el robot parado para siempre).
2. Error en Q: con los bags 4 y 6, corrección con el mapa en sombra más allá de Y = 10.2 y ajuste a las paredes en Q.
   Si el mapa es bueno ahí, activarla; si no, recolocar Q y los waypoints de detrás de la puerta según la cruz.
3. Velocidad: 0.15 global (solo el yaml); después perfil por tramos (rápido en rectas, ~0.10 cerca de obligatorios,
   puerta y replanes; `block_lookahead` mayor en rectas), probado en Gazebo, y rectas a 0.18 → 0.20 (0.22 es el
   máximo del motor: sin margen para corregir el rumbo).
4. El obstáculo fijo en (7.55, 12.9).

## Rutas 7-9 (0.12 m/s): relocalización en Q, mesas en el mapa y otros robots

| Ruta | Hora | Resultado | Log | Bag |
|---|---|---|---|---|
| 7 | 14:53 | A → Q (3:16); **otra vez 20 cm, mismo ángulo** (cinta) | `mission_log_ruta_completa_012_b.csv` | `bag_fase1_20261106_145112` |
| 8 | 15:08 | otros robots en la ruta: **rodeo hacia atrás por las mesas** tras E; Ctrl+C | `mission_log_ruta_robots_rodeo_mesas.csv` | `bag_fase1_20261106_150557` |
| 9 | 15:17 | con otros robots: **A → Q, sobre la cruz** (`foto_Q_reloc_Q.jpg`) | `mission_log_ruta_reloc_Q.csv` | `bag_fase1_20261106_151702` |

### Error en Q → relocalización con la esquina de Q

- Cinta desde la cruz de Q: **0.60 m a la pared de delante (+X) y 0.50 m a la de la derecha (−Y)** → paredes en
  X 9.570 e Y 13.997. El mapa dice 9.530 y 13.780 (mal en Y ~0.22 m, como el tramo del escalón a la puerta).
- Con el mapa (`scripts_tarde/pose_mapa_Q.py`) la X coincidía con la cinta pero la Y no (~0.28 m): el mapa no sirve
  detrás de la puerta. Con las dos paredes medidas (`scripts_tarde/esquina_Q.py`), en las rutas 4, 6 y 7:
  real − creída ≈ (−0.13/−0.17, +0.09/+0.13), **rumbo −8.3/−8.6°** (la foto de la ruta 6 lo confirma: ~8° horario).
  Las dos paredes difieren ~3° entre sí: el rumbo sale de la pared Y (como `pose_heading_axis` en A).
- Nodo: al llegar a Q (o a su sustituto Q~) se para, `ajuste_esquina` con la mediana de 10 scans, corrige x, y y
  rumbo, vuelve a ir a Q y repite (`reloc_q_times: 2`). Eventos `LLEGADA_RELOC_Q n`, `RELOC_Q`, `RELOC_Q_RECHAZADA`,
  `RELOC_Q_INCONSISTENTE`. En fase1.yaml las paredes del mapa (Gazebo); en real.yaml las de la cinta.
- **Ruta 9:** `RELOC_Q dx=-0.184 dy=+0.115 dth=-6.35` y la segunda `dx=-0.005 dy=+0.010 dth=+0.25`. Sobre la cruz.
- **Sin explicar:** de dónde salen los 6-9° de rumbo entre la puerta (RELOC deja dth ~0) y Q. Mirar antes de subir
  la velocidad.

### Rodeo por las mesas (ruta 8) → mesas en el mapa y rodeos largos rechazados

- 15:10:35, pasillo de E a la puerta tapado (otro robot) en (4.93, 7.37): el replan hasta Door daba **10.07 m por
  dentro de la sala** (X mín 2.57) donde quedaban 5.20, y el robot volvió hacia D. Las mesas no estaban en el mapa:
  sus patas salían como OBSTACULO en X 2.5-4.5, Y 3.7-8.7.
- **Zonas prohibidas `mesas_D_E` (X 1.50-4.28, Y 3.00-7.30) y `mesas_puerta` (X 1.50-4.87, Y 7.30-12.65)** con los
  bordes medidos con cinta (X 1.50 sin medir). Ruta regenerada: E-Door_1 (5.115, 6.805), nuevo E-Door_2
  (5.715, 8.155); el de la relocalización en la puerta, (5.915, 11.455), ahora se llama E-Door_3. 18.61 m.
- **Rodeo largo:** un replan más largo que 1.5 × lo que quedaba + 1 m no se toma hasta 30 s sin camino (se espera,
  `RODEO_LARGO`). Con el bag (`scripts_tarde/rodeo_1510.py`): mapa viejo 10.07 m → RODEO_LARGO; mapa nuevo 5.19 m
  por el pasillo.
- **Reintento de la relocalización en la puerta:** SIN_CAMINO a < 1 m de Door sin RELOC total → `RELOC_REINTENTO`
  (hasta 10). Sin probar en el robot (no ha hecho falta).
- `reloc_max_dy` 0.55 → 0.65: dy de la tarde −0.442, −0.462, −0.471, −0.490, −0.446.
- Ruta 9: bloqueos entre D y E rodeados en local (X ≥ 4.7), E tapado → E~ a 0.21 m, y el bloqueo de siempre tras
  Door-Q_1 (7.55, 12.9), ~25 s. Corrección con el mapa: mediana 1.0 cm / 0.5 cm / 0.3°.

## Pendiente (15:22)

1. Rumbo: 6-9° de error acumulado entre la puerta y Q (ver arriba). Con los bags: /odom frente a las paredes en ese tramo.
2. Velocidad: 0.15 global; después perfil por tramos (rectas a 0.18 → 0.20), probado en Gazebo.
3. El bloqueo fijo tras Door-Q_1 en (7.55, 12.9): ¿qué hay ahí que no está en el mapa?
4. Borde lejano de las mesas (X 1.50) sin medir; no afecta a la ruta.
