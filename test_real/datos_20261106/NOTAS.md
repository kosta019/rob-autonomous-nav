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
