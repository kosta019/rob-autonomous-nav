# Pruebas reales del 09/11/2026 (BOT 04, sustituto del BOT 11 durante la tarde)

`ROS_DOMAIN_ID=4`. Calibración de `real.yaml` hecha con el BOT 11, sin ajustar para este robot.

## Misiones I + II desde A (0.12 m/s en el pasillo)

| Fichero | Bag y SLAM | Resultado |
|---|---|---|
| `mission_log_bot04_sin_bag.csv` (14:31) | No | `FASE II COMPLETADA` a los 471 s |
| `mission_log_bot04_con_bag.csv` (14:54) | Sí: `bag_bot04_20261109_145202.tar.zst`, `mapa_bot04.*` | `FASE II COMPLETADA` a los 495 s |

- `POSE_INICIAL` aceptada en las dos, primera vez en el robot real (171 y 200 puntos, RMS 1.5 y 1.7 cm).
- Estaciones movidas respecto a R y T y encontradas igual. Cuadrada: (4.10, 15.51) en la primera y (1.88, 14.25) en la
  segunda; triangular en (5.71, 15.55) en la primera.
- Puerta: `RELOC_PARCIAL sin_borde` y luego `RELOC` con dy −0.416 y −0.473 (límite en `real.yaml`, 0.65).
- `TIMEOUT_SENSOR` e `IMU_HUECO` durante unos 30 s cerca de `P1_o` (segunda misión). Causa sin averiguar.

## Medidas con cinta desde la cruz de P Base

- Pared del tramo ancho (lado de la sala): 2.39 m, Y = 13.31 (añadida a `corridor_wall_*`).
- Pared del extintor: 1.45 m, Y = 17.15 (igual que el 06/11).
- El extremo X menor del pasillo no es el X −0.55 del mapa: está a más de 5 m de P Base (dato de Marc, sin medir exacto).
- LiDAR parado en P: 1.46 y 2.41 m. Coincide con la cinta, así que el LiDAR ve las paredes reales.

## Rumbo del BOT 04 en la fase II (bag de las 14:54)

Giro que alinea el scan con las paredes del pasillo, acumulado desde Q: +4.7° a 4.5 m, +8.2° a 11.5 m, +13.7° a 20.8 m,
+16.7° a 26.1 m. Crece también al pasar por la misma X, así que es del rumbo, no de las paredes.

- `/odom` e `/imu` dan el mismo rumbo. Con las ruedas el error bajaría a unos 11°.
- Parado no deriva (0.0008°/s; `/imu` da exactamente 0, quizá una zona muerta, sin comprobar).
- Con el rumbo torcido los puntos de pared caen fuera de la ventana de 0.15 m: por eso `CORR_PASILLO` no actuó (0 eventos
  en la primera misión; 1 `CORR_PASILLO_SOMBRA` en la segunda).

## Sin confirmar

Posible LiDAR del BOT 04 girado 5-7° respecto al chasis: colocado a ojo da siempre el mismo signo (+4.5° a +6.75°) y en Q
el chasis queda visiblemente torcido. Falta la prueba de apoyarlo contra una pared; se recuperó el BOT 11 antes de hacerla.
