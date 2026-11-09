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

## Misión I + II con el BOT 11 (15:17): la de la entrega de seguridad

`ROS_DOMAIN_ID=11`, fase II a 0.20 m/s, `CORR_PASILLO` activo con las paredes Y 17.15 y 13.31. Ficheros:
`mission_log_bot11_entrega.csv`, `bag_bot11_20261109_151634.tar.zst`, `mapa_bot11.*`. Batería a 11.36 V al salir.

- `FASE II COMPLETADA` a los 370 s. `POSE_INICIAL` aceptada (−0.18°). Puerta y Q sin rechazos.
- Estaciones: cuadrada en (1.88, 14.58), triangular en (5.41, 16.29). `CORR_PASILLO`: 113 eventos.
- Llegada a P Base: **en X, unos 26 cm de la cruz (aproximado)**; en Y, bien (LiDAR: 15.71 con el extintor, 15.79 con la sala).
  Signo de la X sin anotar.
- Rumbo: 3.9° de diferencia con las paredes al final (17° con el BOT 04).
- Error de X parado en `P2_e`: +0.06 a +0.08 m, con una pared x = 9.54 (P Base 3.34 + 6.20 de cinta). Esa pared no estaba
  activa en esta misión.

## Fases II + III desde Q con el BOT 11 (15:33), con otro robot moviéndose por el pasillo

`test_real/prueba_fase23_desde_Q.yaml`, pared x = 9.54 activa. Ficheros: `mission_log_bot11_fases23_desde_Q.csv`,
`bag_bot11_fases23_20261109_153247.tar.zst`. Batería de 11.25 a 11.19 V.

- II + III en 282 s. El robot acabó centrado en la estación (visto por Marc). Log: `FASE III COMPLETADA sin_verificar
  patas=3`. La estación se recoge cada día y se coloca en otro sitio.
- Centro de la estación: (1.88, 14.33) en la fase II y (2.16, 13.94) medido de cerca: 48 cm. No saltó el límite de 0.25 m
  (sin averiguar por qué).
- P Base, medido con las paredes (X 5.18 y 1.32 del mapa; Y 17.15 y 13.31): X +17 a +19 cm (15:17: +27, que coincide con
  los ~26 vistos), **Y +39 a +46 cm**, rumbo −8.3° (15:17: −3.6°). Giro total parecido en las dos (1236° y 1129°).
- `CORR_PASILLO` deja de actuar hacia los 190 s: con el rumbo torcido los puntos caen fuera de la ventana.

## Corrección de rumbo con las paredes (`rumbo_paredes`): NO validada, apagada

Rehaciendo la pose de la fase II con los /odom y /scan grabados (error final respecto a la pose real):

| Misión | Variante | Rumbo | X | Y |
|---|---|---|---|---|
| 15:17 | paredes X e Y, sin rumbo | +3.6° | −15 cm | −4 cm |
| 15:17 | paredes X e Y, con rumbo | −0.9° | −15 cm | **−45 cm** |
| 15:33 | paredes X e Y, sin rumbo | +7.1° | −18 cm | −46 cm |
| 15:33 | paredes X e Y, con rumbo | −1.9° | −13 cm | −34 cm |

Con rumbo, `ajuste_paredes` casi no recibe puntos (ny ≈ 0). Sin resolver: en la primera pasada de las 15:17 el rumbo
corregido hace bajar la Y, mientras que la pared 13.31 indicaba que la Y real era mayor que la creída.

## Relocalización en P Base (16:01, II + III desde Q, con otros robots en el pasillo)

Al llegar a P Base en la fase II: gira a +X, se relocaliza parado con `ajuste_esquina` (X 5.332 calibrada con el robot
sobre la cruz, foto; Y 17.15 de cinta) y vuelve a la cruz; dos veces. El tramo de X se desvía unos 7.4° de la perpendicular:
el rumbo sale del extintor.

- `RELOC_P dx=+0.226 dy=+0.577 dth=-11.95` y luego `dx=-0.023 dy=-0.018 dth=+0.41`. Acabó bien en la cruz (visto por Marc).
  Rozó los límites: subidos a 0.80 m y 15°.
- Fase III: `SIN_CAMINO` en el primer lado, `FASE3_OTRO_LADO` y tres `FASE3_MEDIDA_FALLIDA sin_patas`: `FASE_III_ABANDONADA`,
  sin entrar. Distancias entre lo visto: 0.39, 0.41 y 0.61 (esperado 0.44 y 0.62). Con tolerancia de 5 cm saldría un centro con
  un objeto que no encaja con las patas de la fase II: no se afloja. Causa sin confirmar.

La corrección de rumbo no validada queda fuera del código: `rumbo_paredes_no_validado.patch`.

## Segunda prueba con la relocalización en P Base (16:11, II + III desde Q)

`mission_log_bot11_fases23_reloc_P_2.csv`, `bag_bot11_fases23_reloc_P_2_20261109_161124.tar.zst`. `RELOC_P dx=+0.166 dy=+0.386
dth=-8.10` y luego `+0.017 -0.011 +0.18`. Fase III: tres `sin_patas` desde (1.53, 14.90) y `FASE_III_ABANDONADA`, sin entrar.

Hipótesis descartada: que la estación guardada se quedara en el sistema antiguo tras la relocalización. Las patas vistas de
cerca están a 9-19 cm de las de la fase II, y a 22-67 cm si se les aplica la corrección. Causa sin averiguar.

## Misión final de la entrega (16:22, BOT 11, fases I + II + III desde A)

`mission_log_bot11_mision_final.csv`, `bag_bot11_mision_final_20261109_162122.tar.zst`, `mapa_final.*` (guardado al salir
`FASE II COMPLETADA`). Con vídeo y con la relocalización en P Base **apagada** (`-p reloc_pbase_times:=0`; `real.yaml` la deja
en 2), porque sin ella la fase III ya había entrado (15:33) y con ella había abandonado dos veces.

- `FASE I COMPLETADA` a 171 s, `FASE II COMPLETADA` a 366 s, `FASE III COMPLETADA patas=3` a 429 s, sin tocar patas (Marc).
- Aparcado a 3.2 cm del centro medido, por eso `APARCADO_FUERA_DE_TOLERANCIA` (umbral propio de 3 cm).
- Estaciones: cuadrada en (2.03, 14.62), triangular en (5.38, 16.30). P Base: error en X sin medir en esta misión.

## Fase I sin parar en cada punto (16:39, rama `fase1-sin-paradas`, BOT 11, fases I + II + III desde A)

`mission_log_bot11_sin_paradas.csv`, `bag_bot11_sin_paradas_20261109_163943.tar.zst`, `mapa_sin_paradas.*`. `phase1_pass_speed` 0.06,
relocalización en P Base apagada. Comparación con la misión de las 16:22:

| | 16:22 | 16:39 |
|---|---|---|
| E | 64.7 s | 59.5 s |
| Door | 119.4 s | 109.2 s |
| Fase I completada | 171.3 s | 159.0 s |
| Fase II completada | 366.4 s | 432.5 s (5 `BLOQUEO`, otros robots/personas) |
| Fase III completada | 428.6 s | 480.2 s, `sin_verificar patas=2` |

Un `PARADA_EMERGENCIA` a los 321.5 s en la fase II, con un obstáculo a unos 9 cm en (4.07, 15.29). Estación cuadrada movida a
(3.42, 14.21). Dos `sin_patas` (con 4 objetos vistos) antes de `FASE3_CENTRO patas=3`.
