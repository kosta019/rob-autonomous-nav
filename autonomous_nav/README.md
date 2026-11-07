# autonomous_nav

Navegación autónoma del TurtleBot3 Burger para el proyecto de Robótica (ROB, FIB): fase I (de la zona 1 a la pose final
pasando por E, Door y Q), fase II (exploración del pasillo, identificación de la estación de carga y vuelta a P Base) y
fase III (aparcado en la estación). Navegación propia, sin Nav2; el SLAM (slam_toolbox) solo construye el mapa.

## Requisitos

ROS 2 Jazzy, el paquete del TurtleBot3, `slam_toolbox` y `nav2_map_server` (solo para guardar el mapa). En Python, solo
`numpy` y `yaml`. Los dos scripts de `scripts/` (offline) necesitan además `scipy` y `Pillow`.

## Compilar

```bash
cd ~/ros2_ws/src && # copiar aquí la carpeta autonomous_nav
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
export TURTLEBOT3_MODEL=burger
CFG=install/autonomous_nav/share/autonomous_nav/config
```

## Ejecutar (robot real, con el bringup ya en marcha)

SLAM, con el robot en la salida (terminal aparte):
```bash
ros2 launch autonomous_nav slam.launch.py
```

Misión completa desde A (el robot sobre la cruz, mirando a +X; al arrancar corrige solo su pose con el LiDAR):
```bash
ros2 run autonomous_nav fase1_node --ros-args --params-file $CFG/fase1.yaml --params-file $CFG/fase1_ruta.yaml \
  --params-file $CFG/real.yaml -p phase2_enabled:=true -p phase3_enabled:=true
```
- Desde B, C o D: añadir `--params-file $CFG/salida_C.yaml` (o `salida_B` / `salida_D`) después de `real.yaml`.
- Otra pose final de la fase I: `-p final_pose:="[x, y, theta]"`.

Al acabar la fase II (`FASE II COMPLETADA`), guardar el mapa:
```bash
ros2 run nav2_map_server map_saver_cli -f mapa_fases_I_II
```

## Mission Log

`mission_log_<fecha>.csv` en la carpeta desde la que se lanza el nodo. Columnas: `timestamp`, `current_phase` (I, II o
III), `robot_x`, `robot_y`, `robot_yaw` y, además, `state`, `event`, `error`, `obstacle_x`, `obstacle_y`, `yaw_odom` y
`yaw_imu`. Eventos principales en `event`:
- `OBSTACULO`: obstáculo detectado (en `obstacle_x/y`);
- `ESTACION_CUADRADA` / `ESTACION_TRIANGULAR`: estación identificada (centro en `obstacle_x/y`, ángulo en `error`);
- `PILAR` / `PILAR_DESCARTADA`: cada pata de la estación buena / de la descartada;
- `FASE I COMPLETADA`, `FASE II COMPLETADA`, `FASE III COMPLETADA` (en esta última, `error` es la distancia en metros
  del centro del robot al de la estación).

## Si cambian las coordenadas

Los puntos, poses y zonas están en `config/fase1.yaml`, nunca en el código. Después de cambiarlos, regenerar la ruta y
la rejilla de holgura (desde `autonomous_nav/`): `python3 scripts/planificar_fase1.py`. Si cambian las paredes
(`maps/paredes.yaml`), también el mapa de distancias: `python3 scripts/generar_mapa_distancias.py`.
