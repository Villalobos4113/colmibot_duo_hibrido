# COLMIBOT DUO — repositorio de implementación

Workspace ROS 2 Jazzy (colcon). Código en `src/`; salidas `build/`, `install/`, `log/` ignoradas.

| Paquete | Estado | Qué contiene |
| --- | --- | --- |
| [`colmibot_duo_core`](src/colmibot_duo_core/README.md) | C-002 | Misión simbólica, evidencia SQLite y adaptador ROS 2 mínimo. Solo datos sintéticos, sin actuación física |

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select colmibot_duo_core
colcon test --packages-select colmibot_duo_core
colcon test-result --verbose
```

Este repositorio no contiene ni ejecuta control de motores, puentes seriales, joystick ni
publicación de velocidades. Los datos de prueba nunca se guardan dentro del código fuente.
