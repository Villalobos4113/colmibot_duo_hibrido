# colmibot_gazebo

Este paquete simula un COLMIBOT estacionario en Gazebo Harmonic, con RViz 2, sobre ROS 2 Jazzy.
Usa la descripción de `colmibot_description` (modelo de Víctor) y un arranque nuevo basado en los
patrones de Gazebo de Mingyar. Es un entorno solo de simulación: no hay `DiffDrive`, `/cmd_vel`,
`/odom`, TF desde Gazebo, joystick, teleop, firmware, serial ni hardware.

## Lanzar la demo en WSL

Usa una terminal nueva. El bloque abre un shell limpio para que `~/.bashrc` no cargue
`robotics_ws`, compila los dos paquetes y lanza Gazebo con RViz. Se cierra con Ctrl-C. En las pruebas de C-006, RViz
tardó más de 10 s en cerrarse y `launch` lo terminó con SIGKILL, o salió con SIGSEGV. El envoltorio
`ruby` de Gazebo siempre sale con -2. Ninguno de los dos casos dejó procesos vivos.

```bash
env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 TERM="$TERM" PATH=/usr/bin:/bin \
  DISPLAY="${DISPLAY:-:0}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash --noprofile --norc -c '
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=86 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=colmibot_demo GZ_IP=127.0.0.1
cd "$HOME/itam/seminario-de-titulación/colmibot_duo_hibrido/repo"
colcon build --packages-select colmibot_description colmibot_gazebo
source install/setup.bash
ros2 launch colmibot_gazebo sim_stationary.launch.py'
```

| Argumento | Defecto | Efecto |
| --- | --- | --- |
| `gui` | `true` | `false` lanza solo el servidor de Gazebo (`-s`) |
| `rviz` | `true` | Abre RViz con `rviz/sim.rviz` (fixed frame `base_footprint`) |
| `use_camera` | `false` | Añade la cámara simulada, el system `Sensors` y su puente. En RViz, activa la vista *Camera* |
| `use_meshes` | `true` | `false` usa geometría simplificada de Víctor (por ejemplo, en un clon sin mallas) |

Sin GPU, WSL renderiza con llvmpipe y consume mucha CPU. Puedes usar la GPU NVIDIA solo para ese
proceso, sin cambiar la configuración del sistema: antepón
`GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` a `ros2 launch`. En C-006 esa variante se
probó solo con la cámara y sin ventanas (`use_camera:=true gui:=false rviz:=false`). La GUI de Gazebo
con D3D12 se probó antes, en C-004B, pero no con este paquete.

Si faltan las mallas (por ejemplo, en un clon nuevo), recupéralas según
`../colmibot_description/SOURCE.md`.

## Contrato de interfaces (un robot)

| Tópico ROS | Productor | Dirección |
| --- | --- | --- |
| `/clock` | Gazebo, vía `colmibot_bridge` | GZ→ROS |
| `/joint_states` | JointStatePublisher de Gazebo, vía `colmibot_bridge` (único publicador) | GZ→ROS |
| `/robot_description`, `/tf`, `/tf_static` | `robot_state_publisher` con `use_sim_time: true` | ROS |
| `/camera/image_raw`, `/camera/camera_info` (opcional) | cámara de Gazebo, vía `colmibot_camera_bridge`, `frame_id` `camera_optical_frame` | GZ→ROS |

El árbol TF es `base_footprint → base_link → {wheel_left_link, wheel_right_link, camera_link →
camera_optical_frame, imu_link, tof_*_link, vis_*}`. No hay frame `odom`.

`colmibot_description` exporta `<gazebo_ros gazebo_model_path="${prefix}/../"/>`.
`ros_gz_sim/gz_sim.launch.py` añade esa ruta a `GZ_SIM_RESOURCE_PATH` sin sobrescribirla, y así
Gazebo resuelve los URI `package://colmibot_description/meshes/…`.

## Origen y créditos

**Mingyar** (perfil `Mingyar2`), https://github.com/Mingyar2/colmibot_ws, commit
`0aed97231f0affe94050d3ba44c4698547033661`, `robot_description/robot_description/`. Se tomaron
como patrón, reescritos y no copiados como archivo, estos elementos:

- `launch/gazebo_colmi.launch.py`: la inclusión de `gz_sim.launch.py` con `empty.sdf`,
  `robot_state_publisher` con tiempo simulado, `ros_gz_sim create -topic robot_description -name colmibot`
  y los puentes GZ→ROS de `/clock`, `/joint_states` y la cámara.
- `urdf/colmibot.urdf`: el plugin JointStatePublisher (líneas 329-333), el bloque de cámara
  (líneas 217-235: 640×480, R8G8B8, hfov 1.229 rad, clip 0.05–15 m) y el system Sensors con
  `ogre2` (líneas 336-338).

No se tomaron de Mingyar: el launch completo (incluye `joy_node`, teleop y el puente `/cmd_vel`
ROS→GZ), el `SetEnvironmentVariable` que sobrescribe `GZ_SIM_RESOURCE_PATH`, el DiffDrive, los
puentes `/odom` y `/tf`, la geometría, las masas, la vía de 0.22 m ni la pose de su cámara.

**Víctor Manuel Jiménez González:** `rviz/sim.rviz` adapta `src/robot_description/rviz/display.rviz`
de https://github.com/VicManJ-cloud/colmibot_ws en el commit `985b6b8768e7c44ddb5629a903085039dd937eb5`;
solo se añadió la vista *Camera*, desactivada.

**Nuevo en C-006:** `urdf/colmibot_sim.urdf.xacro`, que envuelve la descripción sin modificarla,
`launch/sim_stationary.launch.py`, `config/bridge_*.yaml`, `test/test_static_contract.py`,
`CMakeLists.txt`, `package.xml` y este README. Cambios frente al patrón de Mingyar: la cámara se
monta en el `camera_link` de Víctor (mira hacia +X) y publica con
`gz_frame_id=camera_optical_frame`, y la tasa baja de 30 a 15 Hz.

Ningún repositorio de origen tiene licencia a nivel de repositorio. Víctor declara MIT en el
`package.xml` de `robot_description`, sin archivo `LICENSE` ni titular. Mingyar declara
`TODO: License declaration` en sus `package.xml` y `Apache-2.0` en sus `setup.py`. Aquí no se
declara ninguna licencia y el campo `<license>` queda pendiente.

## Limitaciones

- Solo es estacionario. La caja de colisión del chasis llega al suelo y envuelve ruedas y casters
  (ver `SOURCE.md` de la descripción). Por eso este paquete no sirve para pruebas de movimiento.
- Masas e inercias provisionales (CAD sin pesar).
- JointStatePublisher publica en cada paso de física (1 kHz).
- `sdformat` avisa que `gz_frame_id` no está en su esquema, pero lo conserva y la cámara publica
  con ese `frame_id`.
- Que un tópico de velocidad valga cero no prueba una parada física, y aquí no existe ninguno.

## Pruebas

```bash
colcon test --packages-select colmibot_description colmibot_gazebo
colcon test-result --verbose
```

`test_static_contract.py` convierte el xacro con `gz sdf -p` y comprueba que no hay elementos de
movimiento, que los puentes son GZ→ROS, las acciones del launch, la exportación de la ruta de
recursos y el fixed frame de RViz.
