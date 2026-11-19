# colmibot_gazebo

Este paquete simula un COLMIBOT en Gazebo Harmonic, con RViz 2, sobre ROS 2 Jazzy. Usa la
descripción de `colmibot_description` (modelo de Víctor) y un arranque nuevo basado en los patrones de
Gazebo de Mingyar. Es un entorno solo de simulación: no hay joystick, teleop, firmware, serial ni
hardware. Tiene dos demos independientes:

- `sim_stationary.launch.py` (C-006): robot estacionario, sin `DiffDrive`, sin tópicos de velocidad, sin
  `/odom` ni TF desde Gazebo.
- `sim_motion.launch.py` (C-007): locomoción diferencial **solo en Gazebo** con el `DiffDrive` nativo
  de Harmonic. El único comando aceptado es `/colmibot_sim/cmd_vel` y la prueba usa una sonda finita
  (`motion_probe`). Ver "Locomoción simulada (C-007)".

## Lanzar la demo estacionaria en WSL

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

## Locomoción simulada (C-007)

Todo esto ocurre en Gazebo. Un tópico de velocidad en cero no prueba una parada física, y nada de esta
demo se conecta a motores, ESP32, UART ni joystick.

### Lanzar la demo de movimiento

Terminal 1: compila y abre Gazebo y RViz (fixed frame `odom`). Las dos terminales deben usar el mismo
`ROS_DOMAIN_ID` y la misma `GZ_PARTITION`; si 87 está ocupado en tu máquina, usa otro dominio libre en
ambas.

```bash
env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 TERM="$TERM" PATH=/usr/bin:/bin \
  DISPLAY="${DISPLAY:-:0}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash --noprofile --norc -c '
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=colmibot_motion_demo GZ_IP=127.0.0.1
cd "$HOME/itam/seminario-de-titulación/colmibot_duo_hibrido/repo"
colcon build --packages-select colmibot_description colmibot_gazebo
source install/setup.bash
ros2 launch colmibot_gazebo sim_motion.launch.py use_meshes:=true'
```

Terminal 2, cuando el robot ya aparece en Gazebo: la prueba finita. La sonda dura unos 14 s simulados y
termina sola.

```bash
env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 TERM="$TERM" PATH=/usr/bin:/bin \
  bash --noprofile --norc -c '
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=colmibot_motion_demo GZ_IP=127.0.0.1
cd "$HOME/itam/seminario-de-titulación/colmibot_duo_hibrido/repo"
source install/setup.bash
ros2 run colmibot_gazebo motion_probe --out "$(mktemp -d /tmp/colmibot_motion_probe.XXXXXX)"'
```

Cierra la terminal 1 con Ctrl-C. En C-007 RViz necesitó SIGKILL de `launch` al cerrar, como en C-006.
En una de once corridas de `sim_motion.launch.py` (la primera de C-007) el servidor `gz sim` quedó
vivo después de que `launch` terminara; no se reprodujo y su causa no se determinó. Comprueba que no quede nada con `ps -eo pid,cmd | grep '[g]z sim'` y, si queda, termina
ese PID concreto.

`sim_motion.launch.py` acepta los mismos argumentos que la demo estacionaria (`gui`, `rviz`,
`use_camera` y `use_meshes`). Sin ventanas: `gui:=false rviz:=false`. Todo lo medido en C-007 usa las
mallas (`use_meshes:=true`). Con `use_meshes:=false` la descripción toma las estimaciones de
`colmibot_params.xacro` (vía 0.19 m, llanta de 26 mm) y la colisión `mobile` cambia en
consecuencia. Esa variante solo se convirtió con xacro y `gz sdf -p`; no se ejecutó en Gazebo, y las
expectativas de rueda de la sonda (vía 0.1741 m) no le corresponden.

### Qué cambia respecto a la demo estacionaria

- `urdf/colmibot_motion.urdf.xacro` incluye `colmibot_sim.urdf.xacro` y añade:
  - `collision_mode:=mobile`: colisión del chasis sin contacto con el suelo ni con las ruedas; ver
    `colmibot_description/SOURCE.md`. El launch lo pide explícitamente.
  - Fricción nula (`mu1 = mu2 = 0`) en las dos esferas caster, que funcionan como bolas locas. El SDF
    generado lo confirma: solo esas dos colisiones tienen `<mu>0</mu>`, y chasis y llantas conservan
    la fricción por defecto.
  - `gz::sim::systems::DiffDrive` con `wheel_left_joint` y `wheel_right_joint`. Radio
    `${wheel_radius}` = 0.030 m y vía `${wheel_left_y - wheel_right_y}` = 0.17410 m, es decir, las
    mismas propiedades que posicionan las ruedas (no los 0.22 m de Mingyar).
  - `frame_id` `odom` y `child_frame_id` `base_footprint`.
  - Límites solo de demostración (no calibrados): ±0.10 m/s, ±0.60 rad/s, ±0.25 m/s² y ±1.5 rad/s².
- `config/bridge_motion.yaml` (nodo `colmibot_motion_bridge`) tiene una sola entrada ROS→GZ
  (`/colmibot_sim/cmd_vel` → `/model/colmibot/cmd_vel`) y dos GZ→ROS (`/odom` y `/tf`). `/clock` y
  `/joint_states` siguen en `colmibot_bridge` con `bridge_stationary.yaml`.
- `rviz/sim_motion.rviz` usa fixed frame `odom` y añade una vista Odometry.

| Tópico ROS | Productor | Dirección |
| --- | --- | --- |
| `/colmibot_sim/cmd_vel` (`geometry_msgs/msg/Twist`) | solo `motion_probe` en las pruebas; suscriptor único `colmibot_motion_bridge` | ROS→GZ |
| `/odom` (`nav_msgs/msg/Odometry`, `odom` → `base_footprint`) | DiffDrive, vía `colmibot_motion_bridge` | GZ→ROS |
| `/tf` arista `odom` → `base_footprint` | DiffDrive, vía `colmibot_motion_bridge` (única autoridad) | GZ→ROS |
| `/tf` ruedas, `/tf_static` resto del árbol | `robot_state_publisher` | ROS |
| `/clock`, `/joint_states` | Gazebo, vía `colmibot_bridge` (un publicador cada uno) | GZ→ROS |

El árbol TF es `odom → base_footprint → base_link → {ruedas, cámara, IMU, ToF, vis_*}`.

### Sonda de prueba `motion_probe`

`ros2 run colmibot_gazebo motion_probe --out DIR` publica solo en `/colmibot_sim/cmd_vel` (fijo en el
código), a 20 Hz y midiendo cada fase en tiempo simulado:

1. Reposo de 1 s.
2. Avance a +0.05 m/s durante 2 s y reposo de 2 s.
3. Retroceso a −0.05 m/s durante 2 s y reposo de 2 s.
4. Giro a +0.30 rad/s durante 2 s y reposo de 2 s.
5. Diez ceros finales. También los envía si la secuencia se interrumpe por SIGINT, SIGTERM, SIGHUP o
   una excepción, siempre que esa instancia haya llegado a ser la escritora activa. No los envía tras
   SIGKILL ni si el proceso, el puente o el transporte fallan.

Antes de publicar algo distinto de cero exige tiempo simulado (`/clock`), `/odom`, `/joint_states` y la
pose verdadera de Gazebo reciente (`/world/empty/pose/info`, vía gz-transport, con menos de 0.5 s
simulados de antigüedad). Además, el único suscriptor del comando debe ser `colmibot_motion_bridge`.

**Una sola escritora (C-007-P2).** La exclusividad se comprueba en cuatro pasos, todos antes del primer
Twist:

1. Antes de crear entidades ROS, la sonda normaliza `ROS_DOMAIN_ID` a su entero canónico (C-007-P2B):
   `87`, `+87` y `' 87'` son el mismo dominio y la misma clave. Rechaza con código 2, antes de ROS y
   sin crear el lock, las formas que rcl leería de otro modo o no acepta: ceros a la izquierda (`010`
   es octal, 8, en Jazzy; `087` es inválido), hexadecimal, signos, decimales y valores mayores que 232.
   Sin definir o exactamente vacío es 0, pero un valor de solo espacios (`' '`) o con caracteres no
   ASCII (por ejemplo NBSP) también se rechaza (C-007-P2C).
   Después toma un lock local no bloqueante (`fcntl.flock`) en
   `$TMPDIR/colmibot_motion_probe.domain<dominio>_colmibot_sim_cmd_vel.lock` (por defecto en `/tmp`).
   El archivo no se borra; el lock se libera al salir. Si otra sonda lo tiene, sale con código 2.
2. Con el grafo estable durante 1 s, no debe haber ningún publicador en el tópico. Se cuentan
   endpoints por GID, no por nombre de nodo; uno ajeno provoca un rechazo inmediato.
3. Crea su publicador y lo reconoce como el único GID nuevo, que debe mantenerse único durante 1 s.
4. Solo entonces pasa a escritora activa. Durante la maniobra audita el grafo cada 0.5 s: si aparece
   otro endpoint, o si deja de llegar la pose verdadera durante más de 0.5 s simulados, aborta y envía
   ceros.

Una instancia rechazada sale con código 2: no publica nada, ni ceros, y no escribe en `DIR`.

**Alcance exacto de la exclusividad.** Esto no es un árbitro de mando:

- `flock` solo excluye a procesos de la misma máquina que comparten la misma clave canónica de dominio
  y la misma ruta del archivo de lock. No protege entre máquinas ni entre procesos con `TMPDIR`
  distinto.
- La auditoría por GID detecta y aborta a una competidora **después** de que el grafo DDS la descubre.
  No garantiza la exclusión antes de cualquier publicación: si dos escritoras con locks distintos se
  descubren tarde, ambas pueden publicar Twist distintos de cero hasta que una vea a la otra. En la
  corrida `10_p2` dos sondas que arrancaron a la vez se rechazaron mutuamente, pero eso fue una
  observación, no una garantía.
- Un observador que llega tarde, un SIGKILL, una caída del proceso o una pérdida del transporte no
  equivalen a un watchdog: el DiffDrive sigue reteniendo el último Twist. Una simulación controlada no
  garantiza una parada física.

En `DIR` deja los flujos crudos (`gt.jsonl`, `odom.jsonl`, `joints.jsonl` y `cmd.jsonl`) y
`summary.json` con métricas y comprobaciones. Cada comprobación tiene estado `PASS`, `FAIL` o
`INSUFFICIENT_EVIDENCE`. Para el reposo de las ruedas solo cuentan las muestras con las dos velocidades
presentes y finitas: una fila con posiciones pero sin velocidad válida no cubre la ventana. La
evaluación nunca extrapola ni interpola a través de huecos mayores que
0.10 s (pose verdadera y `/odom`) o 0.25 s (`/joint_states`); son unas cinco veces el peor hueco
observado en C-007. Exige además cobertura y orden temporal de cada flujo durante toda la secuencia y
en cada ventana de reposo. Si falta un dato que se mide, esa comprobación queda
`INSUFFICIENT_EVIDENCE` con su motivo. Sale con 0 si las 66 comprobaciones pasan y con 1 si alguna
falla o no tiene evidencia suficiente. Las tolerancias, en tiempo simulado:

| Comprobación | Tolerancia | Motivo |
| --- | --- | --- |
| Desplazamiento de comando + reposo, verdadero y odom (esperado v·T = ±0.100 m y ω·T = 0.600 rad) | ±10 %, con piso de 0.01 m / 0.03 rad | Cubre la cuantización de 20 Hz (±0.05 s ≈ ±2.5 %) y el deslizamiento. Es 10⁴ veces mayor que el ruido de la pose en reposo (< 1 µm) |
| Desvío lateral y de yaw en tramos rectos | ≤ 0.01 m / 0.03 rad | Igual |
| Δθ de cada rueda frente a la cinemática (Δx/r ± ω·T·(vía/2)/r) | ±15 % | Muestra que las ruedas giran, no solo la pose |
| Reposo: movimiento verdadero en el último 1 s (0.5 s en el inicial) | ≤ 2 mm, ≤ 0.005 rad, \|ω rueda\| ≤ 0.05 rad/s | Es el 4 % de la velocidad comandada |
| Odom frente a pose verdadera al final de cada fase | 10 % del recorrido comandado acumulado, con piso de 0.01 m / 0.03 rad | Revisada tras medir: la primera corrida usó un límite absoluto de 0.03 rad y falló tras el giro (0.044 rad). Ver la limitación del giro |
| Inclinación y rango de z | ≤ 0.05 rad / ≤ 3 mm | Sin vuelco ni rebote |

`--latch-check` es un diagnóstico: un pulso de 0.5 s, 1.5 s sin mensajes y luego ceros. En C-007 mostró
que el DiffDrive mantiene el último Twist: el robot siguió a 0.05 m/s durante el silencio. No hay
watchdog en Gazebo. La contención es que la sonda escritora termine en cero, lo que no ocurre ante
SIGKILL, una caída del proceso o una pérdida del puente o del transporte. Un árbitro con vencimiento
de mando y una prueba de pérdida de enlace son requisitos antes de autonomía, teleop o integración
física.

### Resultado medido en C-007 (simulación, WSL2, llvmpipe)

Cinco corridas completas de la sonda (headless, con GUI y los dos bloques de arriba) pasaron con la
evaluación original de C-007, que tenía 52 comprobaciones. Tras C-007-P2, otras tres corridas
headless pasaron las 66 comprobaciones nuevas:

| Movimiento (comando + reposo) | Pose verdadera | Odometría |
| --- | --- | --- |
| Avance, esperado +0.100 m | +0.0999 a +0.1001 m | igual a la pose verdadera (< 0.1 mm) |
| Retroceso, esperado −0.100 m | −0.1000 a −0.1001 m | igual a la pose verdadera (< 0.1 mm) |
| Giro, esperado +0.600 rad | +0.644 a +0.646 rad | +0.600 a +0.601 rad |

La traslación en la ventana final de cada reposo quedó por debajo de 4 µm. En una prueba aparte se
interrumpió la sonda a mitad del avance con SIGINT, SIGTERM y SIGHUP. En los tres casos Gazebo recibió
los ceros finales, y el robot recorrió unos 5 mm más, la rampa de frenado, antes de quedar quieto.
Detalles en `evidence/C-007/` (privado).

## Contrato de interfaces de la demo estacionaria (un robot)

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

**C-007:** el DiffDrive se inspira en `urdf/colmibot.urdf:314-326` de Mingyar, y la fricción de los
casters en `:301-309`. Su forma de atributo `<mu1 value=…/>` también la acepta sdformat 14.9 (comprobado
con `gz sdf -p`); aquí se usa la forma de texto. No se tomaron su vía de 0.22 m, su `/cmd_vel` genérico
ni su launch con joystick. Son nuevos en C-007: `urdf/colmibot_motion.urdf.xacro`,
`launch/sim_motion.launch.py`, `config/bridge_motion.yaml`, `rviz/sim_motion.rviz` (adaptado de
`sim.rviz`), `scripts/motion_probe.py` y `test/test_motion_contract.py`.


**Mingyar** (perfil `Mingyar2`), https://github.com/Mingyar2/colmibot_ws, commit
`0aed97231f0affe94050d3ba44c4698547033661`, `robot_description/robot_description/`. Se tomaron
como patrón, reescritos y no copiados como archivo, estos elementos:

- `launch/gazebo_colmi.launch.py`: la inclusión de `gz_sim.launch.py` con `empty.sdf`,
  `robot_state_publisher` con tiempo simulado, `ros_gz_sim create -topic robot_description -name colmibot`
  y los puentes GZ→ROS de `/clock`, `/joint_states` y la cámara.
- `urdf/colmibot.urdf`: el plugin JointStatePublisher (líneas 329-333), el bloque de cámara
  (líneas 217-235: 640×480, R8G8B8, hfov 1.229 rad, clip 0.05–15 m) y el system Sensors con
  `ogre2` (líneas 336-338).

En C-006 no se tomaron de Mingyar: el launch completo (incluye `joy_node`, teleop y el puente
`/cmd_vel` ROS→GZ), el `SetEnvironmentVariable` que sobrescribe `GZ_SIM_RESOURCE_PATH`, el DiffDrive,
los puentes `/odom` y `/tf`, la geometría, las masas, la vía de 0.22 m ni la pose de su cámara. En
C-007 el DiffDrive y los puentes `/odom` y `/tf` sí siguen su patrón (ver la nota de C-007 arriba),
reescritos con los parámetros de Víctor y un tópico de comando propio.

**Víctor Manuel Jiménez González:** `rviz/sim.rviz` adapta `src/robot_description/rviz/display.rviz`
de https://github.com/VicManJ-cloud/colmibot_ws en el commit `985b6b8768e7c44ddb5629a903085039dd937eb5`;
solo se añadió la vista *Camera*, desactivada.

**Nuevo en C-006:** `urdf/colmibot_sim.urdf.xacro`, que envuelve la descripción sin modificarla,
`launch/sim_stationary.launch.py`, `config/bridge_stationary.yaml`, `config/bridge_camera.yaml`,
`test/test_static_contract.py`,
`CMakeLists.txt`, `package.xml` y este README. Cambios frente al patrón de Mingyar: la cámara se
monta en el `camera_link` de Víctor (mira hacia +X) y publica con
`gz_frame_id=camera_optical_frame`, y la tasa baja de 30 a 15 Hz.

Ningún repositorio de origen tiene licencia a nivel de repositorio. Víctor declara MIT en el
`package.xml` de `robot_description`, sin archivo `LICENSE` ni titular. Mingyar declara
`TODO: License declaration` en sus `package.xml` y `Apache-2.0` en sus `setup.py`. Aquí no se
declara ninguna licencia y el campo `<license>` queda pendiente.

## Limitaciones

- La demo estacionaria conserva la caja de colisión `legacy`, que llega al suelo y envuelve ruedas y
  casters. Solo la demo de movimiento usa `collision_mode:=mobile`.
- **Giro en la demo de movimiento:** el robot gira ≈ 7.4 % más de lo que supone la vía de 0.1741 m
  (medido: 0.644 rad para 0.600 comandados). Las llantas son cilindros planos de 12 mm que tocan el
  suelo por sus dos bordes (|y| = 0.081 y 0.093). Durante el giro los sensores de contacto miden fuerzas
  opuestas en los dos bordes y más carga normal en el interno. Se infiere que domina el borde interno:
  la vía efectiva es ≈ 0.162 m, es decir, 2 × 0.081. La odometría, que integra las ruedas con la vía nominal,
  se queda ≈ 0.044 rad corta tras 0.6 rad. En línea recta no hay diferencia medible. No se cambió la vía
  ni la geometría de las ruedas.
- El DiffDrive no tiene watchdog: mantiene el último Twist recibido hasta que llega otro.
- Masas, inercias y posiciones de casters son provisionales; los límites de velocidad y aceleración son
  de demostración.
- Masas e inercias provisionales (CAD sin pesar).
- JointStatePublisher publica en cada paso de física (1 kHz).
- `sdformat` avisa que `gz_frame_id` no está en su esquema, pero lo conserva y la cámara publica
  con ese `frame_id`.
- La colisión `mobile` deja sin cubrir el anillo del cuerpo entre r = 0.079 y 0.100 por debajo de
  62 mm sobre el suelo (ver `SOURCE.md`). Un obstáculo bajo podría tocar el cuerpo real sin contacto
  simulado.
- La sonda envía ceros ante SIGINT, SIGTERM y SIGHUP. Un SIGKILL o un fallo del intérprete la terminan
  sin ceros, y el DiffDrive mantendría el último Twist hasta que se cierre Gazebo.
- Que `/colmibot_sim/cmd_vel` valga cero solo detiene el robot simulado; no prueba una parada física.

## Pruebas

```bash
colcon test --packages-select colmibot_description colmibot_gazebo
colcon test-result --verbose
```

`test_static_contract.py` (C-006) convierte el xacro estacionario con `gz sdf -p` y comprueba que no
hay elementos de movimiento, que los puentes son GZ→ROS, las acciones del launch, la exportación de la
ruta de recursos y el fixed frame de RViz.

`test_motion_contract.py` (C-007) comprueba sin lanzar Gazebo:

- La colisión `mobile`: 13.6 mm sobre el suelo y 2 mm de holgura con las llantas.
- El DiffDrive en URDF y SDF: radio y vía tomados de los joints y colisiones de las ruedas, juntas,
  frames y límites.
- La fricción exacta de cada colisión en el SDF.
- El único puente ROS→GZ.
- El launch, incluido `collision_mode:=mobile`, y el RViz de movimiento.
- Que la sonda es finita, acotada y termina en cero, y que su evaluación detecta un robot atascado o un
  giro fuera de tolerancia.
- C-007-P2:
  - `interp()` no extrapola.
  - La pose verdadera truncada antes del último reposo, los huecos internos y las ventanas sin datos
    dan `INSUFFICIENT_EVIDENCE`, y las muestras fuera de orden dan `FAIL`.
  - Dos endpoints homónimos con distinto GID se cuentan como dos.
  - Una escritora no activa no envía ni ceros.
  - Con el lock tomado, el ejecutable real sale con 2 sin escribir nada.
- C-007-P2B:
  - Velocidades de rueda `NaN`, `None` o ausentes en la ventana final dan `INSUFFICIENT_EVIDENCE`
    aunque las posiciones continúen; también si falta la velocidad de una sola rueda.
  - Tabla de `ROS_DOMAIN_ID` canónico.
  - Con el lock tomado con `87`, una sonda con `+87` o `' 87'` sale con 2.
  - `087`, `010` y `abc` fallan antes de ROS sin crear el lock.
