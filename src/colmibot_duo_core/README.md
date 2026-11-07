# colmibot_duo_core

Núcleo local de **COLMIBOT DUO** (C-002): una misión simbólica con estados explícitos,
evidencia persistente en SQLite y un adaptador ROS 2 mínimo. Todo se prueba con **datos
sintéticos y sin actuación física**.

## Qué es y qué no es

| Es | No es |
| --- | --- |
| Lógica de misión (estados, ruta simbólica, obstáculo, hallazgo) probada por software | Control de movimiento: no calcula velocidades, no usa odometría ni temporizadores |
| Evidencia local reproducible tras reiniciar el proceso | Un paro físico ni un sistema de seguridad certificado |
| Aislamiento lógico de los registros de los robots A y B | Una interfaz con hardware: no abre puertos, no usa joystick, no publica comandos de movimiento |

Un bloqueo `BLOCKED` es un **estado lógico** de esta máquina de estados. No detiene nada
físico. Una verificación aquí es de software y simulación, nunca una demostración con el robot.

## Arquitectura

```
colmibot_duo_core/
  domain/         dominio puro (sin ROS, red ni hardware)
    errors.py       excepciones
    events.py       Event (campos no reasignables), EventType, validaciones
    route.py        StationCatalog (estaciones conocidas) y validación de rutas
    mission.py      Mission: reductor de eventos y tabla de transiciones
  persistence/
    sqlite_store.py EvidenceStore: SQLite estándar, solo-anexar, idempotente, por robot
  service.py      MissionService: casos de uso; valida, persiste y aplica de forma atómica
  adapters/
    command_dispatch.py  mensajes JSON -> MissionService (sin ROS, probado aparte)
    ros_node.py          único módulo que importa ROS (std_msgs/String en ambos sentidos)
```

La dependencia va en un solo sentido: `adapters -> service -> {domain, persistence} -> domain`.
El estado de la misión **se deriva de los eventos** (`Mission.replay`), así que reiniciar el
proceso equivale a releer la evidencia (`MissionService.recover()`).

## Máquina de estados

Estados: `IDLE`, `RUNNING`, `PAUSED`, `BLOCKED`, `COMPLETED`, `CANCELLED`.

| Estado | Evento válido | Resultado |
| --- | --- | --- |
| IDLE | `MISSION_STARTED` | RUNNING |
| IDLE, RUNNING, PAUSED, BLOCKED | `MISSION_CANCELLED` | CANCELLED |
| RUNNING | `MISSION_PAUSED` | PAUSED |
| PAUSED | `MISSION_RESUMED` | RUNNING |
| RUNNING | `STATION_REACHED` (la siguiente de la ruta) | RUNNING; si era la última, se escribe también `MISSION_COMPLETED` en la misma transacción y pasa a COMPLETED |
| RUNNING, PAUSED, BLOCKED | `OBSTACLE_DETECTED` | BLOCKED |
| BLOCKED | `OBSTACLE_CLEARED` (con condiciones) | BLOCKED si queda otro obstáculo abierto; si no, PAUSED |
| RUNNING, PAUSED, BLOCKED, COMPLETED | `FINDING_RECORDED` (en una estación ya confirmada) | sin cambio de estado |

Todo lo demás es `InvalidTransition`, no escribe nada y no cambia el estado. En particular:

- **No hay reanudación automática desde BLOCKED.** Despejar el último obstáculo deja la misión
  en `PAUSED`; hace falta un `MISSION_RESUMED` explícito para volver a `RUNNING`.
- Una lectura booleana "sin obstáculo" (`present=false`) **se ignora**: la ausencia de lectura
  no es una recuperación.
- **Recuperación** (`clear_obstacle`) exige: misión `BLOCKED`, `obstacle_id` abierto,
  `confirmed_by` no vacío y `area_clear` exactamente `true` (un texto `"true"` o un `1` no sirven).
- Solo en `RUNNING` se avanza en la ruta. En `COMPLETED` solo se admiten anotaciones de hallazgo;
  `CANCELLED` no admite nada.
- Un obstáculo reportado en `IDLE` o en un estado terminal se rechaza (no hay movimiento que
  bloquear). Limitación conocida: ese reporte no se conserva.

## Ruta simbólica

`planned_route` se fija al crear la misión (estaciones únicas del catálogo conocido).
`executed_route` crece solo con `STATION_REACHED` confirmados, en orden, uno por uno. Son listas
distintas: la ejecutada nunca se deduce de la planificada. Las estaciones son **nombres**, no
coordenadas.

## Evidencia (SQLite)

Tabla `events`, solo-anexar: triggers impiden `UPDATE` y `DELETE` emitidos por SQL ordinario.
No son una garantía forense: no impiden `INSERT OR REPLACE`, `DROP TRIGGER` ni editar el archivo.

| Columna | Contenido |
| --- | --- |
| `event_id` | Identificador del evento; único **por robot** `(robot_id, event_id)` |
| `robot_id` | `A` o `B` |
| `mission_id` | Único globalmente en el archivo (`MISSION_CREATED`) |
| `ts` | ISO 8601 UTC del reloj inyectado. Etiqueta de registro, no medición física |
| `event_type` | Ver tabla de transiciones |
| `zone` | Estación del evento. En eventos de ciclo de vida es contexto derivado (última estación confirmada, o `NULL`) |
| `image_ref` | Opcional. Referencia opaca; el núcleo **no** genera, valida ni lee imágenes |
| `payload` | JSON canónico. Incluye `origin` (p. ej. `synthetic`) en cada evento |

- **Idempotencia:** reenviar un evento con el mismo `event_id` y el mismo contenido devuelve
  `duplicate` sin efecto; con contenido distinto lanza `EventConflict`. La comparación ignora
  `ts` y la zona derivada. Para que un reintento sea idempotente, quien llama debe reutilizar el
  `event_id` (sin él se genera uno nuevo con `uuid4`). Un `event_id` aportado debe cumplir
  `[A-Za-z0-9][A-Za-z0-9._-]*`, tener a lo sumo 100 caracteres y **no terminar en `.completed`**
  (sufijo reservado para el cierre automático de la misión). Una cadena vacía o `0` es un error,
  no "sin id".
- **Aislamiento A/B:** cada `EvidenceStore` está atado a un `robot_id`; solo lee y escribe sus
  eventos aunque dos robots compartan archivo. Lo recomendado es un archivo por robot
  (`evidence_A.sqlite3`, `evidence_B.sqlite3`).
- **Ubicación de los datos:** siempre fuera del código fuente. Las pruebas usan `tmp_path`;
  `.gitignore` excluye `*.sqlite3` y `*.db` por si acaso.
- Se debe llamar a `recover()` al arrancar; `create_mission` también lo hace si detecta
  misiones persistidas sin recuperar.
- Un solo proceso escritor por archivo y robot. **No se hace cumplir en el código**: dos
  `MissionService` del mismo robot sobre el mismo archivo pueden tener estado en memoria
  obsoleto y aceptar un evento que ya no corresponde. Dos robots distintos sí pueden abrir
  a la vez un archivo nuevo compartido.
- Al recuperar no se vuelve a validar la ruta persistida contra el catálogo de estaciones
  actual: si el catálogo cambia entre ejecuciones, la misión sigue con las estaciones con las que
  se creó. Por coherencia, el reintento de una creación **ya persistida** (mismo `event_id` y mismo
  contenido) se resuelve como `duplicate` sin consultar el catálogo vigente; una creación **nueva**
  sí debe pertenecer a él.

## Adaptador ROS 2

Nodo `mission_node` (ejecutable `mission_node`). Interfaz relativa al nodo, de modo que un
namespace por robot la aísla:

| Dirección | Tópico | Tipo | Contenido |
| --- | --- | --- | --- |
| entrada | `~/command` | `std_msgs/String` | JSON, p. ej. `{"command": "start", "event_id": "..."}` |
| salida | `~/state` | `std_msgs/String` | JSON `{"reply": {...}, "snapshot": {...}}` |

Comandos: `create_mission` (`mission_id`, `route`), `start`, `pause`, `resume`,
`confirm_arrival` (`station`), `obstacle` (`present`, `obstacle_id?`, `detail?`),
`clear_obstacle` (`obstacle_id`, `confirmed_by`, `area_clear`), `finding` (`zone`, `label`,
`note?`, `image_ref?`), `cancel` (`reason?`), `snapshot`. Todos aceptan `event_id`.

Parámetros: `robot_id` (`A`|`B`), `db_dir` (**obligatorio**, fuera del código fuente),
`stations` (catálogo), `origin` (por defecto `synthetic`). **Son de solo lectura tras el arranque**:
se fijan al lanzar el nodo (línea de comandos o archivo de parámetros) y ROS rechaza cambiarlos en
caliente, porque la identidad del robot y el archivo de evidencia se abren una sola vez. Ver `config/mission_node.synthetic.yaml`.

Un texto que no sea JSON, o con una anidación que agote la recursión del decodificador, recibe una
respuesta de error y no cambia nada; el nodo sigue atendiendo comandos.

**Apagado:** `SIGINT`, `SIGTERM` o un `rclpy.shutdown()` externo son una salida normal (código 0):
se cierra el archivo de evidencia y se libera el contexto. Un fallo real de arranque (por ejemplo,
falta `db_dir`) sí termina con error. Si la señal llega justo durante el arranque, el código de
salida sigue siendo 0, aunque `rclpy` puede imprimir en `stderr` un aviso interno de destructor
(`Exception ignored in ... __del__`) que no proviene de este paquete.

Se usa `std_msgs/String` con JSON como simplificación: definir mensajes propios exigiría un
paquete de interfaces, fuera del alcance de C-002. El nodo no publica ningún otro tipo de mensaje.

## Construir y probar

Desde la raíz del workspace (`repo/`), en un shell con solo el underlay de Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select colmibot_duo_core
colcon test --packages-select colmibot_duo_core
colcon test-result --verbose
```

Salidas `build/`, `install/` y `log/` quedan en `repo/` y están ignoradas. Para iterar solo la
lógica: `cd src/colmibot_duo_core && python3 -m pytest -m "not ros" test`.

Las pruebas cubren: matriz completa de transiciones, secuencia de ruta completa, bloqueo y
recuperación explícita, reapertura de SQLite y salida abrupta de un proceso real, reintentos
sin duplicación, aislamiento A/B (archivos separados y archivo compartido), ejecución con
`socket` bloqueado, y comprobaciones estáticas de que el código fuente no contiene nada de
movimiento, puertos seriales ni temporización.

## Decisiones abiertas (para Fernando)

- **Licencia y encabezados de copyright:** `package.xml` y `setup.py` dicen `TODO`. La prueba
  `test_copyright.py` está omitida a propósito hasta decidirlo. Los tres archivos de lint
  (`test_flake8.py`, `test_pep257.py`, `test_copyright.py`) conservan el encabezado Apache-2.0
  de la plantilla estándar de ROS 2.
- **Mantenedor y correo:** marcador `pendiente@example.invalid`.
- **Mensajes propios** en lugar de JSON sobre `String`, cuando se defina la interfaz real.
