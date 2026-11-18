# colmibot_description — origen, créditos y límites

Este paquete es una adaptación local del modelo URDF/xacro de COLMIBOT creado por
**Víctor Manuel Jiménez González**. Se integró en C-006 con su crédito explícito y para el
proyecto académico COLMIBOT DUO. El modelo, la conversión CAD y el cálculo de masa e inercia son
trabajo de Víctor; este paquete solo cambia nombres de paquete y añade empaquetado y pruebas.

## Fuente fija

| Campo | Valor |
| --- | --- |
| Repositorio | https://github.com/VicManJ-cloud/colmibot_ws |
| Commit | `985b6b8768e7c44ddb5629a903085039dd937eb5` (2026-11-08, Víctor Manuel Jiménez González) |
| Ruta upstream | `src/robot_description/` (paquete `ament_cmake` llamado `robot_description`) |
| Descarga verificada | `codeload.github.com/.../tar.gz/<commit>`, SHA-256 `b365d1aa61875b0917b47a3214abf3540b457a6e6faace33f404adb16b7c7bad` el 2026-11-18 |

## Archivos copiados o adaptados

El único cambio en los xacro es `robot_description` → `colmibot_description` en `$(find …)` y en los
URI `package://…`. Los valores numéricos, incluidos los dos archivos `generated/`, no se tocaron.

| Archivo local (= ruta upstream) | Cambio | SHA-256 upstream | SHA-256 local |
| --- | --- | --- | --- |
| `urdf/colmibot.urdf.xacro` | 5 `$(find)` renombrados | `2d98b69c62930fea8b9db30763f497e0ceff675e18e5298fa571150a93f93bd2` | `00bc23cef2e786ac499d3e1a5ccf55f4b74bf39ba50edfb8b114d24c3db0e6fa` |
| `urdf/macros.xacro` | 1 URI de malla renombrado | `0940101cc0b106b5f74b8ff150b95a2113bd028e29b14f1e25a48df6e4e0e217` | `1f0fcc3ee106136ad4d186255f14b4a8f04904207a8b4043d490a3a1f029041c` |
| `urdf/generated/colmibot_meshes.xacro` | 6 URI de malla renombrados | `1471ebfcf55f0cd34239c01852053ebe38365f1ef16a0c6d049af4196868cd8d` | `3adb0db63a562fcbbca771db6dda621002a1f6e82ac495586e4c7dca5407adda` |
| `urdf/colmibot_params.xacro` | ninguno | `47604ed35fbc9b7eacbf3c67a48e516b82b6403bcf953e802dd2b9191007479f` | idéntico |
| `urdf/materials.xacro` | ninguno | `fe889d60c0fa313d195e3fb9aa056fac6ed4ab80dd754a443ee76e322d075119` | idéntico |
| `urdf/generated/colmibot_inertial.xacro` | ninguno | `1dd1a0b6181b21c418e7bdd5c8dc8504b3200fc98f3bafec5c705a01406b2fa2` | idéntico |
| `meshes/*.stl` (8) | ninguno, byte a byte | ver `meshes/SHA256SUMS` | idénticos |

Archivos nuevos de este paquete (C-006): `CMakeLists.txt`, `package.xml`, `SOURCE.md`,
`test/test_model.py`, `meshes/.gitignore` y `meshes/SHA256SUMS`.

No se copiaron, y siguen disponibles en el commit fijo: `tools/` (exportación SolidWorks,
`prepare_meshes.py`, `compute_inertial.py`), `inertia/*.csv` y `launch/display.launch.py` de
`src/robot_description/`, además de `README.md`, `PROCESO.md` y `GUIA_A_MANO.md` de la raíz del
repositorio (las citas de línea de este archivo se refieren a esos documentos de la raíz). La configuración `rviz/display.rviz` se adaptó en
`colmibot_gazebo/rviz/sim.rviz`. Para regenerar `urdf/generated/` hay que usar las herramientas de
Víctor, nunca editar esos archivos a mano.

## Mallas CAD: locales, fuera de Git

Las ocho STL derivan del ensamble CAD que proporcionó el curso. Víctor lo indica en
`README.md:378-379` del upstream. No hay constancia documentada del permiso para redistribuir esos
derivados. Por eso `meshes/.gitignore` ignora todo salvo `.gitignore` y `SHA256SUMS`, y las
mallas no pueden añadirse con un `git add` normal. Solo un `git add -f` explícito las añadiría.

Para recuperarlas desde el commit fijo (por ejemplo, en un clon nuevo), ejecuta esto en Bash:

```bash
cd ~/itam/seminario-de-titulación/colmibot_duo_hibrido/repo/src/colmibot_description/meshes
(
  set -e
  SHA=985b6b8768e7c44ddb5629a903085039dd937eb5
  tmp=$(mktemp -d)
  curl -fsSL -o "$tmp/victor.tar.gz" "https://codeload.github.com/VicManJ-cloud/colmibot_ws/tar.gz/$SHA"
  tar -xzf "$tmp/victor.tar.gz" -C "$tmp" --strip-components=4 \
    --wildcards "colmibot_ws-$SHA/src/robot_description/meshes/*.stl"
  cp "$tmp"/*.stl .
  sha256sum -c SHA256SUMS
  rm -rf "$tmp"
)
```

Después vuelve a compilar el paquete. `test_model.py` falla con un mensaje explícito si falta una
malla o si su hash no coincide.

## Valores provisionales y defectos conocidos

Ningún valor físico se midió en el robot real. Todos se conservan como los generó Víctor.

- **Masa e inercia:** `base_link` 2.25741 kg y ruedas 0.0193581 kg cada una (2.296 kg en total).
  Salen de materiales CAD de marcador que el propio Víctor señala como incorrectos
  (`PROCESO.md:1255-1268`). `inertia/masas.csv` no tiene ningún peso real.
- **Colisión del chasis:** la caja de 0.200 × 0.200 × 0.167 m llega al suelo y envuelve ruedas y
  casters. Fernando la aceptó solo para pruebas estacionarias. Debe corregirse antes de cualquier
  movimiento.
- **Casters:** las esferas de r = 0.012 m en x = ±0.085 m son estimaciones de
  `colmibot_params.xacro`. El CAD sitúa su centro de masa en x = ±0.080 m.
- **Vía entre ruedas:** 0.1741 m, medida entre centros de las mallas de llanta en el CAD, sin
  verificar con calibrador. No se usan los 0.22 m ni la masa de base de 0.084 kg del modelo de
  Mingyar.
- **Cámara, IMU y ToF:** las posiciones vienen del CAD. La cámara mira hacia +X con 6.1° de
  inclinación hacia abajo.

## Licencia

No se declara licencia para este paquete. El `package.xml` upstream dice MIT, pero no incluye
archivo `LICENSE` ni titular, y las mallas derivan de CAD del curso. El campo `<license>` queda como
pendiente hasta que el equipo decida. Esta atribución no concede derechos de redistribución.
