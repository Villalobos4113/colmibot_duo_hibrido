#!/usr/bin/env python3
"""Sonda finita de locomoción SOLO simulada para COLMIBOT en Gazebo (C-007).

Publica una secuencia fija y acotada de Twist en /colmibot_sim/cmd_vel (avance, reposo,
retroceso, reposo, giro, reposo), cada segmento medido en tiempo simulado. Si llegó a ser la
escritora activa, termina con ceros repetidos (try/finally, también ante SIGINT/SIGTERM/SIGHUP o
una excepción; no ante SIGKILL ni una caída del proceso). Mide el efecto en la pose verdadera de
Gazebo (gz-transport, /world/<mundo>/pose/info), en /odom y en /joint_states, y guarda los flujos
crudos (JSONL) y un resumen con métricas y tolerancias (summary.json).

Exclusividad (C-007-P2/P2B): antes de crear el publicador normaliza ROS_DOMAIN_ID a su entero
canónico (rechaza formas ambiguas) y toma un lock local no bloqueante (fcntl.flock) por dominio +
tópico; si otra sonda de esta máquina lo tiene, sale con 2 sin publicar nada, ni ceros. Además audita
el grafo por endpoint (GID), no por nombre de nodo: antes de crear su publicador no debe haber ninguno
en el tópico; después, el único endpoint debe ser el propio (el GID nuevo), y durante la maniobra
cualquier otro endpoint aborta con ceros. Límite: el lock no excluye entre máquinas ni con otro
TMPDIR, y el GID ajeno solo se detecta cuando DDS lo descubre; hasta entonces dos escritoras pueden
publicar. No es un árbitro ni un watchdog.

Antes de publicar algo distinto de cero exige /clock (tiempo simulado), odometría, juntas, pose
verdadera reciente y que el ÚNICO suscriptor del tópico de comando sea el puente simulado
/colmibot_motion_bridge. No toca hardware: el tópico está fijo y no se puede cambiar.

La evaluación no extrapola ni interpola a través de huecos: si falta pose verdadera (u odometría o
juntas) en un intervalo que se mide, esa comprobación queda INSUFFICIENT_EVIDENCE y no pasa.

Uso (con sim_motion.launch.py corriendo en el mismo ROS_DOMAIN_ID y GZ_PARTITION):
  ros2 run colmibot_gazebo motion_probe --out /ruta/salida
  ros2 run colmibot_gazebo motion_probe --out /ruta/salida --latch-check   # diagnóstico

Salida: 0 si todas las comprobaciones pasan, 1 si alguna falla o no tiene evidencia suficiente,
2 si no se cumplieron las precondiciones o la exclusividad (no se publicó ningún Twist y no se
escribe nada en --out).
"""
import argparse
import bisect
import fcntl
import json
import math
import os
import signal
import sys
import tempfile
import threading
import time

CMD_TOPIC = '/colmibot_sim/cmd_vel'
BRIDGE_NODE = 'colmibot_motion_bridge'
MODEL = 'colmibot'
LEFT, RIGHT = 'wheel_left_joint', 'wheel_right_joint'
PUBLISH_HZ = 20.0
# Valores de demostración en simulación (no calibrados en el robot real).
V, W, T = 0.05, 0.30, 2.0
REST = 2.0
# Geometría para las expectativas de las ruedas (mismos valores que el DiffDrive del xacro).
WHEEL_RADIUS, WHEEL_SEPARATION = 0.030, 0.1741

# (fase, vx [m/s], wz [rad/s], duración [s simulados]); vx=None: no publicar (silencio).
SEQUENCE = (
    ('reposo_inicial', 0.0, 0.0, 1.0),
    ('avance', V, 0.0, T),
    ('reposo_avance', 0.0, 0.0, REST),
    ('retroceso', -V, 0.0, T),
    ('reposo_retroceso', 0.0, 0.0, REST),
    ('giro', 0.0, W, T),
    ('reposo_giro', 0.0, 0.0, REST),
)
# Diagnóstico: ¿el DiffDrive mantiene el último Twist cuando dejan de llegar mensajes?
LATCH_SEQUENCE = (
    ('reposo_inicial', 0.0, 0.0, 1.0),
    ('pulso', V, 0.0, 0.5),
    ('silencio', None, None, 1.5),
    ('reposo_final', 0.0, 0.0, REST),
)
FINAL_ZEROS = 10

# Tolerancias (ver README): ±10 % del valor nominal con piso de 1 cm / 0.03 rad.
TOL_REL = 0.10
TOL_LIN = 0.010
TOL_YAW = 0.030
# Error odom-gt: 10 % del recorrido comandado acumulado (trayecto y giro), con los mismos pisos.
# En C-007 una tolerancia absoluta de 0.03 rad falló tras el giro (0.044 rad): el contacto de
# dos bordes de la llanta plana hace girar el robot ~7 % más de lo que supone la odometría.
TOL_ODOM_REL = 0.10
TOL_ODOM_LIN = 0.010
TOL_ODOM_YAW = 0.030
TOL_WHEEL_REL = 0.15
REST_LIN = 0.002      # m en la ventana de reposo
REST_YAW = 0.005      # rad en la ventana de reposo
REST_WHEEL = 0.05     # rad/s
MAX_TILT = 0.05       # rad
MAX_DZ = 0.003        # m
# Hueco máximo entre muestras consecutivas para medir en un intervalo (s simulados). Observado en
# C-007: pose verdadera ≈ 60 Hz (hueco máx. 0.019 s), /odom 50 Hz (0.020 s), /joint_states 1 kHz
# (un hueco aislado de 0.088 s). Los límites dejan ≈ 5 veces el peor hueco observado.
MAX_GAP = {'gt': 0.10, 'odom': 0.10, 'joints': 0.25}
GT_STALE = 0.5          # s simulados sin pose verdadera durante la maniobra: abortar con ceros
DISCOVERY_SETTLE = 1.0  # s de reloj que el grafo debe mantenerse estable antes de escribir
GRAPH_AUDIT_PERIOD = 0.5  # s de reloj entre auditorías del grafo durante la maniobra
OWN_ENDPOINT_TIMEOUT = 10.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_yaw_tilt(x, y, z, w):
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    tilt = math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))
    return yaw, tilt


def interp(rows, t, cols, max_gap=None):
    """Interpolación lineal de las columnas `cols` de filas {'t': ...} ordenadas por t.

    Nunca extrapola: devuelve None si t está fuera de [primera, última] muestra, o si las dos
    muestras que rodean t están separadas más de `max_gap`.
    """
    if not rows:
        return None
    times = [r['t'] for r in rows]
    if t < times[0] or t > times[-1]:
        return None
    i = bisect.bisect_left(times, t)
    if times[i] == t:
        return [rows[i][c] for c in cols]
    a, b = rows[i - 1], rows[i]
    if max_gap is not None and b['t'] - a['t'] > max_gap:
        return None
    f = (t - a['t']) / (b['t'] - a['t'])
    out = []
    for c in cols:
        if c == 'yaw':
            out.append(wrap(a[c] + f * wrap(b[c] - a[c])))
        else:
            out.append(a[c] + f * (b[c] - a[c]))
    return out


def coverage(times, t0, t1, max_gap):
    """¿Hay muestras que cubren [t0, t1] sin huecos mayores que max_gap? -> (ok, motivo, n, hueco).

    `times` ordenados. Exige una muestra <= t0 y otra >= t1 (sin extrapolar) y que ningún par de
    muestras consecutivas entre ellas esté separado más de max_gap.
    """
    if not times:
        return False, 'sin muestras', 0, None
    if times[0] > t0:
        return False, f'primera muestra t={times[0]:.3f} posterior al inicio {t0:.3f}', 0, None
    if times[-1] < t1:
        return False, f'última muestra t={times[-1]:.3f} anterior al final {t1:.3f}', 0, None
    a = bisect.bisect_right(times, t0) - 1
    b = bisect.bisect_left(times, t1)
    span = times[a:b + 1]
    gap = max((y - x for x, y in zip(span, span[1:])), default=0.0)
    n = sum(1 for t in span if t0 <= t <= t1)
    if gap > max_gap:
        i = max(range(len(span) - 1), key=lambda k: span[k + 1] - span[k])
        return False, f'hueco de {gap:.3f} s entre t={span[i]:.3f} y {span[i + 1]:.3f}', n, gap
    return True, '', n, gap


def rel_pose(p0, p1):
    """Pose p1 (x, y, yaw) en el marco de p0."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2]))


def _as_dicts(rows, keys):
    return [dict(zip(keys, r)) for r in rows]


def _finite(value):
    """Número real finito (no None, no bool, no NaN ni infinito)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def evaluate(data, phases, latch=False):
    """Métricas y comprobaciones a partir de los flujos crudos. Devuelve (checks, metrics).

    Cada comprobación lleva `status` PASS, FAIL o INSUFFICIENT_EVIDENCE y `pass` (solo True con
    PASS). Una medición cuyos intervalos no están cubiertos por muestras (sin extrapolar y sin
    huecos > MAX_GAP) queda INSUFFICIENT_EVIDENCE con el motivo en `reason`.
    """
    keys = {'gt': ('t', 'x', 'y', 'z', 'yaw', 'tilt'), 'odom': ('t', 'x', 'y', 'yaw', 'vx', 'wz'),
            'joints': ('t', 'pl', 'pr', 'vl', 'vr')}
    raw_t = {k: [r[0] for r in data[k]] for k in keys}
    rows = {k: sorted(_as_dicts(data[k], keys[k]), key=lambda r: r['t']) for k in keys}
    times = {k: [r['t'] for r in rows[k]] for k in keys}
    gt, od, js = rows['gt'], rows['odom'], rows['joints']
    checks, metrics = [], {'phases': []}

    def check(name, value, ok, limit):
        checks.append({'check': name, 'value': value, 'limit': limit, 'pass': bool(ok),
                       'status': 'PASS' if ok else 'FAIL'})

    def insufficient(name, limit, reason):
        checks.append({'check': name, 'value': None, 'limit': limit, 'pass': False,
                       'status': 'INSUFFICIENT_EVIDENCE', 'reason': reason})

    def covered(stream, t0, t1):
        return coverage(times[stream], t0, t1, MAX_GAP[stream])

    def pose(stream, t):
        return interp(rows[stream], t, ('x', 'y', 'yaw'), MAX_GAP[stream])

    def why(*spans):
        """Motivo de la primera falta de cobertura en (stream, t0, t1), o '' si todo está cubierto."""
        for stream, t0, t1 in spans:
            ok, reason, _, _ = covered(stream, t0, t1)
            if not ok:
                return f'{stream} [{t0:.3f}, {t1:.3f}]: {reason}'
        return ''

    t_start, t_end = phases[0]['t0'], phases[-1]['t1']
    # Orden temporal y cobertura global de cada flujo durante la secuencia.
    for stream in keys:
        back = sum(1 for a, b in zip(raw_t[stream], raw_t[stream][1:]) if b < a)
        check(f'{stream}: orden temporal (retrocesos de t)', back, back == 0, 0)
        ok, reason, n, gap = covered(stream, t_start, t_end)
        if ok:
            check(f'{stream}: cobertura de la secuencia, hueco máx (s)', gap, True, MAX_GAP[stream])
        else:
            insufficient(f'{stream}: cobertura de la secuencia, hueco máx (s)', MAX_GAP[stream], reason)
    run = [r for r in gt if t_start <= r['t'] <= t_end]
    metrics['samples'] = {'gt': len(gt), 'odom': len(od), 'joints': len(js),
                          'gt_in_run': len(run), 'sim_duration': t_end - t_start}
    if run:
        metrics['max_tilt'] = max(r['tilt'] for r in run)
        metrics['z_range'] = max(r['z'] for r in run) - min(r['z'] for r in run)
    if why(('gt', t_start, t_end)) or not run:
        reason = why(('gt', t_start, t_end)) or 'sin pose verdadera en la secuencia'
        insufficient('inclinación máxima (rad)', MAX_TILT, reason)
        insufficient('rango de z (m)', MAX_DZ, reason)
    else:
        check('inclinación máxima (rad)', metrics['max_tilt'], metrics['max_tilt'] <= MAX_TILT, MAX_TILT)
        check('rango de z (m)', metrics['z_range'], metrics['z_range'] <= MAX_DZ, MAX_DZ)
    gt0, od0 = pose('gt', t_start), pose('odom', t_start)

    path, turn = 0.0, 0.0   # recorrido comandado acumulado
    for i, ph in enumerate(phases):
        entry = dict(ph)
        name = ph['name']
        if ph['vx'] is not None:
            path += abs(ph['vx']) * (ph['t1'] - ph['t0'])
            turn += abs(ph['wz']) * (ph['t1'] - ph['t0'])
        tol_lin = max(TOL_ODOM_LIN, TOL_ODOM_REL * path)
        tol_yaw = max(TOL_ODOM_YAW, TOL_ODOM_REL * turn)
        g_a, g_b, o_b = pose('gt', ph['t0']), pose('gt', ph['t1']), pose('odom', ph['t1'])
        missing = why(('gt', t_start, ph['t1']), ('odom', t_start, ph['t1']))
        if missing or None in (gt0, od0, g_a, g_b, o_b):
            missing = missing or 'pose no interpolable sin extrapolar'
            insufficient(f'{name}: |odom-gt| posición al final (m)', tol_lin, missing)
            insufficient(f'{name}: |odom-gt| yaw al final (rad)', tol_yaw, missing)
        else:
            entry['gt_start'] = rel_pose(gt0, g_a)
            entry['gt_end'] = rel_pose(gt0, g_b)
            entry['odom_end'] = rel_pose(od0, o_b)
            err = [entry['odom_end'][k] - entry['gt_end'][k] for k in range(3)]
            err[2] = wrap(err[2])
            entry['odom_minus_gt_end'] = err
            check(f'{name}: |odom-gt| posición al final (m)', math.hypot(err[0], err[1]),
                  math.hypot(err[0], err[1]) <= tol_lin, tol_lin)
            check(f'{name}: |odom-gt| yaw al final (rad)', abs(err[2]), abs(err[2]) <= tol_yaw, tol_yaw)
        if ph['vx'] == 0.0 and ph['wz'] == 0.0:
            win = min(1.0, 0.5 * (ph['t1'] - ph['t0']))
            w0, w1 = ph['t1'] - win, ph['t1']
            entry['rest_window'] = win
            label = f'{name}: en último {win:.1f} s'
            for stream in ('gt', 'joints'):
                ok, reason, n, gap = covered(stream, w0, w1)
                if ok:
                    check(f'{label}: cobertura {stream}, muestras', n, True, f'hueco <= {MAX_GAP[stream]} s')
                else:
                    insufficient(f'{label}: cobertura {stream}, muestras', f'hueco <= {MAX_GAP[stream]} s', reason)
            a, b = pose('gt', w0), pose('gt', w1)
            missing = why(('gt', w0, w1))
            if missing or a is None or b is None:
                missing = missing or 'pose no interpolable sin extrapolar'
                insufficient(f'{name}: traslación en último {win:.1f} s (m)', REST_LIN, missing)
                insufficient(f'{name}: giro en último {win:.1f} s (rad)', REST_YAW, missing)
            else:
                d = rel_pose(a, b)
                entry['rest_gt_motion'] = d
                check(f'{name}: traslación en último {win:.1f} s (m)', math.hypot(d[0], d[1]),
                      math.hypot(d[0], d[1]) <= REST_LIN, REST_LIN)
                check(f'{name}: giro en último {win:.1f} s (rad)', abs(d[2]), abs(d[2]) <= REST_YAW, REST_YAW)
            # Solo cuentan las muestras con las DOS velocidades presentes y finitas: la cobertura y el
            # máximo se calculan sobre ellas (una fila con posiciones pero sin velocidad no cubre nada).
            valid = [r for r in js if _finite(r.get('vl')) and _finite(r.get('vr'))]
            ok, reason, _, _ = coverage([r['t'] for r in valid], w0, w1, MAX_GAP['joints'])
            win_js = [r for r in valid if w0 <= r['t'] <= w1]
            if not ok or not win_js:
                # Sin velocidades válidas no se puede afirmar el reposo de las ruedas.
                insufficient(f'{name}: |vel. rueda| máx en último {win:.1f} s (rad/s)', REST_WHEEL,
                             f'velocidades de rueda válidas [{w0:.3f}, {w1:.3f}]: '
                             f'{reason or "sin muestras en la ventana"}')
                entry['rest_max_wheel_speed'] = None
            else:
                wheel = max(max(abs(r['vl']), abs(r['vr'])) for r in win_js)
                entry['rest_max_wheel_speed'] = wheel
                check(f'{name}: |vel. rueda| máx en último {win:.1f} s (rad/s)', wheel,
                      wheel <= REST_WHEEL, REST_WHEEL)
        elif ph['vx'] is not None and not latch:
            # Fase con comando + reposo siguiente: las rampas de aceleración se compensan.
            nxt = phases[i + 1]
            dur = ph['t1'] - ph['t0']
            exp = (ph['vx'] * dur, 0.0, ph['wz'] * dur)
            tol_x = max(TOL_LIN, TOL_REL * abs(exp[0]))
            tol_yaw = max(TOL_YAW, TOL_REL * abs(exp[2]))
            entry['expected_cmd_plus_rest'] = exp
            span = (ph['t0'], nxt['t1'])
            for src in ('gt', 'odom'):
                p0, p1 = pose(src, span[0]), pose(src, span[1])
                missing = why((src, *span))
                names = (f'{name}: {src} Δx cuerpo (m) vs {exp[0]:+.4f}', f'{name}: {src} Δy cuerpo (m) vs 0',
                         f'{name}: {src} Δyaw (rad) vs {exp[2]:+.4f}')
                if missing or p0 is None or p1 is None:
                    missing = missing or 'pose no interpolable sin extrapolar'
                    for nm, lim in zip(names, (tol_x, TOL_LIN, tol_yaw)):
                        insufficient(nm, lim, missing)
                    continue
                d = rel_pose(p0, p1)
                entry[f'{src}_delta_cmd_plus_rest'] = d
                if src == 'gt':
                    entry['gt_gain'] = [g / e if e else None for g, e in ((d[0], exp[0]), (d[2], exp[2]))]
                check(names[0], d[0], abs(d[0] - exp[0]) <= tol_x, tol_x)
                check(names[1], d[1], abs(d[1]) <= TOL_LIN, TOL_LIN)
                check(names[2], d[2], abs(d[2] - exp[2]) <= tol_yaw, tol_yaw)
            half = WHEEL_SEPARATION / 2.0
            exp_l = (exp[0] - exp[2] * half) / WHEEL_RADIUS
            exp_r = (exp[0] + exp[2] * half) / WHEEL_RADIUS
            entry['wheel_expected'] = (exp_l, exp_r)
            j0 = interp(js, span[0], ('pl', 'pr'), MAX_GAP['joints'])
            j1 = interp(js, span[1], ('pl', 'pr'), MAX_GAP['joints'])
            missing = why(('joints', *span))
            for k, (side, exp_w) in enumerate((('izq', exp_l), ('der', exp_r))):
                nm, tol = f'{name}: Δθ rueda {side} (rad) vs {exp_w:+.3f}', TOL_WHEEL_REL * abs(exp_w)
                if missing or j0 is None or j1 is None:
                    insufficient(nm, tol, missing or 'juntas no interpolables sin extrapolar')
                else:
                    check(nm, j1[k] - j0[k], abs(j1[k] - j0[k] - exp_w) <= tol, tol)
            if j0 is not None and j1 is not None:
                entry['wheel_delta'] = (j1[0] - j0[0], j1[1] - j0[1])
        metrics['phases'].append(entry)
    if latch:
        sil = next(p for p in phases if p['name'] == 'silencio')
        a, b = pose('gt', sil['t0']), pose('gt', sil['t1'])
        missing = why(('gt', sil['t0'], sil['t1']))
        dur = sil['t1'] - sil['t0']
        if missing or a is None or b is None:
            insufficient('silencio: pose verdadera', MAX_GAP['gt'], missing or 'pose no interpolable')
        else:
            d = rel_pose(a, b)
            metrics['latch'] = {'silence_duration': dur, 'gt_delta_during_silence': d,
                                'latched_speed_estimate': d[0] / dur if dur > 0 else None,
                                'latch_observed': d[0] > 0.5 * V * dur}
    return checks, metrics


MAX_DOMAIN_ID = 232   # mayor dominio válido para los puertos DDS con rmw_fastrtps


def canonical_domain_id(text):
    """ROS_DOMAIN_ID -> (dominio entero, None) o (None, motivo). Falla cerrado ante formas dudosas.

    Jazzy (rcl, strtoul en base 0) da 87 para '87', '+87' y ' 87', pero lee '010' como octal (8),
    '0x57' como 87 y rechaza '087'. Aquí solo se aceptan decimales sin ceros a la izquierda (salvo
    '0'), con '+' y espacios ASCII opcionales (' \\t\\n\\r\\v\\f', los que salta strtoul): para todos
    ellos rcl usa el mismo número. Sin definir o exactamente '' es 0, como en Jazzy; solo espacios
    (' ') o cualquier carácter no ASCII (NBSP, etc.) los rechaza rcl o son ambiguos, y aquí también
    (C-007-P2C, sin str.strip() Unicode). rcl rechaza '0 ' (con 0 exige fin de cadena), así que
    los espacios finales solo se aceptan con dominio distinto de 0. Todo lo demás se rechaza antes
    de crear nada de ROS.
    """
    if text is None or text == '':
        return 0, None
    if not isinstance(text, str) or not text.isascii():
        return None, f'ROS_DOMAIN_ID={text!r} contiene caracteres no ASCII'
    body = text.lstrip(' \t\n\r\v\f')
    raw = body.rstrip(' \t\n\r\v\f')
    if raw == '':
        return None, f'ROS_DOMAIN_ID={text!r} contiene solo espacios (rcl lo rechaza)'
    digits = raw[1:] if raw.startswith('+') else raw
    if not digits.isdigit() or not digits.isascii():
        return None, f'ROS_DOMAIN_ID={text!r} no es un entero decimal'
    if len(digits) > 1 and digits.startswith('0'):
        return None, f'ROS_DOMAIN_ID={text!r} tiene ceros a la izquierda (rcl lo leería en octal o lo rechaza)'
    value = int(digits)
    if value == 0 and raw != body:
        return None, f'ROS_DOMAIN_ID={text!r} es 0 con espacios finales (rcl lo rechaza)'
    if value > MAX_DOMAIN_ID:
        return None, f'ROS_DOMAIN_ID={value} fuera de 0..{MAX_DOMAIN_ID}'
    return value, None


def lock_path(domain, topic):
    """Ruta del lock local para el dominio canónico (entero) + tópico (respeta TMPDIR)."""
    if not isinstance(domain, int) or isinstance(domain, bool):
        raise TypeError('lock_path requiere el dominio canónico entero (canonical_domain_id)')
    return os.path.join(tempfile.gettempdir(),
                        f'colmibot_motion_probe.domain{domain}{topic.replace("/", "_")}.lock')


def acquire_writer_lock(domain, topic):
    """Lock exclusivo no bloqueante. Devuelve (fd, ruta, error). Nunca borra ni espera locks ajenos."""
    path = lock_path(domain, topic)
    try:
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    except OSError as exc:
        return None, path, f'no se pudo abrir el lock: {exc}'
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        holder = os.pread(fd, 200, 0).decode(errors='replace').strip()
        os.close(fd)
        return None, path, f'otra sonda tiene el lock ({holder or "sin datos"})'
    os.ftruncate(fd, 0)
    os.pwrite(fd, f'pid {os.getpid()}\n'.encode(), 0)
    return fd, path, ''


def release_writer_lock(fd):
    if fd is not None:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)


def gid_of(endpoint):
    return bytes(endpoint.endpoint_gid).hex()


def audit_publishers(endpoints, own_gid=None):
    """Endpoints publicadores reales (por GID, no por nombre). -> (propio_visible, otros)."""
    gids = [(gid_of(e), f'{e.node_namespace.rstrip("/")}/{e.node_name}') for e in endpoints]
    own = own_gid is not None and any(g == own_gid for g, _ in gids)
    return own, sorted(f'{name} gid={g[:12]}' for g, name in gids if g != own_gid)


class CommandWriter:
    """Único camino para publicar Twist. Antes de activate() no publica nada, ni ceros."""

    def __init__(self, publish_fn):
        self._publish = publish_fn
        self.active = False
        self.sent = 0

    def activate(self):
        self.active = True

    def send(self, phase, vx, wz):
        if not self.active:
            raise RuntimeError('esta instancia no es la escritora activa: no publica')
        self._publish(phase, vx, wz)
        self.sent += 1

    def close_with_zeros(self, n, between=None):
        """Ceros finales solo si esta instancia llegó a ser la escritora activa."""
        if not self.active:
            return 0
        for _ in range(n):
            self.send('cero_final', 0.0, 0.0)
            if between:
                between()
        return n


class PreconditionError(Exception):
    """Precondición o exclusividad no cumplida antes de ser escritora activa."""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--out', required=True, help='directorio de salida (se crea)')
    parser.add_argument('--world', default='empty')
    parser.add_argument('--ready-timeout', type=float, default=60.0, help='segundos de reloj')
    parser.add_argument('--latch-check', action='store_true',
                        help='diagnóstico: pulso, silencio sin mensajes y ceros')
    args = parser.parse_args(argv)

    # 1. Lock local ANTES de crear cualquier entidad ROS. Si falla: salida 2, sin publicador,
    #    sin Twist, sin ceros y sin escribir en --out (podría ser el directorio de la otra sonda).
    domain, domain_error = canonical_domain_id(os.environ.get('ROS_DOMAIN_ID'))
    if domain_error:
        print(f'ERROR: {domain_error}. No se creó lock ni publicador; no se publicó nada.', file=sys.stderr)
        return 2
    lock_fd, lock_file, lock_error = acquire_writer_lock(domain, CMD_TOPIC)
    if lock_fd is None:
        print(f'ERROR: exclusividad no adquirida ({lock_file}): {lock_error}. '
              'No se publicó nada.', file=sys.stderr)
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode

    sequence = LATCH_SEQUENCE if args.latch_check else SEQUENCE
    lock = threading.RLock()   # reentrante: gt_age() se consulta con el lock tomado
    data = {'gt': [], 'odom': [], 'joints': [], 'cmd': []}
    stop = {'signal': None}

    def on_signal(signum, _frame):
        stop['signal'] = signum

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # SIGHUP: cerrar la terminal. SIGKILL no se puede atrapar: el DiffDrive mantendría el último Twist.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    node = rclpy.create_node('colmibot_motion_probe', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    pub = None

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw, _ = quat_yaw_tilt(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with lock:
            data['odom'].append((t, p.x, p.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            data.setdefault('odom_frames', {(msg.header.frame_id, msg.child_frame_id)})
            data['odom_frames'].add((msg.header.frame_id, msg.child_frame_id))

    def on_joints(msg):
        if LEFT not in msg.name or RIGHT not in msg.name:
            return
        il, ir = msg.name.index(LEFT), msg.name.index(RIGHT)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vel = list(msg.velocity) or [None] * len(msg.name)
        with lock:
            data['joints'].append((t, msg.position[il], msg.position[ir], vel[il], vel[ir]))

    def on_gt(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == MODEL:
                o = p.orientation
                yaw, tilt = quat_yaw_tilt(o.x, o.y, o.z, o.w)
                with lock:
                    data['gt'].append((t, p.position.x, p.position.y, p.position.z, yaw, tilt))
                return

    node.create_subscription(Odometry, '/odom', on_odom, 50)
    node.create_subscription(JointState, '/joint_states', on_joints, 100)
    gz_node = GzNode()
    gt_topic = f'/world/{args.world}/pose/info'

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def raw_publish(phase, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        pub.publish(msg)
        with lock:
            data['cmd'].append((sim_now(), phase, vx, wz))

    def gt_age():
        with lock:
            return sim_now() - data['gt'][-1][0] if data['gt'] else float('inf')

    def publishers():
        return node.get_publishers_info_by_topic(CMD_TOPIC)

    writer = CommandWriter(raw_publish)
    phases, status, reason = [], 2, ''
    try:
        if not gz_node.subscribe(Pose_V, gt_topic, on_gt):
            raise PreconditionError(f'no se pudo suscribir a {gt_topic}')
        # 2. Precondiciones con el grafo estable DISCOVERY_SETTLE s. Aún no existe publicador
        #    propio, así que cualquier publicador del tópico es ajeno (sin importar su nombre).
        deadline = time.monotonic() + args.ready_timeout
        stable_since = None
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            subs = node.get_subscriptions_info_by_topic(CMD_TOPIC)
            names = sorted(f'{e.node_namespace.rstrip("/")}/{e.node_name}' for e in subs)
            _, others = audit_publishers(publishers())
            if others:   # rechazo inmediato: no se espera a que el canal "quede libre"
                raise PreconditionError(f'otro publicador en {CMD_TOPIC} antes de escribir: {others}')
            with lock:
                ready = {'clock': sim_now() > 0.0, 'gt': gt_age() <= GT_STALE, 'odom': bool(data['odom']),
                         'joints': bool(data['joints']), 'bridge': names == ['/' + BRIDGE_NODE],
                         'sin_otros_publicadores': not others}
            if all(ready.values()):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= DISCOVERY_SETTLE:
                    break
            else:
                stable_since = None
            if stop['signal'] or time.monotonic() > deadline:
                raise PreconditionError(f'precondiciones no cumplidas: {ready}; suscriptores de '
                                        f'{CMD_TOPIC}: {names}; otros publicadores: {others}')
        # 3. Crear el publicador y reconocer el endpoint propio por diferencia de grafo: antes no
        #    había ninguno, así que debe aparecer exactamente uno nuevo y mantenerse único.
        pub = node.create_publisher(Twist, CMD_TOPIC, 10)
        own_gid, deadline, stable_since = None, time.monotonic() + OWN_ENDPOINT_TIMEOUT, None
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            eps = publishers()
            if own_gid is None and len(eps) == 1:
                own_gid = gid_of(eps[0])
            own, others = audit_publishers(eps, own_gid)
            if len(eps) > 1 or others:
                raise PreconditionError(f'hay otro publicador en {CMD_TOPIC}: {others or len(eps)} endpoints')
            if own:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= DISCOVERY_SETTLE:
                    break
            if stop['signal'] or time.monotonic() > deadline:
                raise PreconditionError('el endpoint propio no apareció o no se mantuvo único')
        # 4. Escritora activa: desde aquí, y solo desde aquí, se publica (y habrá ceros finales).
        writer.activate()
        print(f'listo: suscriptores de {CMD_TOPIC} = {names}; publicador propio gid={own_gid[:12]} '
              f'único; lock {lock_file}; t_sim = {sim_now():.3f}', flush=True)
        status = 1
        period = 1.0 / PUBLISH_HZ
        next_audit = time.monotonic() + GRAPH_AUDIT_PERIOD
        for name, vx, wz, dur in sequence:
            t0 = sim_now()
            phase = {'name': name, 'vx': vx, 'wz': wz, 'duration_cmd': dur, 't0': t0}
            phases.append(phase)
            print(f'fase {name}: vx={vx} wz={wz} durante {dur} s simulados (t0={t0:.3f})', flush=True)
            next_pub, last_t, last_wall = t0, t0, time.monotonic()
            while not stop['signal']:
                rclpy.spin_once(node, timeout_sec=0.005)
                t = sim_now()
                if t >= t0 + dur:
                    break
                if t > last_t:
                    last_t, last_wall = t, time.monotonic()
                elif time.monotonic() - last_wall > 5.0:
                    raise RuntimeError('el tiempo simulado no avanza (¿Gazebo en pausa o cerrado?)')
                if gt_age() > GT_STALE:
                    raise RuntimeError(f'sin pose verdadera de Gazebo desde hace {gt_age():.2f} s simulados')
                if time.monotonic() >= next_audit:
                    _, others = audit_publishers(publishers(), own_gid)
                    if others:
                        raise RuntimeError(f'apareció otro publicador en {CMD_TOPIC}: {others}')
                    next_audit = time.monotonic() + GRAPH_AUDIT_PERIOD
                if vx is not None and t >= next_pub:
                    writer.send(name, vx, wz)
                    next_pub = max(next_pub + period, t)
            phase['t1'] = sim_now()
            if stop['signal']:
                reason = f'interrumpida por señal {stop["signal"]}'
                break
        else:
            status = 0
    except PreconditionError as exc:
        reason = f'precondición: {exc}'
        print(f'ERROR: {reason}. No se publicó nada.', file=sys.stderr)
        status = 2
    except Exception as exc:  # noqa: B902 - se registra y se cierra con ceros si era escritora
        reason = f'excepción: {exc!r}'
        print('ERROR: ' + reason, file=sys.stderr)
        status = 1
    finally:
        # Ceros repetidos solo si esta instancia llegó a ser la escritora activa: una sonda
        # rechazada nunca interfiere con otra enviando ceros.
        zeros = writer.close_with_zeros(FINAL_ZEROS, lambda: rclpy.spin_once(node, timeout_sec=0.05))
        if zeros:
            t_settle = time.monotonic() + 0.5
            while time.monotonic() < t_settle:
                rclpy.spin_once(node, timeout_sec=0.05)
        with lock:
            snapshot = {k: list(v) for k, v in data.items() if k != 'odom_frames'}
            odom_frames = sorted(data.get('odom_frames', set()))
        node.destroy_node()
        rclpy.try_shutdown()
    if status == 2:
        release_writer_lock(lock_fd)
        return 2

    os.makedirs(args.out, exist_ok=True)
    for key in ('gt', 'odom', 'joints', 'cmd'):
        with open(os.path.join(args.out, f'{key}.jsonl'), 'w') as f:
            for row in snapshot[key]:
                f.write(json.dumps(row) + '\n')
    summary = {'sequence': 'latch-check' if args.latch_check else 'demo', 'cmd_topic': CMD_TOPIC,
               'reason': reason, 'odom_frames': odom_frames, 'phases': phases,
               'cmd_count': len(snapshot['cmd']), 'writer_lock': lock_file}
    if status == 0:
        checks, metrics = evaluate(snapshot, phases, latch=args.latch_check)
        if odom_frames != [('odom', 'base_footprint')]:
            checks.append({'check': 'frames de /odom', 'value': odom_frames,
                           'limit': [('odom', 'base_footprint')], 'pass': False, 'status': 'FAIL'})
        summary.update(checks=checks, metrics=metrics)
        status = 0 if all(c['pass'] for c in checks) else 1
        for c in checks:
            value = c['value']
            shown = f'{value:+.5f}' if isinstance(value, float) else str(value)
            extra = f' [{c["reason"]}]' if c.get('reason') else ''
            print(f'{c.get("status", "PASS" if c["pass"] else "FAIL")}  {c["check"]}: {shown} '
                  f'(límite {c["limit"]}){extra}')
        for entry in metrics['phases']:
            if 'gt_gain' in entry:
                gx, gyaw = entry['gt_gain']
                print(f'medido {entry["name"]}: Δ verdadero / comandado: x={gx and round(gx, 4)} '
                      f'yaw={gyaw and round(gyaw, 4)}')
        if 'latch' in metrics:
            print('latch:', json.dumps(metrics['latch']))
    summary['status'] = status
    with open(os.path.join(args.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)
    print(f'resultado: {"PASS" if status == 0 else "FAIL"} (salida {status}); datos en {args.out}')
    release_writer_lock(lock_fd)
    return status


if __name__ == '__main__':
    sys.exit(main())
