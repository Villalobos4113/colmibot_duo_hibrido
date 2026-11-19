"""Contrato estático de la locomoción simulada de colmibot_gazebo (C-007).

Sin lanzar Gazebo comprueba: colisión mobile sin contacto con el suelo ni con las ruedas;
DiffDrive nativo de Harmonic con radio y vía tomados de la misma geometría que posiciona las
ruedas; fricción de casters en el SDF generado por `gz sdf -p`; un único puente ROS->GZ en
/colmibot_sim/cmd_vel; odometría y odom->base_footprint GZ->ROS; launch separado que pide
collision_mode:=mobile; y que la sonda de prueba es finita y termina en cero.
"""
import ast
import importlib.machinery
import importlib.util
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest
import xacro
import yaml

SHARE = get_package_share_directory('colmibot_gazebo')
MOTION_XACRO = os.path.join(SHARE, 'urdf', 'colmibot_motion.urdf.xacro')
SIM_XACRO = os.path.join(SHARE, 'urdf', 'colmibot_sim.urdf.xacro')
DESC_XACRO = os.path.join(get_package_share_directory('colmibot_description'), 'urdf',
                          'colmibot.urdf.xacro')
PROBE = os.path.join(get_package_prefix('colmibot_gazebo'), 'lib', 'colmibot_gazebo', 'motion_probe')
SIM_CMD = '/colmibot_sim/cmd_vel'
GZ_CMD = '/model/colmibot/cmd_vel'
FORBIDDEN = ('joy', 'teleop', '/dev/', 'joint_state_publisher_gui', 'serial')
CLEARANCE = 0.0136   # despeje del CAD: z mínimo del cuerpo sin ruedas ni casters (-0.0164)
GAP = 0.002


def _floats(text):
    return [float(v) for v in text.split()]


def _urdf(path=MOTION_XACRO, **mappings):
    return ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())


def _sdf(robot):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(ET.tostring(robot, encoding='unicode'))
    try:
        result = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True,
                                timeout=60)
    finally:
        os.unlink(f.name)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def _origin(element):
    origin = element.find('origin')
    xyz = _floats(origin.get('xyz', '0 0 0')) if origin is not None else [0.0] * 3
    return xyz


@pytest.fixture(scope='module')
def motion():
    return _urdf()


@pytest.fixture(scope='module')
def sdf(motion):
    return _sdf(motion)


def _wheels(robot):
    """Posición, radio y ancho de cada rueda según su joint y su colisión en el URDF."""
    out = {}
    for side in ('left', 'right'):
        joint = robot.find(f"joint[@name='wheel_{side}_joint']")
        cyl = robot.find(f"link[@name='wheel_{side}_link']/collision/geometry/cylinder")
        out[side] = (_origin(joint), float(cyl.get('radius')), float(cyl.get('length')))
    return out


def _diffdrive(tree):
    plugins = [p for p in tree.iter('plugin') if p.get('name') == 'gz::sim::systems::DiffDrive']
    assert len(plugins) == 1
    return plugins[0]


def test_mobile_collision_clears_ground_and_wheels(motion):
    wheels = _wheels(motion)
    base_z = _origin(motion.find("joint[@name='base_joint']"))[2]
    collisions = motion.findall("link[@name='base_link']/collision")
    named = {c.get('name'): c for c in collisions if c.get('name')}
    assert set(named) == {'chassis_lower', 'chassis_upper'}
    assert not motion.findall("link[@name='base_link']/collision/geometry/box")
    inner = min(abs(xyz[1]) - width / 2 for xyz, _, width in wheels.values())
    wheel_top = max(xyz[2] + radius for xyz, radius, _ in wheels.values())
    spans = {}
    for name, col in named.items():
        cyl = col.find('geometry/cylinder')
        z = _origin(col)[2]
        length, radius = float(cyl.get('length')), float(cyl.get('radius'))
        assert _origin(col)[:2] == [0.0, 0.0]
        spans[name] = (z - length / 2, z + length / 2, radius)
    lower, upper = spans['chassis_lower'], spans['chassis_upper']
    # Suelo en z = -base_z respecto a base_link. Despeje del CAD, no apoyo falso.
    assert lower[0] + base_z == pytest.approx(CLEARANCE, abs=1e-6)
    # Sin invadir las ruedas: radio inferior bajo la cara interna, superior sobre la llanta.
    assert lower[2] == pytest.approx(inner - GAP, abs=1e-9)
    assert lower[1] == pytest.approx(wheel_top + GAP, abs=1e-9)
    assert upper[0] == pytest.approx(wheel_top + GAP, abs=1e-9)
    assert upper[2] == pytest.approx(0.100, abs=1e-9)
    assert upper[1] == pytest.approx(0.13720, abs=1e-6)   # tapa de la envolvente de Víctor
    # Casters sin cambios: esferas en x=±0.085, r=0.012, tocando el suelo como las ruedas.
    spheres = [c for c in collisions if c.find('geometry/sphere') is not None]
    assert len(spheres) == 2 and all(not c.get('name') for c in spheres)
    for col, x in zip(spheres, (0.085, -0.085)):
        r = float(col.find('geometry/sphere').get('radius'))
        xyz = _origin(col)
        assert (xyz[0], xyz[1], r) == pytest.approx((x, 0.0, 0.012))
        assert xyz[2] - r + base_z == pytest.approx(0.0, abs=1e-9)
    for xyz, radius, _ in wheels.values():
        assert xyz[2] - radius + base_z == pytest.approx(0.0, abs=1e-9)


def _canonical(robot):
    for element in robot.iter():
        element.tail = None
        if element.text is not None and not element.text.strip():
            element.text = None
    return ET.tostring(robot, encoding='unicode')


def test_collision_modes():
    legacy_default = _canonical(_urdf(DESC_XACRO))
    assert _canonical(_urdf(DESC_XACRO, collision_mode='legacy')) == legacy_default
    # El xacro estacionario de C-006 sigue en legacy (caja que toca el suelo).
    sim = _urdf(SIM_XACRO)
    box = sim.find("link[@name='base_link']/collision/geometry/box")
    assert _floats(box.get('size')) == pytest.approx([0.2, 0.2, 0.16718])
    # El de locomoción usa mobile por defecto; legacy solo si se pide, para inspección estática:
    # en legacy la caja no tiene nombre y sdformat también le aplica la fricción nula de los casters.
    assert _urdf(MOTION_XACRO, collision_mode='legacy').find(
        "link[@name='base_link']/collision/geometry/box") is not None
    mobile = _urdf(DESC_XACRO, collision_mode='mobile')
    assert {c.get('name') for c in mobile.findall("link[@name='base_link']/collision")} == {
        'chassis_lower', 'chassis_upper', None}
    # Fuera de la colisión del chasis, mobile y legacy son idénticos (masas, TF, ruedas, sensores).
    assert _canonical(_strip_chassis(mobile)) == _canonical(_strip_chassis(_urdf(DESC_XACRO)))


def _strip_chassis(robot):
    link = robot.find("link[@name='base_link']")
    for col in link.findall('collision'):
        if col.find('geometry/sphere') is None:
            link.remove(col)
    return robot


def test_invalid_collision_mode_fails():
    result = subprocess.run(['xacro', DESC_XACRO, 'collision_mode:=bogus'], capture_output=True,
                            text=True, timeout=60)
    assert result.returncode != 0
    assert 'collision_mode' in result.stderr


def test_diffdrive_uses_wheel_geometry(motion, sdf):
    wheels = _wheels(motion)
    separation = wheels['left'][0][1] - wheels['right'][0][1]
    radius = wheels['left'][1]
    assert separation == pytest.approx(0.17410, abs=1e-9)
    assert radius == wheels['right'][1] == pytest.approx(0.030)
    assert _origin(motion.find("joint[@name='base_joint']"))[2] == pytest.approx(radius)
    joints = {j.get('name'): j.get('type') for j in motion.findall('joint')}
    sdf_joints = {j.get('name'): j.get('type') for j in sdf.find('model').findall('joint')}
    for tree, joint_types, kind in ((motion, joints, 'continuous'), (sdf, sdf_joints, 'revolute')):
        plugin = _diffdrive(tree)
        values = {e.tag: e.text for e in plugin}
        assert plugin.get('filename') == 'gz-sim-diff-drive-system'
        # Conjunto exacto de plugins: ningún otro system publica TF, odometría ni comandos.
        assert {p.get('name') for p in tree.iter('plugin')} == {
            'gz::sim::systems::JointStatePublisher', 'gz::sim::systems::DiffDrive'}
        assert float(values['wheel_separation']) == pytest.approx(separation, abs=1e-9)
        assert float(values['wheel_radius']) == pytest.approx(radius, abs=1e-9)
        assert float(values['wheel_separation']) != pytest.approx(0.22)
        assert joint_types[values['left_joint']] == kind
        assert joint_types[values['right_joint']] == kind
        assert (values['left_joint'], values['right_joint']) == ('wheel_left_joint', 'wheel_right_joint')
        assert values['topic'] == GZ_CMD
        assert values['odom_topic'] == '/model/colmibot/odometry'
        assert values['tf_topic'] == '/model/colmibot/tf'
        assert (values['frame_id'], values['child_frame_id']) == ('odom', 'base_footprint')
        for kind_ in ('linear_velocity', 'angular_velocity', 'linear_acceleration',
                      'angular_acceleration'):
            assert float(values[f'max_{kind_}']) > 0
            assert float(values[f'min_{kind_}']) == -float(values[f'max_{kind_}'])
    assert [m.get('name') for m in sdf.findall('model')] == ['colmibot']


def _friction(collision):
    mu = collision.find('surface/friction/ode/mu')
    mu2 = collision.find('surface/friction/ode/mu2')
    return (None if mu is None else float(mu.text), None if mu2 is None else float(mu2.text))


def test_sdf_friction_survives_conversion(sdf):
    seen = {'sphere': [], 'cylinder': []}
    for link in sdf.iter('link'):
        for col in link.findall('collision'):
            shape = col.find('geometry')[0].tag
            seen.setdefault(shape, []).append((link.get('name'), col.get('name'), _friction(col)))
    spheres = seen['sphere']
    assert len(spheres) == 2
    assert all(f == (0.0, 0.0) for _, _, f in spheres), spheres
    # Chasis y llantas conservan la fricción por defecto (sin override): un contacto falso del
    # chasis frenaría el robot en lugar de quedar oculto.
    cylinders = seen['cylinder']
    assert len(cylinders) == 4, cylinders
    assert all(f == (None, None) for _, _, f in cylinders), cylinders
    assert {name for name, _, _ in cylinders} == {'base_footprint', 'wheel_left_link',
                                                  'wheel_right_link'}


def _bridge(name):
    with open(os.path.join(SHARE, 'config', name)) as f:
        return yaml.safe_load(f)


def test_motion_bridge_directions(motion):
    entries = _bridge('bridge_motion.yaml')
    to_gz = [e for e in entries if e['direction'] == 'ROS_TO_GZ']
    assert to_gz == [{'ros_topic_name': SIM_CMD, 'gz_topic_name': GZ_CMD,
                      'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist',
                      'direction': 'ROS_TO_GZ'}]
    plugin = {e.tag: e.text for e in _diffdrive(motion)}
    from_gz = {e['ros_topic_name']: e for e in entries if e['direction'] == 'GZ_TO_ROS'}
    assert set(from_gz) == {'/odom', '/tf'}
    assert len(entries) == 3
    assert from_gz['/odom']['gz_topic_name'] == plugin['odom_topic']
    assert from_gz['/odom']['ros_type_name'] == 'nav_msgs/msg/Odometry'
    assert from_gz['/tf']['gz_topic_name'] == plugin['tf_topic']
    assert from_gz['/tf']['ros_type_name'] == 'tf2_msgs/msg/TFMessage'
    # Ningún puente del paquete expone /cmd_vel genérico ni duplica /clock o /joint_states.
    topics = []
    for name in ('bridge_stationary.yaml', 'bridge_motion.yaml', 'bridge_camera.yaml'):
        topics += [e['ros_topic_name'].lstrip('/') for e in _bridge(name)]
    assert 'cmd_vel' not in topics
    assert len(topics) == len(set(topics))


def _launch(**overrides):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    path = os.path.join(SHARE, 'launch', 'sim_motion.launch.py')
    spec = importlib.util.spec_from_file_location('sim_motion', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = _perform(context, entity.default_value)
    context.launch_configurations.update(overrides)
    return path, entities, context


def _perform(context, value):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _node_params(context, node):
    from launch_ros.parameter_descriptions import ParameterValue
    params = {}
    for group in node._Node__parameters or []:
        for key, value in group.items():
            if isinstance(value, ParameterValue):
                value = value.value
            params[_perform(context, key)] = (
                value if isinstance(value, bool) else _perform(context, value))
    return params


def test_motion_launch():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.actions import Node

    path, entities, context = _launch()
    assert all(isinstance(e, (Node, IncludeLaunchDescription, DeclareLaunchArgument))
               for e in entities)
    nodes = [e for e in entities if isinstance(e, Node)]
    assert sorted((n.node_package, n.node_executable) for n in nodes) == sorted([
        ('robot_state_publisher', 'robot_state_publisher'), ('ros_gz_sim', 'create'),
        ('ros_gz_bridge', 'parameter_bridge'), ('ros_gz_bridge', 'parameter_bridge'),
        ('ros_gz_bridge', 'parameter_bridge'), ('rviz2', 'rviz2')])
    for node in nodes:
        assert not node._Node__remappings
    rsp = next(n for n in nodes if n.node_executable == 'robot_state_publisher')
    params = _node_params(context, rsp)
    assert params['use_sim_time'] is True
    robot = ET.fromstring(params['robot_description'])
    _diffdrive(robot)
    assert {c.get('name') for c in robot.findall("link[@name='base_link']/collision")} >= {
        'chassis_lower', 'chassis_upper'}
    raw = {_perform(context, k): v for k, v in rsp._Node__parameters[0].items()}
    command = _perform(context, raw['robot_description'].value[0].command)
    assert command.startswith('xacro ' + MOTION_XACRO + ' collision_mode:=mobile '), command
    bridges = {_perform(context, n._Node__node_name): n
               for n in nodes if n.node_executable == 'parameter_bridge'}
    expected = {'colmibot_bridge': 'bridge_stationary.yaml',
                'colmibot_motion_bridge': 'bridge_motion.yaml',
                'colmibot_camera_bridge': 'bridge_camera.yaml'}
    assert set(bridges) == set(expected)
    for name, node in bridges.items():
        assert not node._Node__arguments
        assert _node_params(context, node) == {
            'config_file': os.path.join(SHARE, 'config', expected[name])}
    assert bridges['colmibot_bridge'].condition is None
    assert bridges['colmibot_motion_bridge'].condition is None
    assert not bridges['colmibot_camera_bridge'].condition.evaluate(context)
    rviz = next(n for n in nodes if n.node_executable == 'rviz2')
    assert [_perform(context, a) for a in rviz._Node__arguments] == [
        '-d', os.path.join(SHARE, 'rviz', 'sim_motion.rviz')]
    assert _node_params(context, rviz) == {'use_sim_time': True}
    create = next(n for n in nodes if n.node_executable == 'create')
    assert [_perform(context, a) for a in create._Node__arguments] == [
        '-topic', 'robot_description', '-name', 'colmibot', '-z', '0.0']
    # Vocabulario prohibido en el código del launch (sin contar el docstring).
    with open(path) as f:
        tree = ast.parse(f.read())
    tree.body = [n for n in tree.body if not (isinstance(n, ast.Expr)
                                                and isinstance(n.value, ast.Constant))]
    code = ast.unparse(tree).lower()
    for token in FORBIDDEN + ('setenvironmentvariable', "'/cmd_vel'"):
        assert token not in code, token


def test_motion_rviz_fixed_frame():
    with open(os.path.join(SHARE, 'rviz', 'sim_motion.rviz')) as f:
        config = yaml.safe_load(f)['Visualization Manager']
    assert config['Global Options']['Fixed Frame'] == 'odom'
    odom = [d for d in config['Displays'] if d['Class'] == 'rviz_default_plugins/Odometry']
    assert [d['Topic']['Value'] for d in odom] == ['/odom']


@pytest.fixture(scope='module')
def probe():
    loader = importlib.machinery.SourceFileLoader('motion_probe', PROBE)
    spec = importlib.util.spec_from_loader('motion_probe', loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_probe_is_finite_bounded_and_ends_at_zero(probe, motion):
    assert os.access(PROBE, os.X_OK)
    assert probe.CMD_TOPIC == SIM_CMD
    assert probe.BRIDGE_NODE == 'colmibot_motion_bridge'
    plugin = {e.tag: float(e.text) for e in _diffdrive(motion) if e.tag.startswith(('max_', 'wheel_'))}
    assert probe.WHEEL_RADIUS == pytest.approx(plugin['wheel_radius'])
    assert probe.WHEEL_SEPARATION == pytest.approx(plugin['wheel_separation'])
    for sequence in (probe.SEQUENCE, probe.LATCH_SEQUENCE):
        assert sequence[0][1:3] == (0.0, 0.0) and sequence[-1][1:3] == (0.0, 0.0)
        assert sequence[-1][3] >= 1.0
        assert sum(step[3] for step in sequence) <= 20.0
        for name, vx, wz, dur in sequence:
            if vx is None:
                continue
            assert abs(vx) <= 0.05 < plugin['max_linear_velocity']
            assert abs(wz) <= 0.30 < plugin['max_angular_velocity']
            if (vx, wz) != (0.0, 0.0):
                assert 0.0 < dur <= 2.0, name
    moves = [(vx, wz) for _, vx, wz, _ in probe.SEQUENCE if (vx, wz) != (0.0, 0.0)]
    assert moves == [(0.05, 0.0), (-0.05, 0.0), (0.0, 0.30)]
    assert probe.FINAL_ZEROS >= 5
    with open(PROBE) as f:
        source = f.read()
    assert source.count("'/cmd_vel'") == 0 and source.count('"/cmd_vel"') == 0


def _synthetic(probe, scale=1.0, gt_yaw_gain=1.0):
    """Robot cinemático ideal (escala 1) o atascado (escala 0) siguiendo SEQUENCE a 100 Hz.

    gt_yaw_gain escala solo el giro de la pose verdadera (la odometría sigue a las ruedas).
    """
    data = {'gt': [], 'odom': [], 'joints': [], 'cmd': []}
    phases, t, x, y, yaw, pl, pr = [], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
    gt_yaw = 0.0
    dt, half = 0.01, probe.WHEEL_SEPARATION / 2
    # Como en Gazebo, los flujos empiezan antes de la secuencia y siguen después (sin extrapolar).
    data['gt'].append((t, x, y, 0.0, gt_yaw, 0.0))
    data['odom'].append((t, x, y, yaw, 0.0, 0.0))
    data['joints'].append((t, pl, pr, 0.0, 0.0))
    for name, vx, wz, dur in probe.SEQUENCE + (('cola', 0.0, 0.0, 0.5),):
        phases.append({'name': name, 'vx': vx, 'wz': wz, 'duration_cmd': dur, 't0': t})
        for _ in range(int(round(dur / dt))):
            v, w = scale * vx, scale * wz
            vl, vr = (v - w * half) / probe.WHEEL_RADIUS, (v + w * half) / probe.WHEEL_RADIUS
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
            gt_yaw += gt_yaw_gain * w * dt
            pl += vl * dt
            pr += vr * dt
            t += dt
            data['gt'].append((t, x, y, 0.0, gt_yaw, 0.0))
            data['odom'].append((t, x, y, yaw, v, w))
            data['joints'].append((t, pl, pr, vl, vr))
        phases[-1]['t1'] = t
    return data, phases[:-1]


def test_probe_evaluation_detects_motion_and_frozen_robot(probe):
    checks, metrics = probe.evaluate(*_synthetic(probe))
    failed = [c for c in checks if not c['pass']]
    assert not failed, failed
    assert len(metrics['phases']) == len(probe.SEQUENCE)
    frozen, _ = probe.evaluate(*_synthetic(probe, scale=0.0))
    failed = {c['check'] for c in frozen if not c['pass']}
    for phase in ('avance', 'retroceso', 'giro'):
        assert any(name.startswith(f'{phase}: gt') for name in failed), phase


def test_probe_odometry_tolerance_is_bounded(probe):
    # C-007: en Gazebo el giro real fue 1.074 veces el comandado y la odometría siguió a las ruedas.
    checks, _ = probe.evaluate(*_synthetic(probe, gt_yaw_gain=1.074))
    assert all(c['pass'] for c in checks), [c for c in checks if not c['pass']]
    checks, _ = probe.evaluate(*_synthetic(probe, gt_yaw_gain=1.25))
    failed = {c['check'] for c in checks if not c['pass']}
    assert 'giro: |odom-gt| yaw al final (rad)' in failed
    assert any(name.startswith('giro: gt Δyaw') for name in failed)


# ---------------- C-007-P2: cobertura de la pose verdadera (sin extrapolar) ----------------

def _status(checks, name):
    found = [c for c in checks if c['check'] == name]
    assert len(found) == 1, (name, [c['check'] for c in checks])
    return found[0]['status']


def _drop(data, stream, t0, t1):
    data[stream] = [r for r in data[stream] if not (t0 < r[0] < t1)]


def _phase(phases, name):
    return next(p for p in phases if p['name'] == name)


def test_interp_never_extrapolates_nor_bridges_gaps(probe):
    rows = [{'t': 1.0, 'x': 0.0}, {'t': 2.0, 'x': 1.0}, {'t': 2.05, 'x': 1.05}]
    assert probe.interp(rows, 1.5, ('x',)) == [0.5]
    assert probe.interp(rows, 0.9, ('x',)) is None           # antes de la primera muestra
    assert probe.interp(rows, 2.1, ('x',)) is None           # después de la última (caso Codex)
    assert probe.interp(rows, 1.5, ('x',), max_gap=0.1) is None   # hueco de 1 s
    assert probe.interp(rows, 2.02, ('x',), max_gap=0.1) == pytest.approx([1.02])
    assert probe.interp([], 1.0, ('x',)) is None
    ok, reason, _, _ = probe.coverage([1.0, 2.0], 1.0, 2.0, 0.1)
    assert not ok and 'hueco' in reason
    assert probe.coverage([1.0, 1.05, 1.1], 1.0, 1.1, 0.1)[0]


def test_truncated_gt_before_final_rest_is_not_pass(probe):
    # Reproducción de Codex: la pose verdadera termina antes de la ventana final de reposo.
    data, phases = _synthetic(probe)
    last = _phase(phases, 'reposo_giro')
    cut = last['t1'] - 1.0 - 0.965          # equivale a 54.647 s frente a una ventana desde 55.612 s
    data['gt'] = [r for r in data['gt'] if r[0] <= cut]
    checks, _ = probe.evaluate(data, phases)
    assert not all(c['pass'] for c in checks)
    for name in ('reposo_giro: traslación en último 1.0 s (m)', 'reposo_giro: giro en último 1.0 s (rad)',
                 'reposo_giro: en último 1.0 s: cobertura gt, muestras',
                 'gt: cobertura de la secuencia, hueco máx (s)', 'inclinación máxima (rad)',
                 'giro: gt Δyaw (rad) vs +0.6000', 'reposo_giro: |odom-gt| yaw al final (rad)'):
        assert _status(checks, name) == 'INSUFFICIENT_EVIDENCE', name
    # Lo que sí está cubierto se sigue midiendo: no hay falsos negativos fuera del hueco.
    assert _status(checks, 'avance: gt Δx cuerpo (m) vs +0.1000') == 'PASS'
    assert _status(checks, 'reposo_avance: traslación en último 1.0 s (m)') == 'PASS'


def test_continuous_gt_passes_and_reports_coverage(probe):
    checks, _ = probe.evaluate(*_synthetic(probe))
    assert all(c['status'] == 'PASS' for c in checks), [c for c in checks if c['status'] != 'PASS']
    names = {c['check'] for c in checks}
    for rest in ('reposo_inicial', 'reposo_avance', 'reposo_retroceso', 'reposo_giro'):
        win = '0.5' if rest == 'reposo_inicial' else '1.0'
        assert f'{rest}: en último {win} s: cobertura gt, muestras' in names
    for stream in ('gt', 'odom', 'joints'):
        assert f'{stream}: orden temporal (retrocesos de t)' in names


def test_internal_gaps_and_missing_windows_are_detected(probe):
    data, phases = _synthetic(probe)
    rest = _phase(phases, 'reposo_avance')
    _drop(data, 'gt', rest['t1'] - 0.8, rest['t1'] - 0.4)          # hueco interno de 0.4 s
    adv = _phase(phases, 'retroceso')
    _drop(data, 'odom', adv['t0'] + 0.5, adv['t0'] + 0.9)          # hueco de /odom en una maniobra
    gir = _phase(phases, 'reposo_giro')
    _drop(data, 'joints', gir['t1'] - 0.7, gir['t1'] - 0.3)        # juntas ausentes en un reposo
    checks, _ = probe.evaluate(data, phases)
    assert not all(c['pass'] for c in checks)
    assert _status(checks, 'reposo_avance: traslación en último 1.0 s (m)') == 'INSUFFICIENT_EVIDENCE'
    assert _status(checks, 'reposo_avance: en último 1.0 s: cobertura gt, muestras') == 'INSUFFICIENT_EVIDENCE'
    assert _status(checks, 'retroceso: odom Δx cuerpo (m) vs -0.1000') == 'INSUFFICIENT_EVIDENCE'
    assert _status(checks, 'reposo_giro: |vel. rueda| máx en último 1.0 s (rad/s)') == 'INSUFFICIENT_EVIDENCE'
    # Los demás reposos siguen midiéndose con datos reales.
    assert _status(checks, 'reposo_retroceso: traslación en último 1.0 s (m)') == 'PASS'
    reason = next(c for c in checks if c['check'] == 'reposo_avance: traslación en último 1.0 s (m)')['reason']
    assert 'hueco' in reason


def test_out_of_order_samples_fail(probe):
    data, phases = _synthetic(probe)
    data['gt'][10], data['gt'][11] = data['gt'][11], data['gt'][10]
    checks, _ = probe.evaluate(data, phases)
    assert _status(checks, 'gt: orden temporal (retrocesos de t)') == 'FAIL'


# ---------------- C-007-P2: exclusividad del escritor ----------------

def _endpoint(gid_byte, name='colmibot_motion_probe', ns='/'):
    from types import SimpleNamespace
    return SimpleNamespace(endpoint_gid=[gid_byte] * 16, node_name=name, node_namespace=ns)


def test_audit_counts_homonymous_endpoints_by_gid(probe):
    a, b = _endpoint(1), _endpoint(2)          # mismo nombre de nodo, distinto GID (caso Codex)
    own, others = probe.audit_publishers([a, b])
    assert not own and len(others) == 2        # antes de crear el propio: ambos son ajenos
    own, others = probe.audit_publishers([a, b], probe.gid_of(a))
    assert own and len(others) == 1 and others[0].startswith('/colmibot_motion_probe gid=0202')
    own, others = probe.audit_publishers([a], probe.gid_of(a))
    assert own and others == []


def test_rejected_writer_sends_nothing_not_even_zeros(probe):
    sent = []
    writer = probe.CommandWriter(lambda *m: sent.append(m))
    with pytest.raises(RuntimeError):
        writer.send('avance', 0.05, 0.0)
    assert writer.close_with_zeros(probe.FINAL_ZEROS) == 0 and sent == []
    writer.activate()
    writer.send('avance', 0.05, 0.0)
    assert writer.close_with_zeros(probe.FINAL_ZEROS) == probe.FINAL_ZEROS
    assert sent[1:] == [('cero_final', 0.0, 0.0)] * probe.FINAL_ZEROS


def test_probe_order_lock_before_ros_and_publisher_before_activation():
    with open(PROBE) as f:
        src = f.read()
    main = src[src.index('def main('):]
    order = [main.index(k) for k in ('acquire_writer_lock(', 'import rclpy', 'create_publisher(',
                                     'writer.activate()', 'writer.close_with_zeros(')]
    assert order == sorted(order)
    assert src.count('pub.publish(') == 1                 # solo dentro de raw_publish
    assert "if p.node_name != 'colmibot_motion_probe'" not in src   # filtro por nombre eliminado
    # Un publicador ajeno visto antes de escribir rechaza de inmediato (no espera a que se libere).
    assert 'otro publicador en {CMD_TOPIC} antes de escribir' in main


_LOAD = ('import importlib.machinery, importlib.util, sys, time; '
         'l = importlib.machinery.SourceFileLoader("p", sys.argv[1]); '
         's = importlib.util.spec_from_loader("p", l); m = importlib.util.module_from_spec(s); '
         'l.exec_module(m); dom, err = m.canonical_domain_id(sys.argv[2]); '
         'fd, path, err = m.acquire_writer_lock(dom, m.CMD_TOPIC); print(path, flush=True); ')


def _hold_lock(env, domain_text):
    """Proceso aparte que toma el lock con `domain_text` y lo retiene; devuelve (proceso, ruta)."""
    holder = subprocess.Popen(
        [sys.executable, '-c', _LOAD + 'time.sleep(60) if fd is not None else sys.exit(3)', PROBE,
         domain_text], env=env, stdout=subprocess.PIPE, text=True)
    return holder, holder.stdout.readline().strip()


def _run_probe(env, out, domain_text):
    return subprocess.run([PROBE, '--out', str(out)], env=dict(env, ROS_DOMAIN_ID=domain_text),
                          capture_output=True, text=True, timeout=60)


def test_lock_rejects_second_probe_without_publishing(probe, tmp_path):
    env = dict(os.environ, TMPDIR=str(tmp_path))
    first, path = _hold_lock(env, '199')
    try:
        assert path.startswith(str(tmp_path)) and 'domain199_' in path
        second = subprocess.run([sys.executable, '-c', _LOAD + 'sys.exit(0 if fd is not None else 3)',
                                 PROBE, '199'], env=env, capture_output=True, text=True, timeout=30)
        assert second.returncode == 3              # el lock está tomado por otro proceso
        out = tmp_path / 'segunda_sonda'
        probe_run = _run_probe(env, out, '199')
        assert probe_run.returncode == 2
        assert 'exclusividad no adquirida' in probe_run.stderr
        assert 'No se publicó nada' in probe_run.stderr
        assert not out.exists()                    # ni datos ni ceros: no tocó el canal
    finally:
        first.kill()
        first.wait()
    again = subprocess.run([sys.executable, '-c', _LOAD + 'sys.exit(0 if fd is not None else 3)',
                            PROBE, '199'], env=env, capture_output=True, text=True, timeout=30)
    assert again.returncode == 0                   # liberado al terminar el proceso dueño


# ---------------- C-007-P2B ----------------

@pytest.mark.parametrize('text, expected', [
    ('87', 87), ('+87', 87), (' 87', 87), ('87 ', 87), ('\t87', 87), ('', 0), (None, 0), ('0', 0),
    ('+0', 0), ('232', 232),
    ('087', None), ('010', None), ('0x57', None), ('8.7', None), ('-1', None), ('abc', None),
    ('87abc', None), ('+087', None), ('233', None), ('٨٧', None),
    # C-007-P2C (evidence/C-007/P2C/01): Jazzy salta solo espacios ASCII y rechaza '' de solo espacios
    ('\n87', 87), ('\r87', 87), ('\v87', 87), ('\f87', 87), ('87\t', 87), (' +87 ', 87),
    (' ', None), ('\t', None), (' \t ', None), ('\n', None), ('+ 87', None), ('\u00a087', None),
    ('87\u00a0', None), ('\u00a0', None), ('\u200387', None), ('\u300087', None), ('\u202f87', None),
    ('\x1c87', None), ('\x8587', None),
    (' 0', 0), ('\t+0', 0), ('0 ', None), ('0\t', None), (' +0 ', None), ('232 ', 232),
])
def test_canonical_domain_id(probe, text, expected):
    # Comprobado en Jazzy (evidence/C-007/P2B/01): '87', '+87', ' 87', '\t87' -> 87; '010' -> 8 (octal);
    # '087' inválido. Las formas ambiguas se rechazan en lugar de adivinar.
    value, error = probe.canonical_domain_id(text)
    assert value == expected
    assert (error is None) == (expected is not None)
    if expected is not None:
        assert probe.lock_path(value, probe.CMD_TOPIC).endswith(f'domain{expected}_colmibot_sim_cmd_vel.lock')
    with pytest.raises(TypeError):
        probe.lock_path('87', probe.CMD_TOPIC)    # nunca la clave textual


@pytest.mark.parametrize('spelling', ['+87', ' 87', '\t87'])
def test_lock_is_shared_by_equivalent_domain_spellings(probe, tmp_path, spelling):
    env = dict(os.environ, TMPDIR=str(tmp_path))
    holder, path = _hold_lock(env, '87')
    try:
        assert path.endswith('domain87_colmibot_sim_cmd_vel.lock')
        out = tmp_path / 'segunda'
        run = _run_probe(env, out, spelling)
        assert run.returncode == 2, run.stderr
        assert 'exclusividad no adquirida' in run.stderr and 'No se publicó nada' in run.stderr
        assert not out.exists()
        assert sorted(p.name for p in tmp_path.iterdir() if p.suffix == '.lock') == [
            'colmibot_motion_probe.domain87_colmibot_sim_cmd_vel.lock']
    finally:
        holder.kill()
        holder.wait()


@pytest.mark.parametrize('text, message', [
    ('087', 'ceros a la izquierda'), ('010', 'ceros a la izquierda'), ('abc', 'no es un entero decimal')])
def test_invalid_domain_fails_closed_before_ros(probe, tmp_path, text, message):
    out = tmp_path / 'sonda'
    run = _run_probe(dict(os.environ, TMPDIR=str(tmp_path)), out, text)
    assert run.returncode == 2
    assert message in run.stderr and 'No se creó lock ni publicador' in run.stderr
    assert not out.exists()
    assert not [p for p in tmp_path.iterdir() if p.suffix == '.lock']   # ni siquiera el archivo de lock


# C-007-P2C: el ejecutable real dentro de un proceso que informa si llegó a importar rclpy.
_RUN_AND_REPORT_RCLPY = (
    'import runpy, sys\n'
    'sys.argv = [sys.argv[1], "--out", sys.argv[2]]\n'
    'try:\n'
    '    runpy.run_path(sys.argv[0], run_name="__main__")\n'
    '    code = 0\n'
    'except SystemExit as e:\n'
    '    code = e.code if isinstance(e.code, int) else 1\n'
    'print("RCLPY_IMPORTADO" if "rclpy" in sys.modules else "RCLPY_NO_IMPORTADO", file=sys.stderr)\n'
    'sys.exit(code)\n')


@pytest.mark.parametrize('text, message', [
    (' ', 'solo espacios'), ('\t', 'solo espacios'), (' \t ', 'solo espacios'),
    ('\u00a087', 'no ASCII'), ('87\u00a0', 'no ASCII'), ('\u200387', 'no ASCII'),
    ('\x1c87', 'no es un entero decimal'), ('0 ', 'espacios finales')])
def test_whitespace_or_unicode_domain_fails_closed_before_ros(probe, tmp_path, text, message):
    # Antes de P2C, str.strip() convertía ' ' en 0 y '\u00a087' en 87: creaba el lock y rclpy.init()
    # fallaba con RCLError (salida 1). Ahora: 2, sin lock, sin --out y sin importar rclpy.
    tmp = tmp_path / 'tmpdir'
    tmp.mkdir()
    out = tmp_path / 'sonda'
    run = subprocess.run([sys.executable, '-c', _RUN_AND_REPORT_RCLPY, PROBE, str(out)],
                         env=dict(os.environ, TMPDIR=str(tmp), ROS_DOMAIN_ID=text),
                         capture_output=True, text=True, timeout=60)
    assert run.returncode == 2, run.stderr
    assert message in run.stderr and 'No se creó lock ni publicador' in run.stderr
    assert 'RCLPY_NO_IMPORTADO' in run.stderr and 'RCLError' not in run.stderr
    assert not out.exists()
    assert list(tmp.iterdir()) == []                # ni lock ni ningún otro archivo


@pytest.mark.parametrize('kind', ['nan', 'none', 'sin_campos'])
def test_rest_requires_valid_velocities_of_both_wheels(probe, kind):
    # Caso Codex: posiciones hasta el final de la ventana, velocidades válidas solo hasta 0.2 s antes.
    data, phases = _synthetic(probe)
    last = _phase(phases, 'reposo_giro')
    cut = last['t1'] - 0.200
    broken = {'nan': lambda r: r[:3] + [math.nan, math.nan], 'none': lambda r: r[:3] + [None, None],
              'sin_campos': lambda r: r[:3]}[kind]
    data['joints'] = [r if r[0] <= cut else broken(list(r)) for r in data['joints']]
    assert data['joints'][-1][0] >= last['t1']      # el flujo de posiciones sigue cubriendo la ventana
    checks, _ = probe.evaluate(data, phases)
    name = 'reposo_giro: |vel. rueda| máx en último 1.0 s (rad/s)'
    assert _status(checks, name) == 'INSUFFICIENT_EVIDENCE'
    assert not all(c['pass'] for c in checks)
    reason = next(c for c in checks if c['check'] == name)['reason']
    assert 'velocidades de rueda válidas' in reason and 'última muestra' in reason
    assert _status(checks, 'reposo_retroceso: |vel. rueda| máx en último 1.0 s (rad/s)') == 'PASS'


def test_one_wheel_velocity_missing_is_not_enough(probe):
    data, phases = _synthetic(probe)
    rest = _phase(phases, 'reposo_avance')
    data['joints'] = [list(r[:4]) + [math.nan] if rest['t1'] - 0.6 < r[0] < rest['t1'] - 0.3 else r
                      for r in data['joints']]     # hueco interno de 0.3 s en la rueda derecha
    checks, _ = probe.evaluate(data, phases)
    assert _status(checks, 'reposo_avance: |vel. rueda| máx en último 1.0 s (rad/s)') == 'INSUFFICIENT_EVIDENCE'
