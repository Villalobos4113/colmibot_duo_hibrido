"""Contrato estático de colmibot_gazebo (C-006).

Sin lanzar Gazebo comprueba que el bringup es estacionario: sin DiffDrive, cmd_vel, joystick,
puentes ROS->GZ ni TF desde Gazebo; /joint_states viene solo de Gazebo y la cámara (opcional)
sale en camera_optical_frame mirando hacia +X. Las pruebas del launch resuelven sus
sustituciones reales (sin ejecutar nodos), incluido el xacro que recibe robot_state_publisher.
"""
import ast
import importlib.util
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

SHARE = get_package_share_directory('colmibot_gazebo')
SIM_XACRO = os.path.join(SHARE, 'urdf', 'colmibot_sim.urdf.xacro')
FORBIDDEN = ('diffdrive', 'diff-drive', 'diff_drive', 'cmd_vel', 'odom', 'joy', 'teleop',
             'pose-publisher', 'posepublisher', '/dev/')


def _urdf(use_camera):
    return xacro.process_file(SIM_XACRO, mappings={'use_camera': use_camera}).toxml()


def _sdf(urdf_text):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(urdf_text)
    try:
        result = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True,
                                timeout=60)
    finally:
        os.unlink(f.name)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


@pytest.fixture(scope='module', params=['false', 'true'], ids=['base', 'camera'])
def variant(request):
    urdf_text = _urdf(request.param)
    return request.param == 'true', urdf_text, ET.fromstring(urdf_text), _sdf(urdf_text)


def test_no_motion_or_teleop_elements(variant):
    # ElementTree descarta comentarios: se revisan elementos y atributos, no la documentación.
    _, _, urdf, sdf = variant
    for tree in (urdf, sdf):
        text = ET.tostring(tree, encoding='unicode').lower()
        for token in FORBIDDEN:
            assert token not in text, token


def test_plugins(variant):
    camera, _, urdf, sdf = variant
    expected = {'gz::sim::systems::JointStatePublisher'}
    if camera:
        expected.add('gz::sim::systems::Sensors')
    assert {p.get('name') for p in urdf.iter('plugin')} == expected
    assert {p.get('name') for p in sdf.iter('plugin')} == expected
    jsp = next(p for p in urdf.iter('plugin') if p.get('name').endswith('JointStatePublisher'))
    assert [j.text for j in jsp.findall('joint_name')] == ['wheel_left_joint', 'wheel_right_joint']
    assert jsp.find('topic').text == 'joint_states'


def test_sdf_model(variant):
    camera, _, _, sdf = variant
    models = sdf.findall('model')
    assert [m.get('name') for m in models] == ['colmibot']
    joints = {j.get('name'): j.get('type') for j in models[0].findall('joint')}
    assert joints == {'wheel_left_joint': 'revolute', 'wheel_right_joint': 'revolute'}
    sensors = list(sdf.iter('sensor'))
    if not camera:
        assert not sensors
        return
    assert len(sensors) == 1
    sensor = sensors[0]
    assert sensor.get('type') == 'camera'
    assert sensor.find('gz_frame_id').text == 'camera_optical_frame'
    assert sensor.find('topic').text == 'camera/image_raw'
    # La cámara conserva la pose de camera_link (Víctor): +X adelante, 6.1° hacia abajo.
    xyz_rpy = [float(v) for v in sensor.find('pose').text.split()]
    assert xyz_rpy[0] > 0.0
    assert xyz_rpy[3:] == pytest.approx([0.0, 0.1064, 0.0], abs=1e-6)


def _bridge(name):
    with open(os.path.join(SHARE, 'config', name)) as f:
        return yaml.safe_load(f)


def test_bridges_are_gz_to_ros_only():
    stationary = _bridge('bridge_stationary.yaml')
    camera = _bridge('bridge_camera.yaml')
    for entry in stationary + camera:
        assert entry['direction'] == 'GZ_TO_ROS'
        assert entry['ros_topic_name'] == entry['gz_topic_name']
    assert {e['ros_topic_name'] for e in stationary} == {'clock', 'joint_states'}
    assert {e['ros_topic_name'] for e in camera} == {'camera/image_raw', 'camera/camera_info'}


def _launch(**overrides):
    """Carga el launch instalado y resuelve sus argumentos (por defecto + overrides)."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    path = os.path.join(SHARE, 'launch', 'sim_stationary.launch.py')
    spec = importlib.util.spec_from_file_location('sim_stationary', path)
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


def _nodes(entities):
    from launch_ros.actions import Node
    return [e for e in entities if isinstance(e, Node)]


def test_launch_actions():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.actions import Node

    path, entities, context = _launch()
    allowed = (Node, IncludeLaunchDescription, DeclareLaunchArgument)
    assert all(isinstance(e, allowed) for e in entities)
    assert sorted((n.node_package, n.node_executable) for n in _nodes(entities)) == sorted([
        ('robot_state_publisher', 'robot_state_publisher'),
        ('ros_gz_sim', 'create'),
        ('ros_gz_bridge', 'parameter_bridge'),
        ('ros_gz_bridge', 'parameter_bridge'),
        ('rviz2', 'rviz2'),
    ])
    for node in _nodes(entities):
        assert not node._Node__remappings, node.node_executable
    # Vocabulario prohibido en el código del launch (sin contar el docstring).
    with open(path) as f:
        tree = ast.parse(f.read())
    tree.body = [n for n in tree.body if not (isinstance(n, ast.Expr)
                                                and isinstance(n.value, ast.Constant))]
    code = ast.unparse(tree).lower()
    for token in FORBIDDEN + ('setenvironmentvariable', 'gz_sim_resource_path',
                              'joint_state_publisher', '/tf'):
        assert token not in code, token


@pytest.mark.parametrize('gui, expected', [
    ('true', '-r -v 3 empty.sdf'), ('True', '-r -v 3 empty.sdf'), ('1', '-r -v 3 empty.sdf'),
    ('false', '-r -v 3 -s empty.sdf'),
])
def test_gazebo_include(gui, expected):
    from launch.actions import IncludeLaunchDescription

    _, entities, context = _launch(gui=gui)
    includes = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    assert len(includes) == 1
    location = _perform(context, includes[0].launch_description_source._LaunchDescriptionSource__location)
    assert location == os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                    'gz_sim.launch.py')
    args = {_perform(context, k): _perform(context, v) for k, v in includes[0].launch_arguments}
    assert args == {'gz_args': expected, 'on_exit_shutdown': 'true'}


@pytest.mark.parametrize('use_meshes', ['true', 'false'])
def test_robot_state_publisher_uses_sim_model(use_meshes):
    _, entities, context = _launch(use_meshes=use_meshes)
    rsp = next(n for n in _nodes(entities) if n.node_executable == 'robot_state_publisher')
    params = _node_params(context, rsp)
    assert params['use_sim_time'] is True
    urdf = ET.fromstring(params['robot_description'])
    assert urdf.get('name') == 'colmibot'
    assert {p.get('name') for p in urdf.iter('plugin')} == {'gz::sim::systems::JointStatePublisher'}
    assert bool(list(urdf.iter('mesh'))) == (use_meshes == 'true')


def test_spawn_bridges_and_rviz():
    _, entities, context = _launch()
    nodes = _nodes(entities)
    create = next(n for n in nodes if n.node_executable == 'create')
    assert [_perform(context, a) for a in create._Node__arguments] == [
        '-topic', 'robot_description', '-name', 'colmibot', '-z', '0.0']
    bridges = {_perform(context, n._Node__node_name): n
               for n in nodes if n.node_executable == 'parameter_bridge'}
    assert set(bridges) == {'colmibot_bridge', 'colmibot_camera_bridge'}
    expected_yaml = {'colmibot_bridge': 'bridge_stationary.yaml',
                     'colmibot_camera_bridge': 'bridge_camera.yaml'}
    for name, node in bridges.items():
        assert not node._Node__arguments, name
        config = _node_params(context, node)
        assert config == {'config_file': os.path.join(SHARE, 'config', expected_yaml[name])}
    assert bridges['colmibot_bridge'].condition is None
    assert not bridges['colmibot_camera_bridge'].condition.evaluate(context)
    _, _, camera_context = _launch(use_camera='true')
    assert bridges['colmibot_camera_bridge'].condition.evaluate(camera_context)
    rviz = next(n for n in nodes if n.node_executable == 'rviz2')
    assert _node_params(context, rviz) == {'use_sim_time': True}
    assert [_perform(context, a) for a in rviz._Node__arguments] == [
        '-d', os.path.join(SHARE, 'rviz', 'sim.rviz')]


def test_gazebo_resource_path_is_exported():
    manifest = os.path.join(get_package_share_directory('colmibot_description'), 'package.xml')
    exports = ET.parse(manifest).getroot().find('export')
    paths = [e.get('gazebo_model_path') for e in exports.findall('gazebo_ros')]
    assert paths == ['${prefix}/../']


def test_rviz_fixed_frame():
    with open(os.path.join(SHARE, 'rviz', 'sim.rviz')) as f:
        config = yaml.safe_load(f)
    assert config['Visualization Manager']['Global Options']['Fixed Frame'] == 'base_footprint'
