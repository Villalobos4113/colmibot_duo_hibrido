"""Pruebas estáticas de colmibot_description (C-006).

Comprueban que el port del modelo de Víctor (VicManJ-cloud/colmibot_ws@985b6b8) conserva
árbol, geometría, masas e inercias, y que las mallas instaladas son las del upstream.
No lanzan simulación ni hardware.
"""
import hashlib
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

SHARE = get_package_share_directory('colmibot_description')
MODEL = os.path.join(SHARE, 'urdf', 'colmibot.urdf.xacro')
MESH_DIR = os.path.join(SHARE, 'meshes')

EXPECTED_LINKS = {
    'base_footprint', 'base_link', 'wheel_left_link', 'wheel_right_link',
    'camera_link', 'camera_optical_frame', 'imu_link',
    'tof_left_link', 'tof_center_link', 'tof_right_link',
    'vis_camara', 'vis_electronica', 'vis_impreso_3d', 'vis_motores',
    'vis_placas_laser', 'vis_ruedas_locas', 'vis_esp32', 'vis_xl4015',
}
EXPECTED_MESHES = {
    'camara.stl', 'electronica.stl', 'impreso_3d.stl', 'motores.stl',
    'placas_laser.stl', 'ruedas_locas.stl', 'wheel_left.stl', 'wheel_right.stl',
}
# Valores de urdf/generated/*.xacro del upstream; provisionales (materiales CAD sin pesar).
WHEEL_Y = 0.08705
WHEEL_RADIUS = 0.030
BASE_MASS = 2.25741
WHEEL_MASS = 0.0193581
CAMERA_XYZ = (0.05289, 0.00005, 0.12140)
CAMERA_PITCH = 0.1064
# (origen xyz del inertial, [ixx, ixy, ixz, iyy, iyz, izz]) de generated/colmibot_inertial.xacro.
INERTIALS = {
    'base_link': ((-0.00672385, -0.00022214, 0.0213195),
                  [0.00675856, 2.13689e-05, -0.000966576, 0.00705147, 3.66283e-06, 0.00777712]),
    'wheel_left_link': ((-5.29078e-08, 0.00169416, 3.0834e-06),
                        [4.67554e-06, -1.8664e-12, 2.57785e-12, 9.11427e-06, 1.08796e-10,
                         4.67539e-06]),
    'wheel_right_link': ((6.88206e-15, -0.00169416, 3.0838e-06),
                         [4.67554e-06, 4.54209e-16, 2.11737e-18, 9.11427e-06, -1.08812e-10,
                          4.67539e-06]),
}
# Colisiones de base_link: caja del chasis (llega al suelo, provisional) y dos esferas de caster.
CHASSIS_BOX = ((0.0, 0.0, 0.05361), (0.20000, 0.20000, 0.16718))
CASTERS = [((0.085, 0.0, -0.018), 0.012), ((-0.085, 0.0, -0.018), 0.012)]
WHEEL_WIDTH = 0.01200


def _robot(use_meshes='true'):
    doc = xacro.process_file(MODEL, mappings={'use_meshes': use_meshes})
    return ET.fromstring(doc.toxml())


def _floats(text):
    return [float(v) for v in text.split()]


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _joint(robot, name):
    joint = robot.find(f"joint[@name='{name}']")
    assert joint is not None, name
    return joint


def _origin(element):
    origin = element.find('origin')
    xyz = _floats(origin.get('xyz', '0 0 0')) if origin is not None else [0.0] * 3
    rpy = _floats(origin.get('rpy', '0 0 0')) if origin is not None else [0.0] * 3
    return xyz, rpy


@pytest.fixture(scope='module')
def robot():
    return _robot()


def test_tree(robot):
    assert robot.get('name') == 'colmibot'
    links = {link.get('name') for link in robot.findall('link')}
    assert links == EXPECTED_LINKS
    joints = {j.get('name'): j.get('type') for j in robot.findall('joint')}
    assert len(joints) == len(EXPECTED_LINKS) - 1
    moving = {name for name, kind in joints.items() if kind != 'fixed'}
    assert moving == {'wheel_left_joint', 'wheel_right_joint'}
    children = {j.find('child').get('link') for j in robot.findall('joint')}
    assert EXPECTED_LINKS - children == {'base_footprint'}


def test_wheels_and_track(robot):
    xyz, rpy = _origin(_joint(robot, 'base_joint'))
    assert xyz == pytest.approx([0.0, 0.0, WHEEL_RADIUS])
    ys = {}
    for side, sign in (('left', 1.0), ('right', -1.0)):
        joint = _joint(robot, f'wheel_{side}_joint')
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert _floats(joint.find('axis').get('xyz')) == [0.0, 1.0, 0.0]
        xyz, rpy = _origin(joint)
        assert xyz == pytest.approx([0.0, sign * WHEEL_Y, 0.0], abs=1e-9)
        assert rpy == [0.0, 0.0, 0.0]
        ys[side] = xyz[1]
    assert ys['left'] - ys['right'] == pytest.approx(0.1741, abs=1e-9)
    for side in ('left', 'right'):
        cyl = robot.find(f"link[@name='wheel_{side}_link']/collision/geometry/cylinder")
        assert float(cyl.get('radius')) == pytest.approx(WHEEL_RADIUS)


def _inertia_matrix(inertial):
    i = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
    return [
        [i['ixx'], i['ixy'], i['ixz']],
        [i['ixy'], i['iyy'], i['iyz']],
        [i['ixz'], i['iyz'], i['izz']],
    ]


def _det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def test_masses_and_inertia(robot):
    masses = {}
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        name = link.get('name')
        masses[name] = float(inertial.find('mass').get('value'))
        m = _inertia_matrix(inertial)
        # Definida positiva (criterio de Sylvester) y desigualdad triangular.
        assert m[0][0] > 0
        assert m[0][0] * m[1][1] - m[0][1] ** 2 > 0
        assert _det3(m) > 0
        ixx, iyy, izz = m[0][0], m[1][1], m[2][2]
        assert ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy, name
    assert masses == pytest.approx(
        {'base_link': BASE_MASS, 'wheel_left_link': WHEEL_MASS, 'wheel_right_link': WHEEL_MASS})
    for name, (com, tensor) in INERTIALS.items():
        inertial = robot.find(f"link[@name='{name}']/inertial")
        xyz, rpy = _origin(inertial)
        assert xyz == pytest.approx(list(com), rel=1e-9, abs=1e-20), name
        assert rpy == [0.0, 0.0, 0.0]
        values = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        keys = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
        assert [values[k] for k in keys] == pytest.approx(tensor, rel=1e-9, abs=1e-25), name


def test_collision_geometry(robot):
    collisions = robot.findall("link[@name='base_link']/collision")
    assert len(collisions) == 3
    box = collisions[0]
    assert _origin(box)[0] == pytest.approx(list(CHASSIS_BOX[0]))
    assert _floats(box.find('geometry/box').get('size')) == pytest.approx(list(CHASSIS_BOX[1]))
    for collision, (xyz, radius) in zip(collisions[1:], CASTERS):
        assert _origin(collision)[0] == pytest.approx(list(xyz))
        assert float(collision.find('geometry/sphere').get('radius')) == pytest.approx(radius)
    for side in ('left', 'right'):
        collision = robot.find(f"link[@name='wheel_{side}_link']/collision")
        assert len(robot.findall(f"link[@name='wheel_{side}_link']/collision")) == 1
        assert _origin(collision)[1] == pytest.approx([math.pi / 2, 0.0, 0.0])
        cylinder = collision.find('geometry/cylinder')
        assert float(cylinder.get('length')) == pytest.approx(WHEEL_WIDTH)


def test_collision_has_inertial(robot):
    for link in robot.findall('link'):
        has_collision = link.find('collision') is not None
        has_inertial = link.find('inertial') is not None
        assert has_collision == has_inertial, link.get('name')


def test_camera_faces_plus_x(robot):
    xyz, rpy = _origin(_joint(robot, 'camera_link_joint'))
    assert xyz == pytest.approx(list(CAMERA_XYZ), abs=1e-9)
    assert rpy == pytest.approx([0.0, CAMERA_PITCH, 0.0])
    xyz_optical, rpy_optical = _origin(_joint(robot, 'camera_optical_frame_joint'))
    assert xyz_optical == [0.0, 0.0, 0.0]
    # Convención óptica REP-103 respecto a camera_link: x->-Y (derecha), y->-Z (abajo), z->+X.
    optical = _rpy_matrix(*rpy_optical)
    expected = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    for row in range(3):
        assert optical[row] == pytest.approx(expected[row], abs=1e-9)
    rot = _matmul(_rpy_matrix(*rpy), optical)
    optical_z = [rot[row][2] for row in range(3)]
    # El eje óptico (z) apunta a +X de base_link, inclinado 6.1° hacia abajo.
    assert optical_z[0] == pytest.approx(math.cos(CAMERA_PITCH), abs=1e-6)
    assert optical_z[2] == pytest.approx(-math.sin(CAMERA_PITCH), abs=1e-6)


def _sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def test_meshes_resolve_and_match_upstream(robot):
    uris = {m.get('filename') for m in robot.iter('mesh')}
    prefix = 'package://colmibot_description/meshes/'
    assert all(uri.startswith(prefix) for uri in uris), uris
    assert {uri[len(prefix):] for uri in uris} == EXPECTED_MESHES
    with open(os.path.join(MESH_DIR, 'SHA256SUMS')) as f:
        expected = dict(reversed(line.split()) for line in f if line.strip())
    assert set(expected) == EXPECTED_MESHES
    missing = sorted(n for n in EXPECTED_MESHES if not os.path.isfile(os.path.join(MESH_DIR, n)))
    assert not missing, f'Mallas ausentes {missing}: recupéralas según SOURCE.md'
    for name in EXPECTED_MESHES:
        assert _sha256(os.path.join(MESH_DIR, name)) == expected[name], name


def test_description_has_no_simulator_plugins(robot):
    assert robot.find('gazebo') is None
    assert not list(robot.iter('plugin'))


def test_primitive_mode_still_builds():
    robot = _robot('false')
    assert not list(robot.iter('mesh'))
    assert {link.get('name') for link in robot.findall('link')} == EXPECTED_LINKS - {
        n for n in EXPECTED_LINKS if n.startswith('vis_')}


def test_check_urdf(robot):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(ET.tostring(robot, encoding='unicode'))
    try:
        result = subprocess.run(['check_urdf', f.name], capture_output=True, text=True, timeout=30)
    finally:
        os.unlink(f.name)
    assert result.returncode == 0, result.stderr
    assert 'robot name is: colmibot' in result.stdout
