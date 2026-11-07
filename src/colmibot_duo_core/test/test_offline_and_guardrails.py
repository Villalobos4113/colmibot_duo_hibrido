"""Sin red y sin actuación: el núcleo funciona offline y no puede mover ni abrir hardware."""

import ast
from pathlib import Path
import re
import socket
import subprocess
import sys

import colmibot_duo_core
from colmibot_duo_core.service import ObstacleReport
import pytest

PACKAGE_DIR = Path(colmibot_duo_core.__file__).resolve().parent
PACKAGE_ROOT = PACKAGE_DIR.parent
SOURCES = sorted(PACKAGE_DIR.rglob('*.py'))
ROS_ADAPTER = PACKAGE_DIR / 'adapters' / 'ros_node.py'

PURE_ALLOWED = {'colmibot_duo_core', 'collections', 'contextlib', 'copy', 'dataclasses',
                'datetime', 'enum', 'json', 'pathlib', 're', 'sqlite3', 'typing', 'uuid'}
ROS_ALLOWED = PURE_ALLOWED | {'rcl_interfaces', 'rclpy', 'std_msgs'}

# Nada que mueva, mida tiempo físico o abra dispositivos puede aparecer en el código fuente.
FORBIDDEN_WORDS = re.compile(
    r'\b(Twist|cmd_vel|geometry_msgs|nav_msgs|serial|termios|create_timer|sleep|'
    r'threading|asyncio|odom|velocity)\b')


def _imports(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            names.add(node.module.split('.')[0])
    return names


def _full_cycle(service):
    service.create_mission('SYN-OFFLINE-1', ['S1', 'S2', 'S3'])
    service.start()
    service.confirm_arrival('S1')
    service.report_obstacle(ObstacleReport(True, 'OBS-1', 'obstáculo sintético'))
    service.clear_obstacle('OBS-1', confirmed_by='op-sintetico', area_clear=True)
    service.resume()
    service.confirm_arrival('S2')
    return service.confirm_arrival('S3')


def test_full_cycle_works_with_the_network_blocked(make_rig, monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError('acceso a red prohibido en el núcleo')

    for name in ('socket', 'create_connection', 'getaddrinfo', 'gethostbyname'):
        monkeypatch.setattr(socket, name, refuse)
    result = _full_cycle(make_rig().service)
    assert result.state.value == 'COMPLETED'


_OFFLINE_CHILD = """
import sys
from datetime import datetime, timezone
sys.modules['socket'] = None   # cualquier "import socket" falla en este proceso
sys.path.insert(0, sys.argv[2])
from colmibot_duo_core.domain.route import StationCatalog
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService, ObstacleReport

clock = lambda: datetime(2026, 1, 1, tzinfo=timezone.utc)
with EvidenceStore(sys.argv[1], 'B') as store:
    service = MissionService(store, StationCatalog(['S1', 'S2']), clock, origin='synthetic')
    service.create_mission('SYN-OFF-2', ['S1', 'S2'])
    service.start()
    service.confirm_arrival('S1')
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))
    service.clear_obstacle('OBS-1', confirmed_by='op', area_clear=True)
    service.resume()
    state = service.confirm_arrival('S2').state.value
assert 'rclpy' not in sys.modules, 'el núcleo no debe importar ROS'
print('OFFLINE_OK', state)
"""


def test_core_runs_in_a_process_where_sockets_cannot_be_imported(tmp_path):
    done = subprocess.run(
        [sys.executable, '-B', '-I', '-c', _OFFLINE_CHILD, str(tmp_path / 'off.sqlite3'),
         str(PACKAGE_ROOT)], capture_output=True, text=True, check=False)
    assert done.returncode == 0, done.stderr
    assert 'OFFLINE_OK COMPLETED' in done.stdout


@pytest.mark.parametrize('path', SOURCES, ids=lambda p: str(p.relative_to(PACKAGE_DIR)))
def test_source_imports_are_restricted(path):
    allowed = ROS_ALLOWED if path == ROS_ADAPTER else PURE_ALLOWED
    assert _imports(path) <= allowed, sorted(_imports(path) - allowed)


@pytest.mark.parametrize('path', SOURCES, ids=lambda p: str(p.relative_to(PACKAGE_DIR)))
def test_source_has_no_motion_serial_or_timing_code(path):
    text = path.read_text(encoding='utf-8')
    assert FORBIDDEN_WORDS.findall(text) == []


def test_only_the_adapter_touches_ros():
    for path in SOURCES:
        if path != ROS_ADAPTER:
            assert 'rclpy' not in _imports(path), path
    # El adaptador solo publica texto (std_msgs/String).
    adapter = ROS_ADAPTER.read_text(encoding='utf-8')
    assert adapter.count('create_publisher(') == 1
    assert 'create_publisher(String,' in adapter


def test_no_evidence_files_inside_the_source_tree():
    leftovers = [p for pattern in ('*.sqlite3', '*.db', '*.sqlite3-journal')
                 for p in PACKAGE_ROOT.rglob(pattern)]
    assert leftovers == []
