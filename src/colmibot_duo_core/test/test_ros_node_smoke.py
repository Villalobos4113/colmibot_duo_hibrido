"""Humo del adaptador ROS 2: ida y vuelta de comandos JSON en DDS local (SINTÉTICO)."""

import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import colmibot_duo_core
import pytest

DOMAIN_ID = 87
PACKAGE_ROOT = Path(colmibot_duo_core.__file__).resolve().parent.parent


@pytest.fixture
def ros_env(tmp_path, monkeypatch):
    """Nodo de misión del robot A, un nodo auxiliar y un ejecutor, sobre DDS local."""
    rclpy = pytest.importorskip('rclpy')  # sin ROS cargado las pruebas se omiten
    from colmibot_duo_core.adapters.ros_node import MissionNode
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    executor = SingleThreadedExecutor(context=context)
    mission = MissionNode(context=context, parameter_overrides=[
        Parameter('robot_id', value='A'),
        Parameter('db_dir', value=str(tmp_path)),
        Parameter('stations', value=['S1', 'S2']),
        Parameter('origin', value='synthetic'),
    ])
    probe = Node('probe', context=context)
    replies = []
    probe.create_subscription(
        String, '/mission_node/state', lambda msg: replies.append(json.loads(msg.data)), 10)
    publisher = probe.create_publisher(String, '/mission_node/command', 10)
    executor.add_node(mission)
    executor.add_node(probe)

    def publish_until(raw, matches, timeout=20.0):
        """Publica el texto hasta recibir la respuesta esperada; repetir es seguro."""
        payload = String()
        payload.data = raw
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            publisher.publish(payload)
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            for reply in replies:
                if matches(reply):
                    return reply
        raise AssertionError(f'sin la respuesta esperada para {raw[:80]!r}')

    def send(message):
        """Envía un comando con event_id; el id lo hace idempotente ante los reintentos."""
        return publish_until(
            json.dumps(message),
            lambda reply: message['event_id'] in reply['reply'].get('event_ids', []))

    yield SimpleNamespace(mission=mission, send=send, publish_until=publish_until,
                          Parameter=Parameter, String=String, db=tmp_path)
    executor.shutdown()
    mission.destroy_node()
    probe.destroy_node()
    rclpy.shutdown(context=context)


@pytest.mark.ros
def test_mission_node_round_trip(ros_env):
    from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
    send = ros_env.send
    # Salvo /parameter_events (infraestructura de rclpy), el adaptador solo publica
    # texto en ~/state: ningún otro tipo de mensaje ni tópico.
    own = [(pub.topic_name, pub.msg_type) for pub in ros_env.mission.publishers
           if pub.topic_name != '/parameter_events']
    assert own == [('/mission_node/state', ros_env.String)]
    send({'command': 'create_mission', 'mission_id': 'SYN-R1', 'route': ['S1', 'S2'],
          'event_id': 'ros-1'})
    send({'command': 'start', 'event_id': 'ros-2'})
    blocked = send({'command': 'obstacle', 'present': True, 'obstacle_id': 'OBS-1',
                    'event_id': 'ros-3'})
    assert blocked['reply']['state'] == 'BLOCKED'
    assert blocked['snapshot']['open_obstacles'] == ['OBS-1']
    send({'command': 'clear_obstacle', 'obstacle_id': 'OBS-1',
          'confirmed_by': 'op-sintetico', 'area_clear': True, 'event_id': 'ros-4'})
    send({'command': 'resume', 'event_id': 'ros-5'})
    send({'command': 'confirm_arrival', 'station': 'S1', 'event_id': 'ros-6'})
    done = send({'command': 'confirm_arrival', 'station': 'S2', 'event_id': 'ros-7'})
    assert done['snapshot']['state'] == 'COMPLETED'
    with EvidenceStore(ros_env.db / 'evidence_A.sqlite3', 'A') as store:
        assert [e.event_id for e in store.events()] == [
            'ros-1', 'ros-2', 'ros-3', 'ros-4', 'ros-5', 'ros-6', 'ros-7', 'ros-7.completed']


@pytest.mark.ros
@pytest.mark.parametrize(('raw', 'detail'), [
    # Listas anidadas: agotan la recursión del decodificador (RecursionError en json.loads).
    ('[' * 12000 + ']' * 12000, 'JSON inválido'),
    ('{no es json', 'JSON inválido'),
    # Objetos anidados: sí se decodifican, y el despachador responde con un error de dominio.
    ('{"command": ' * 6000 + '1' + '}' * 6000, 'command debe ser texto'),
])
def test_pathological_json_gets_an_error_reply_and_changes_nothing(ros_env, raw, detail):
    reply = ros_env.publish_until(raw, lambda r: r['reply'].get('detail') == detail)
    assert reply['reply']['ok'] is False
    assert reply['reply']['error'] == 'ValidationError'
    assert reply['snapshot']['mission_id'] is None      # el estado no cambió
    # El nodo sigue vivo y atiende comandos normales después del mensaje patológico.
    ok = ros_env.send({'command': 'create_mission', 'mission_id': 'SYN-R2', 'route': ['S1'],
                       'event_id': 'ros-after'})
    assert ok['reply']['ok'] is True


@pytest.mark.ros
@pytest.mark.parametrize(('name', 'value'), [
    ('robot_id', 'B'),
    ('origin', 'REAL-HARDWARE'),
    ('stations', ['S9']),
    ('db_dir', '/tmp/otro-directorio'),
])
def test_startup_only_parameters_cannot_be_changed_at_runtime(ros_env, name, value):
    before = ros_env.mission.get_parameter(name).value
    results = ros_env.mission.set_parameters([ros_env.Parameter(name, value=value)])
    assert [result.successful for result in results] == [False]
    assert ros_env.mission.get_parameter(name).value == before
    # La identidad efectiva sigue siendo la de arranque.
    reply = ros_env.send({'command': 'create_mission', 'mission_id': 'SYN-R3', 'route': ['S1'],
                          'event_id': 'ros-param'})
    assert reply['snapshot']['robot_id'] == 'A'


_MAIN = ('import sys; from colmibot_duo_core.adapters.ros_node import main; '
         'main(args=sys.argv)')


def _run_node_and_signal(tmp_path, sig, delay):
    """Lanza main() en un subproceso, envía la señal una vez arrancado y devuelve el resultado."""
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_DOMAIN_ID=str(DOMAIN_ID + 1), PYTHONDONTWRITEBYTECODE='1',
               PYTHONPATH=os.pathsep.join([str(PACKAGE_ROOT), os.environ.get('PYTHONPATH', '')]))
    proc = subprocess.Popen(
        [sys.executable, '-B', '-c', _MAIN, '--ros-args', '-p', f'db_dir:={tmp_path}'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    log, deadline = '', time.monotonic() + 30
    try:
        while 'mission_node listo' not in log and time.monotonic() < deadline:
            if proc.poll() is not None:
                break
            if select.select([proc.stderr], [], [], 0.2)[0]:
                log += proc.stderr.readline()
        assert 'mission_node listo' in log, f'el nodo no arrancó:\n{log}'
        time.sleep(delay)          # con delay > 0 el nodo ya está dentro de spin()
        proc.send_signal(sig)
        code = proc.wait(timeout=30)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    return code, log + proc.stderr.read()


@pytest.mark.ros
@pytest.mark.parametrize('sig', [signal.SIGTERM, signal.SIGINT], ids=['SIGTERM', 'SIGINT'])
def test_requested_shutdown_is_a_clean_exit(tmp_path, sig):
    pytest.importorskip('rclpy')
    from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
    code, stderr = _run_node_and_signal(tmp_path, sig, delay=2.0)
    assert code == 0, stderr
    assert 'Traceback' not in stderr, stderr
    # Con el nodo ya girando es el camino normal: no debe pasar por la rama de rescate.
    assert 'apagado durante el arranque' not in stderr, stderr
    # El archivo de evidencia queda íntegro y se puede reabrir.
    with EvidenceStore(tmp_path / 'evidence_A.sqlite3', 'A') as store:
        assert store.events() == []


@pytest.mark.ros
def test_context_shutdown_during_startup_is_a_normal_exit(tmp_path, monkeypatch, capfd):
    rclpy = pytest.importorskip('rclpy')
    from colmibot_duo_core.adapters import ros_node

    class ShutDownWhileBuilding:
        def __init__(self, *args, **kwargs):
            rclpy.shutdown()                     # lo que hace el manejador de SIGTERM
            raise RuntimeError('el contexto ya no es válido')

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setattr(ros_node, 'SingleThreadedExecutor', ShutDownWhileBuilding)
    ros_node.main(args=['--ros-args', '-p', f'db_dir:={tmp_path}'])   # no debe lanzar
    assert 'apagado durante el arranque' in capfd.readouterr().err
    assert not rclpy.ok()


@pytest.mark.ros
def test_signal_right_after_ready_is_also_a_clean_exit(tmp_path):
    pytest.importorskip('rclpy')
    # La señal llega apenas el nodo imprime que está listo: según la carrera, cae antes o
    # dentro de spin(). En ambos casos la salida debe ser normal.
    code, stderr = _run_node_and_signal(tmp_path, signal.SIGTERM, delay=0)
    assert code == 0, stderr


@pytest.mark.ros
def test_a_real_startup_failure_still_fails(tmp_path):
    pytest.importorskip('rclpy')
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_DOMAIN_ID=str(DOMAIN_ID + 1), PYTHONDONTWRITEBYTECODE='1',
               PYTHONPATH=os.pathsep.join([str(PACKAGE_ROOT), os.environ.get('PYTHONPATH', '')]))
    done = subprocess.run([sys.executable, '-B', '-c', _MAIN], capture_output=True, text=True,
                          env=env, timeout=60, check=False)      # sin db_dir obligatorio
    assert done.returncode != 0
    assert 'db_dir' in done.stderr
