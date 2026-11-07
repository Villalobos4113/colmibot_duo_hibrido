"""Regresiones de la revisión independiente de C-002 y huecos de la suite (SINTÉTICO)."""

import ast
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
import threading

import colmibot_duo_core
from colmibot_duo_core.adapters.command_dispatch import dispatch
from colmibot_duo_core.domain.errors import (
    EventConflict, InvalidTransition, ValidationError)
from colmibot_duo_core.domain.events import (
    Event, EventType, validate_identifier, validate_station)
from colmibot_duo_core.domain.route import StationCatalog
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService, ObstacleReport, Outcome
import pytest

PACKAGE_DIR = Path(colmibot_duo_core.__file__).resolve().parent
TS = datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)


# --- validación de entradas -------------------------------------------------------------

@pytest.mark.parametrize('value', ['E1\n', 'E1 ', ' E1', '', 'a\nb', '\n'])
def test_identifiers_reject_trailing_newline_and_spaces(value):
    with pytest.raises(ValidationError):
        validate_identifier('event_id', value)


@pytest.mark.parametrize('value', ['S1\n', 'S 1', '', 'S1\n\n'])
def test_stations_reject_trailing_newline(value):
    with pytest.raises(ValidationError):
        validate_station(value)
    with pytest.raises(ValidationError):
        StationCatalog(['S2', value])


def test_mission_and_event_ids_with_newline_are_rejected(make_rig):
    service = make_rig().service
    with pytest.raises(ValidationError):
        service.create_mission('M1\n', ['S1'])
    service.create_mission('M1', ['S1', 'S2'])
    with pytest.raises(ValidationError):
        service.start(event_id='E1\n')


@pytest.mark.parametrize('route', [None, 5, 1.5, True, {'S1', 'S2'}, {'S1': 1}, object()])
def test_route_must_be_a_list_or_tuple(make_rig, route):
    service = make_rig().service
    with pytest.raises(ValidationError):
        service.create_mission('M1', route)
    assert dispatch(service, {'command': 'create_mission', 'mission_id': 'M1',
                              'route': route})['error'] == 'ValidationError'


@pytest.mark.parametrize('command', [[], {}, None, 5, ['start']])
def test_command_must_be_text(make_rig, command):
    reply = dispatch(make_rig().service, {'command': command})
    assert reply['ok'] is False
    assert reply['error'] == 'ValidationError'


@pytest.mark.parametrize('bad', ['', 0, False, {}, []])
def test_explicit_but_empty_event_ids_are_rejected_not_replaced(make_rig, bad):
    service = make_rig().service
    service.create_mission('M1', ['S1', 'S2'])
    with pytest.raises(ValidationError):
        service.start(event_id=bad)
    with pytest.raises(ValidationError):
        service.report_obstacle(ObstacleReport(True), event_id=bad)
    assert service.snapshot()['state'] == 'IDLE'


def test_obstacle_id_empty_is_rejected_not_replaced(make_rig):
    service = make_rig().service
    service.create_mission('M1', ['S1', 'S2'])
    service.start()
    for bad in ('', 0, {}):
        with pytest.raises(ValidationError):
            service.report_obstacle(ObstacleReport(True, obstacle_id=bad))
    assert service.snapshot()['state'] == 'RUNNING'


def test_lone_surrogates_become_validation_errors(make_rig):
    service = make_rig().service
    service.create_mission('M1', ['S1', 'S2'])
    service.start()
    service.confirm_arrival('S1')
    reply = dispatch(service, {'command': 'finding', 'zone': 'S1', 'label': 'x',
                               'note': 'a\ud800'})
    assert reply['ok'] is False
    assert reply['error'] == 'ValidationError'
    assert service.snapshot()['findings'] == 0


def test_timestamp_out_of_range_is_a_validation_error():
    extreme = datetime(1, 1, 1, tzinfo=timezone(timedelta(hours=5)))
    with pytest.raises(ValidationError):
        Event(event_id='e', robot_id='A', mission_id='M1', ts=extreme,
              event_type=EventType.MISSION_STARTED)


# --- identificadores derivados y reintentos ---------------------------------------------

def test_reserved_suffix_and_length_for_caller_event_ids(make_rig):
    service = make_rig().service
    service.create_mission('M1', ['S1', 'S2'])
    for bad in ('X.completed', 'x' * 101):
        with pytest.raises(ValidationError):
            service.start(event_id=bad)
    assert service.start(event_id='x' * 100).outcome is Outcome.APPLIED


def test_arrival_and_closing_are_written_atomically(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('M1', ['S1', 'S2'])
    service.start()
    service.confirm_arrival('S1')
    # Alguien escribió ya un evento distinto con el id que usaría el cierre.
    rig.store.append(Event(event_id='X.completed', robot_id='A', mission_id='M1', ts=TS,
                           event_type=EventType.MISSION_PAUSED,
                           payload={'origin': 'synthetic'}))
    before = rig.store.events()
    with pytest.raises(EventConflict):
        service.confirm_arrival('S2', event_id='X')
    assert rig.store.get('X') is None            # la llegada tampoco quedó escrita
    assert rig.store.events() == before
    assert service.snapshot()['executed_route'] == ['S1']
    assert service.snapshot()['state'] == 'RUNNING'


def test_late_retry_of_an_obstacle_report_is_still_a_duplicate(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('M1', ['S1', 'S2', 'S3'])
    service.start()
    service.confirm_arrival('S1')
    first = service.report_obstacle(ObstacleReport(True, 'OBS-1', 'caja'), event_id='obs-e1')
    service.clear_obstacle('OBS-1', confirmed_by='op-sintetico', area_clear=True)
    service.resume()
    service.confirm_arrival('S2')
    count = len(rig.store.events())
    again = service.report_obstacle(ObstacleReport(True, 'OBS-1', 'caja'), event_id='obs-e1')
    assert first.outcome is Outcome.APPLIED
    assert again.outcome is Outcome.DUPLICATE
    assert len(rig.store.events()) == count
    assert service.snapshot()['state'] == 'RUNNING'


def test_duplicate_of_an_old_mission_event_does_not_report_the_current_state(make_rig):
    service = make_rig().service
    service.create_mission('M1', ['S1'], event_id='C1')
    service.cancel('cierre sintético')
    service.create_mission('M2', ['S1'])
    service.start()
    retry = service.create_mission('M1', ['S1'], event_id='C1')
    assert retry.outcome is Outcome.DUPLICATE
    assert retry.state is None   # la misión actual (M2) no es la del evento reintentado


@pytest.mark.parametrize('call_a, call_b', [
    (lambda s: s.cancel('motivo A', event_id='k'), lambda s: s.cancel('motivo B', event_id='k')),
    (lambda s: s.report_obstacle(ObstacleReport(True, 'O1', 'a'), event_id='k'),
     lambda s: s.report_obstacle(ObstacleReport(True, 'O1', 'b'), event_id='k')),
    (lambda s: s.record_finding('S1', 'etiqueta-a', event_id='k'),
     lambda s: s.record_finding('S1', 'etiqueta-b', event_id='k')),
    (lambda s: s.record_finding('S1', 'etiqueta', image_ref='synthetic://a', event_id='k'),
     lambda s: s.record_finding('S1', 'etiqueta', image_ref='synthetic://b', event_id='k')),
])
def test_same_id_with_different_inputs_conflicts(make_rig, call_a, call_b):
    service = make_rig().service
    service.create_mission('M1', ['S1', 'S2'])
    service.start()
    service.confirm_arrival('S1')
    call_a(service)
    with pytest.raises(EventConflict):
        call_b(service)


# --- recuperación ------------------------------------------------------------------------

def test_recover_returns_the_latest_mission_and_create_recovers_by_itself(make_rig, catalog):
    rig = make_rig()
    service = rig.service
    service.create_mission('X1', ['S1'])
    service.start()
    service.confirm_arrival('S1')                 # X1 completada
    service.create_mission('X2', ['S1', 'S2'])    # X2 queda en IDLE
    rig.store.close()

    with EvidenceStore(rig.path, 'A') as store:
        recovered = MissionService(store, catalog, lambda: TS, origin='synthetic')
        assert recovered.recover().mission_id == 'X2'
        # Un servicio nuevo que olvidó llamar a recover() no puede crear sobre una activa.
        forgetful = MissionService(store, catalog, lambda: TS, origin='synthetic')
        with pytest.raises(InvalidTransition):
            forgetful.create_mission('X3', ['S1'])


def test_findings_in_every_state_that_allows_them(make_rig):
    def at_s1(service, mission_id):
        service.create_mission(mission_id, ['S1', 'S2'])
        service.start()
        service.confirm_arrival('S1')

    service = make_rig().service
    at_s1(service, 'M1')
    assert service.record_finding('S1', 'en-running').state.value == 'RUNNING'
    service.pause()
    assert service.record_finding('S1', 'en-paused').state.value == 'PAUSED'
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))
    assert service.record_finding('S1', 'en-blocked').state.value == 'BLOCKED'
    service.cancel('cierre sintético')
    with pytest.raises(InvalidTransition):
        service.record_finding('S1', 'en-cancelled')
    assert service.snapshot()['findings'] == 3


# --- apertura concurrente del esquema ---------------------------------------------------

def test_two_robots_can_open_a_new_shared_file_at_the_same_time(tmp_path):
    for attempt in range(15):
        path = tmp_path / f'nuevo-{attempt}.sqlite3'
        barrier = threading.Barrier(2)

        def open_store(robot_id):
            barrier.wait()
            with EvidenceStore(path, robot_id):
                return robot_id

        with ThreadPoolExecutor(max_workers=2) as pool:
            results = [future.result() for future in
                       [pool.submit(open_store, 'A'), pool.submit(open_store, 'B')]]
        assert sorted(results) == ['A', 'B']


# --- comandos JSON: cada uno hace lo que dice -------------------------------------------

def test_each_documented_command_has_its_own_effect(make_rig):
    service = make_rig().service
    sent = [
        ({'command': 'create_mission', 'mission_id': 'M1', 'route': ['S1', 'S2']}, 'IDLE'),
        ({'command': 'start'}, 'RUNNING'),
        ({'command': 'pause'}, 'PAUSED'),
        ({'command': 'snapshot'}, 'PAUSED'),
        ({'command': 'resume'}, 'RUNNING'),
        ({'command': 'obstacle', 'present': True, 'obstacle_id': 'OBS-1'}, 'BLOCKED'),
        ({'command': 'snapshot'}, 'BLOCKED'),
        ({'command': 'clear_obstacle', 'obstacle_id': 'OBS-1', 'confirmed_by': 'op-sintetico',
          'area_clear': True}, 'PAUSED'),
        ({'command': 'cancel', 'reason': 'cierre sintético'}, 'CANCELLED'),
    ]
    for message, expected in sent:
        reply = dispatch(service, message)
        assert reply['ok'] is True, (message, reply)
        assert reply['state'] == expected, (message, reply)
        assert service.snapshot()['state'] == expected


def test_clear_command_without_confirmed_by_is_an_error_not_a_default(make_rig):
    service = make_rig().service
    for message in ({'command': 'create_mission', 'mission_id': 'M1', 'route': ['S1']},
                    {'command': 'start'},
                    {'command': 'obstacle', 'present': True, 'obstacle_id': 'OBS-1'}):
        dispatch(service, message)
    reply = dispatch(service, {'command': 'clear_obstacle', 'obstacle_id': 'OBS-1',
                               'area_clear': True})
    assert reply['ok'] is False
    assert reply['state'] == 'BLOCKED'


def test_finding_command_uses_the_given_zone(make_rig):
    service = make_rig().service
    for message in ({'command': 'create_mission', 'mission_id': 'M1', 'route': ['S1', 'S2']},
                    {'command': 'start'},
                    {'command': 'confirm_arrival', 'station': 'S1'}):
        dispatch(service, message)
    bad = dispatch(service, {'command': 'finding', 'zone': 'S2', 'label': 'x'})
    assert bad['ok'] is False and bad['error'] == 'RouteViolation'
    good = dispatch(service, {'command': 'finding', 'zone': 'S1', 'label': 'x'})
    assert good['ok'] is True


# --- guardarraíles estáticos adicionales -------------------------------------------------

SOURCES = sorted(PACKAGE_DIR.rglob('*.py'))
FORBIDDEN_CALLS = {'open', 'create_rate', 'create_timer', 'create_service', 'create_client',
                   'create_guard_condition', 'create_subscription_with_options'}


def _calls(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            yield func.attr if isinstance(func, ast.Attribute) else getattr(func, 'id', '')


@pytest.mark.parametrize('path', SOURCES, ids=lambda p: str(p.relative_to(PACKAGE_DIR)))
def test_sources_never_open_files_or_create_timers_services_or_rates(path):
    assert not (set(_calls(path)) & FORBIDDEN_CALLS)


def test_ros_adapter_has_one_subscription_and_one_publisher_on_the_documented_topics():
    tree = ast.parse((PACKAGE_DIR / 'adapters' / 'ros_node.py').read_text(encoding='utf-8'))
    topics = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr in ('create_publisher', 'create_subscription')):
            topics.setdefault(node.func.attr, []).append(node.args[1].value)
    assert topics == {'create_publisher': ['~/state'], 'create_subscription': ['~/command']}


# --- empaquetado -----------------------------------------------------------------------

def test_entry_point_and_config_are_declared_and_exist():
    package_root = PACKAGE_DIR.parent
    setup_text = (package_root / 'setup.py').read_text(encoding='utf-8')
    assert "'mission_node = colmibot_duo_core.adapters.ros_node:main'" in setup_text
    assert "glob('config/*.yaml')" in setup_text
    adapter = ast.parse((PACKAGE_DIR / 'adapters' / 'ros_node.py').read_text(encoding='utf-8'))
    assert 'main' in {n.name for n in adapter.body if isinstance(n, ast.FunctionDef)}
    assert list((package_root / 'config').glob('*.yaml'))
    assert (package_root / 'resource' / 'colmibot_duo_core').is_file()


# --- hallazgos de la revisión de Codex (F1 hermano y F2) --------------------------------

def test_deeply_nested_value_inside_a_valid_command_is_a_validation_error(make_rig):
    service = make_rig().service
    for message in ({'command': 'create_mission', 'mission_id': 'M1', 'route': ['S1']},
                    {'command': 'start'},
                    {'command': 'obstacle', 'present': True, 'obstacle_id': 'OBS-1'}):
        dispatch(service, message)
    nested = []
    for _ in range(20000):   # supera el límite de recursión en C de json (≈10 000)
        nested = [nested]
    reply = dispatch(service, {'command': 'clear_obstacle', 'obstacle_id': 'OBS-1',
                               'confirmed_by': nested, 'area_clear': True})
    assert reply['ok'] is False
    assert reply['error'] == 'ValidationError'
    assert reply['state'] == 'BLOCKED'


def _two_catalog_runs(make_rig, first_catalog, second_catalog):
    """Crea una misión con un catálogo, reabre el archivo con otro y recupera."""
    rig = make_rig()
    first = MissionService(rig.store, StationCatalog(first_catalog), lambda: TS,
                           origin='synthetic')
    first.create_mission('M1', ['S1', 'S2'], event_id='c-1')
    rig.store.close()
    store = EvidenceStore(rig.path, 'A')
    second = MissionService(store, StationCatalog(second_catalog), lambda: TS,
                            origin='synthetic')
    assert second.recover().mission_id == 'M1'
    return store, second


def test_creation_retry_is_resolved_before_the_current_catalog(make_rig):
    store, service = _two_catalog_runs(make_rig, ['S1', 'S2'], ['S9'])
    try:
        before = store.events()
        retry = service.create_mission('M1', ['S1', 'S2'], event_id='c-1')
        assert retry.outcome is Outcome.DUPLICATE
        assert store.events() == before
        # La misión recuperada conserva su ruta original y puede avanzar con ella.
        assert service.snapshot()['planned_route'] == ['S1', 'S2']
    finally:
        store.close()


def test_new_creations_and_conflicting_retries_still_face_the_current_catalog(make_rig):
    store, service = _two_catalog_runs(make_rig, ['S1', 'S2'], ['S9'])
    try:
        service.cancel('cierre sintético')
        with pytest.raises(ValidationError):       # creación nueva: fuera del catálogo vigente
            service.create_mission('M2', ['S1'], event_id='c-2')
        with pytest.raises(EventConflict):         # mismo id, ruta distinta: conflicto
            service.create_mission('M1', ['S2', 'S1'], event_id='c-1')
        with pytest.raises(ValidationError):       # la forma de la ruta se valida siempre
            service.create_mission('M1', None, event_id='c-1')
        assert service.create_mission('M3', ['S9'], event_id='c-3').outcome is Outcome.APPLIED
    finally:
        store.close()
