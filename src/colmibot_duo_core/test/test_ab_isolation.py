"""Aislamiento de los registros de los robots A y B (SINTÉTICO)."""

from datetime import datetime, timezone

from colmibot_duo_core.domain.errors import (
    InvalidTransition, IsolationError, MissionIdInUse, UnknownMission)
from colmibot_duo_core.domain.events import Event, EventType
from colmibot_duo_core.domain.mission import Mission
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService, ObstacleReport, Outcome
import pytest

TS = datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)


def test_separate_files_do_not_mix(make_rig):
    a, b = make_rig('A'), make_rig('B')
    # Mismo event_id en ambos robots: cada robot tiene su propio espacio de identificadores.
    assert a.service.create_mission('SYN-A-1', ['S1', 'S2'], event_id='shared-1').outcome \
        is Outcome.APPLIED
    assert b.service.create_mission('SYN-B-1', ['S3', 'S4'], event_id='shared-1').outcome \
        is Outcome.APPLIED
    a.service.start()
    a.service.confirm_arrival('S1')
    b.service.start()

    assert {e.robot_id for e in a.store.events()} == {'A'}
    assert {e.robot_id for e in b.store.events()} == {'B'}
    assert {e.mission_id for e in a.store.events()} == {'SYN-A-1'}
    assert {e.mission_id for e in b.store.events()} == {'SYN-B-1'}
    assert a.service.snapshot()['executed_route'] == ['S1']
    assert b.service.snapshot()['executed_route'] == []
    assert a.path != b.path


def test_shared_file_is_isolated_by_robot(make_rig, catalog):
    a = make_rig('A', filename='compartido.sqlite3')
    b = make_rig('B', filename='compartido.sqlite3')
    assert a.path == b.path
    a.service.create_mission('SYN-A-1', ['S1', 'S2'], event_id='same-id')
    b.service.create_mission('SYN-B-1', ['S3'], event_id='same-id')
    a.service.start(event_id='a-start')
    b.service.start(event_id='b-start')

    assert a.store.mission_ids() == ['SYN-A-1']
    assert b.store.mission_ids() == ['SYN-B-1']
    assert {e.robot_id for e in a.store.events()} == {'A'}
    assert {e.robot_id for e in b.store.events()} == {'B'}
    assert a.store.get('b-start') is None
    assert b.store.get('a-start') is None
    assert a.store.get('same-id').mission_id == 'SYN-A-1'
    assert b.store.get('same-id').mission_id == 'SYN-B-1'

    # Cada robot reconstruye solo su misión.
    for rig, mission_id in ((a, 'SYN-A-1'), (b, 'SYN-B-1')):
        fresh = MissionService(rig.store, catalog, lambda: TS, origin='synthetic')
        assert fresh.recover().mission_id == mission_id


def test_cross_robot_writes_are_refused(make_rig):
    a = make_rig('A', filename='compartido.sqlite3')
    b = make_rig('B', filename='compartido.sqlite3')
    a.service.create_mission('SYN-A-1', ['S1', 'S2'])
    a.service.start()
    before = a.store.events()

    # B no puede reutilizar el mission_id de A.
    with pytest.raises(MissionIdInUse):
        b.service.create_mission('SYN-A-1', ['S1'])
    # B no puede escribir eventos en la misión de A.
    intruder = Event(event_id='x-1', robot_id='B', mission_id='SYN-A-1', ts=TS,
                     event_type=EventType.MISSION_PAUSED, payload={'origin': 'synthetic'})
    with pytest.raises(UnknownMission):
        b.store.append(intruder)
    # El almacén de A rechaza eventos marcados como de B.
    with pytest.raises(IsolationError):
        a.store.append(intruder)
    assert a.store.events() == before
    assert b.store.events() == []


def test_mission_rejects_events_of_another_robot_or_mission():
    created = Event(event_id='c', robot_id='A', mission_id='SYN-A-1', ts=TS,
                    event_type=EventType.MISSION_CREATED,
                    payload={'planned_route': ['S1']})
    mission = Mission.from_created_event(created)
    for robot_id, mission_id in (('B', 'SYN-A-1'), ('A', 'SYN-OTRA')):
        foreign = Event(event_id='f', robot_id=robot_id, mission_id=mission_id, ts=TS,
                        event_type=EventType.MISSION_STARTED)
        with pytest.raises(IsolationError):
            mission.apply(foreign)
    assert mission.state.value == 'IDLE'


def test_ab_scenario_with_restart(make_rig, catalog):
    """Escenario sintético A/B: A completa con obstáculo y hallazgo; B se queda en pausa."""
    a, b = make_rig('A'), make_rig('B')

    a.service.create_mission('SYN-A-001', ['S1', 'S2', 'S3'])
    a.service.start()
    a.service.confirm_arrival('S1')
    a.service.record_finding('S1', 'marca-sintetica-1', note='dato sintético')
    a.service.confirm_arrival('S2')
    a.service.report_obstacle(ObstacleReport(True, 'OBS-A-1', 'caja sintética antes de S3'))
    with pytest.raises(InvalidTransition):
        a.service.confirm_arrival('S3')  # bloqueado: no hay avance lógico
    a.service.clear_obstacle('OBS-A-1', confirmed_by='operador-sintetico', area_clear=True)
    a.service.resume()
    a.service.confirm_arrival('S3')

    b.service.create_mission('SYN-B-001', ['S2', 'S3'])
    b.service.start()
    b.service.confirm_arrival('S2')
    b.service.pause()

    # Reinicio: se cierran ambos almacenes y se reabren desde disco.
    for rig in (a, b):
        rig.store.close()
    reopened = {}
    for robot_id, rig in (('A', a), ('B', b)):
        store = EvidenceStore(rig.path, robot_id)
        service = MissionService(store, catalog, lambda: TS, origin='synthetic')
        service.recover()
        reopened[robot_id] = (store, service)

    store_a, service_a = reopened['A']
    store_b, service_b = reopened['B']
    assert service_a.snapshot()['state'] == 'COMPLETED'
    assert service_a.snapshot()['executed_route'] == ['S1', 'S2', 'S3']
    assert service_b.snapshot()['state'] == 'PAUSED'
    assert [e.event_type.value for e in store_a.events()] == [
        'MISSION_CREATED', 'MISSION_STARTED', 'STATION_REACHED', 'FINDING_RECORDED',
        'STATION_REACHED', 'OBSTACLE_DETECTED', 'OBSTACLE_CLEARED', 'MISSION_RESUMED',
        'STATION_REACHED', 'MISSION_COMPLETED']
    assert [e.event_type.value for e in store_b.events()] == [
        'MISSION_CREATED', 'MISSION_STARTED', 'STATION_REACHED', 'MISSION_PAUSED']

    print('\n--- ESCENARIO SINTÉTICO A/B (tras reinicio) ---')
    for robot_id, store in (('A', store_a), ('B', store_b)):
        for event in store.events():
            print(f'{robot_id} {event.mission_id} {event.event_id:<8} '
                  f'{event.event_type.value:<18} zona={event.zone}')
    store_a.close()
    store_b.close()
