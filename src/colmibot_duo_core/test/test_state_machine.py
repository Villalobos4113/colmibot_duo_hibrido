"""Máquina de estados: transiciones válidas e inválidas (datos SINTÉTICOS)."""

from itertools import product

from colmibot_duo_core.domain.errors import (
    InvalidTransition, MissionIdInUse, ValidationError)
from colmibot_duo_core.domain.events import EventType
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import ObstacleReport, Outcome
import pytest

STATES = ['IDLE', 'RUNNING', 'PAUSED', 'BLOCKED', 'COMPLETED', 'CANCELLED']

COMMANDS = {
    'start': lambda s: s.start(),
    'pause': lambda s: s.pause(),
    'resume': lambda s: s.resume(),
    'arrival': lambda s: s.confirm_arrival('S1'),
    'cancel': lambda s: s.cancel('prueba'),
    'obstacle': lambda s: s.report_obstacle(ObstacleReport(True, 'OBS-NEW')),
    'clear': lambda s: s.clear_obstacle('OBS-1', confirmed_by='operador-sintetico',
                                        area_clear=True),
}

# Transiciones válidas: (estado, comando) -> estado resultante. Todo lo demás es inválido.
VALID = {
    ('IDLE', 'start'): 'RUNNING',
    ('IDLE', 'cancel'): 'CANCELLED',
    ('RUNNING', 'pause'): 'PAUSED',
    ('RUNNING', 'arrival'): 'RUNNING',
    ('RUNNING', 'cancel'): 'CANCELLED',
    ('RUNNING', 'obstacle'): 'BLOCKED',
    ('PAUSED', 'resume'): 'RUNNING',
    ('PAUSED', 'cancel'): 'CANCELLED',
    ('PAUSED', 'obstacle'): 'BLOCKED',
    ('BLOCKED', 'cancel'): 'CANCELLED',
    ('BLOCKED', 'obstacle'): 'BLOCKED',
    ('BLOCKED', 'clear'): 'PAUSED',
}


@pytest.mark.parametrize(('state', 'command'), list(product(STATES, COMMANDS)))
def test_transition_matrix(make_rig, drive_to, state, command):
    rig = make_rig()
    drive_to(rig.service, state)
    assert rig.service.snapshot()['state'] == state
    before_snapshot = rig.service.snapshot()
    before_events = rig.store.events()

    if (state, command) in VALID:
        result = COMMANDS[command](rig.service)
        assert result.outcome is Outcome.APPLIED
        assert result.state.value == VALID[(state, command)]
        assert rig.service.snapshot()['state'] == VALID[(state, command)]
        assert len(rig.store.events()) > len(before_events)
    else:
        with pytest.raises(InvalidTransition):
            COMMANDS[command](rig.service)
        # Un comando inválido no cambia el estado ni escribe evidencia.
        assert rig.service.snapshot() == before_snapshot
        assert rig.store.events() == before_events


def test_create_gives_idle_mission_with_identity(make_rig):
    rig = make_rig('B')
    result = rig.service.create_mission('SYN-M1', ['S1', 'S2'])
    assert result.outcome is Outcome.APPLIED
    snapshot = rig.service.snapshot()
    assert snapshot['robot_id'] == 'B'
    assert snapshot['mission_id'] == 'SYN-M1'
    assert snapshot['state'] == 'IDLE'
    assert snapshot['planned_route'] == ['S1', 'S2']
    assert snapshot['executed_route'] == []


@pytest.mark.parametrize('robot_id', ['C', 'a', '', None, 'AB'])
def test_robot_id_must_be_a_or_b(tmp_path, robot_id):
    with pytest.raises(ValidationError):
        EvidenceStore(tmp_path / 'x.sqlite3', robot_id)


def test_full_valid_lifecycle_event_order(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2'])
    service.start()
    service.pause()
    service.resume()
    service.confirm_arrival('S1')
    result = service.confirm_arrival('S2')
    assert result.state.value == 'COMPLETED'
    assert [e.event_type for e in rig.store.events()] == [
        EventType.MISSION_CREATED, EventType.MISSION_STARTED, EventType.MISSION_PAUSED,
        EventType.MISSION_RESUMED, EventType.STATION_REACHED, EventType.STATION_REACHED,
        EventType.MISSION_COMPLETED]


def test_blocked_never_resumes_by_itself(make_rig, drive_to):
    service = make_rig().service
    drive_to(service, 'BLOCKED')
    for attempt in (service.resume, service.start, service.pause):
        with pytest.raises(InvalidTransition):
            attempt()
    with pytest.raises(InvalidTransition):
        service.confirm_arrival('S1')
    # Una lectura "sin obstáculo" tampoco recupera: se ignora y no escribe nada.
    result = service.report_obstacle(ObstacleReport(present=False))
    assert result.outcome is Outcome.IGNORED
    assert result.events == ()
    assert service.snapshot()['state'] == 'BLOCKED'


@pytest.mark.parametrize('state', ['IDLE', 'PAUSED', 'BLOCKED', 'COMPLETED', 'CANCELLED'])
def test_no_advance_outside_running(make_rig, drive_to, state):
    service = make_rig().service
    drive_to(service, state)
    executed = service.snapshot()['executed_route']
    with pytest.raises(InvalidTransition):
        service.confirm_arrival(service.snapshot()['next_station'] or 'S1')
    assert service.snapshot()['executed_route'] == executed


def test_commands_without_mission_are_rejected(make_rig):
    service = make_rig().service
    for attempt in (service.start, service.pause, service.resume,
                    lambda: service.confirm_arrival('S1')):
        with pytest.raises(InvalidTransition):
            attempt()


def test_single_active_mission_and_unique_mission_id(make_rig):
    service = make_rig().service
    service.create_mission('SYN-M1', ['S1'])
    with pytest.raises(InvalidTransition):
        service.create_mission('SYN-M2', ['S1'])
    service.cancel('cierre sintético')
    with pytest.raises(MissionIdInUse):
        service.create_mission('SYN-M1', ['S1'])
    assert service.create_mission('SYN-M2', ['S1']).outcome is Outcome.APPLIED
