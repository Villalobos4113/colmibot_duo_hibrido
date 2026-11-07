"""Ruta simbólica: planificada frente a ejecutada, solo por eventos confirmados (SINTÉTICO)."""

from datetime import datetime, timezone

from colmibot_duo_core.domain.errors import (
    InvalidTransition, RouteViolation, ValidationError)
from colmibot_duo_core.domain.events import Event, EventType
from colmibot_duo_core.domain.mission import Mission
from colmibot_duo_core.service import Outcome
import pytest


def test_complete_route_sequence(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    service.start()
    for station in ('S1', 'S2'):
        assert service.confirm_arrival(station).state.value == 'RUNNING'
    last = service.confirm_arrival('S3')
    assert last.state.value == 'COMPLETED'
    # La llegada a la última estación y el cierre se escriben juntos.
    assert [e.event_type for e in last.events] == [
        EventType.STATION_REACHED, EventType.MISSION_COMPLETED]
    snapshot = service.snapshot()
    assert snapshot['planned_route'] == ['S1', 'S2', 'S3']
    assert snapshot['executed_route'] == ['S1', 'S2', 'S3']
    assert snapshot['next_station'] is None
    reached = [e.zone for e in rig.store.events() if e.event_type is EventType.STATION_REACHED]
    assert reached == ['S1', 'S2', 'S3']


def test_planned_route_differs_from_executed_route(make_rig):
    service = make_rig().service
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    service.start()
    service.confirm_arrival('S1')
    snapshot = service.snapshot()
    assert snapshot['planned_route'] == ['S1', 'S2', 'S3']
    assert snapshot['executed_route'] == ['S1']
    assert snapshot['next_station'] == 'S2'


@pytest.mark.parametrize('station', ['S3', 'S1', 'S4', 'ZZ'])
def test_out_of_order_or_unknown_arrival_is_rejected(make_rig, station):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    service.start()
    service.confirm_arrival('S1')
    before = rig.store.events()
    with pytest.raises(RouteViolation):
        service.confirm_arrival(station)
    assert service.snapshot()['executed_route'] == ['S1']
    assert rig.store.events() == before


def test_cannot_complete_before_last_station():
    ts = datetime(2026, 1, 1, tzinfo=timezone.utc)
    created = Event(event_id='c', robot_id='A', mission_id='SYN-M1', ts=ts,
                    event_type=EventType.MISSION_CREATED,
                    payload={'planned_route': ['S1', 'S2']})
    mission = Mission.from_created_event(created)
    mission.apply(Event(event_id='s', robot_id='A', mission_id='SYN-M1', ts=ts,
                        event_type=EventType.MISSION_STARTED))
    closing = Event(event_id='x', robot_id='A', mission_id='SYN-M1', ts=ts,
                    event_type=EventType.MISSION_COMPLETED)
    with pytest.raises(RouteViolation):
        mission.apply(closing)
    assert mission.state.value == 'RUNNING'


@pytest.mark.parametrize('route', [[], ['S1', 'S1'], ['S1', 'NOPE'], 'S1', ['bad name']])
def test_invalid_planned_routes(make_rig, route):
    service = make_rig().service
    with pytest.raises(ValidationError):
        service.create_mission('SYN-M1', route)


def test_arrival_after_completion_is_rejected(make_rig, drive_to):
    service = make_rig().service
    drive_to(service, 'COMPLETED')
    with pytest.raises(InvalidTransition):
        service.confirm_arrival('S1')


def test_retry_of_arrival_does_not_advance_twice(make_rig):
    service = make_rig().service
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    service.start()
    assert service.confirm_arrival('S1', event_id='arr-1').outcome is Outcome.APPLIED
    assert service.confirm_arrival('S1', event_id='arr-1').outcome is Outcome.DUPLICATE
    assert service.snapshot()['executed_route'] == ['S1']
    # Una segunda confirmación de S1 con otro event_id sí es una violación de ruta.
    with pytest.raises(RouteViolation):
        service.confirm_arrival('S1', event_id='arr-2')
