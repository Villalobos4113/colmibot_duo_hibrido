"""Despachador de comandos JSON hacia el servicio, sin ROS (SINTÉTICO)."""

from colmibot_duo_core.adapters.command_dispatch import dispatch
import pytest


@pytest.fixture
def service(make_rig):
    return make_rig().service


def _send(service, **message):
    return dispatch(service, message)


def test_full_mission_through_commands(service):
    created = _send(service, command='create_mission', mission_id='SYN-D1',
                    route=['S1', 'S2'], event_id='d-1')
    assert created == {'ok': True, 'outcome': 'applied', 'state': 'IDLE',
                       'event_ids': ['d-1'], 'reason': ''}
    assert _send(service, command='start', event_id='d-2')['state'] == 'RUNNING'
    assert _send(service, command='confirm_arrival', station='S1',
                 event_id='d-3')['state'] == 'RUNNING'
    blocked = _send(service, command='obstacle', present=True, obstacle_id='OBS-1',
                    detail='caja sintética', event_id='d-4')
    assert blocked['state'] == 'BLOCKED'
    assert _send(service, command='obstacle', present=False)['outcome'] == 'ignored'
    cleared = _send(service, command='clear_obstacle', obstacle_id='OBS-1',
                    confirmed_by='op-sintetico', area_clear=True, event_id='d-5')
    assert cleared['state'] == 'PAUSED'
    assert _send(service, command='resume', event_id='d-6')['state'] == 'RUNNING'
    done = _send(service, command='confirm_arrival', station='S2', event_id='d-7')
    assert done['state'] == 'COMPLETED'
    assert done['event_ids'] == ['d-7', 'd-7.completed']


def test_retry_through_commands_is_a_duplicate(service):
    _send(service, command='create_mission', mission_id='SYN-D1', route=['S1'], event_id='d-1')
    first = _send(service, command='start', event_id='d-2')
    again = _send(service, command='start', event_id='d-2')
    assert first['outcome'] == 'applied'
    assert again['outcome'] == 'duplicate'
    assert again['event_ids'] == ['d-2']


def test_finding_command(service):
    _send(service, command='create_mission', mission_id='SYN-D1', route=['S1', 'S2'])
    _send(service, command='start')
    _send(service, command='confirm_arrival', station='S1')
    reply = _send(service, command='finding', zone='S1', label='marca-sintetica',
                  note='dato sintético', image_ref='synthetic://sin-archivo')
    assert reply['ok'] is True
    assert service.snapshot()['findings'] == 1


@pytest.mark.parametrize(('message', 'error'), [
    ({'command': 'despegar'}, 'ValidationError'),
    ({}, 'ValidationError'),
    ({'command': 'create_mission', 'route': ['S1']}, 'ValidationError'),   # falta mission_id
    ({'command': 'start'}, 'InvalidTransition'),                           # sin misión
    ({'command': 'obstacle', 'present': True}, 'InvalidTransition'),       # sin misión
    ({'command': 'obstacle', 'present': 'si'}, 'ValidationError'),         # no booleano
])
def test_domain_errors_become_error_replies(service, message, error):
    reply = dispatch(service, message)
    assert reply['ok'] is False
    assert reply['error'] == error


def test_non_object_message_is_rejected(service):
    for bad in ('start', 3, None, ['start']):
        reply = dispatch(service, bad)
        assert reply['ok'] is False
        assert reply['error'] == 'ValidationError'


def test_recovery_conditions_are_enforced_through_commands(service):
    _send(service, command='create_mission', mission_id='SYN-D1', route=['S1'])
    _send(service, command='start')
    _send(service, command='obstacle', present=True, obstacle_id='OBS-1')
    for extra in ({'area_clear': 'true'}, {'area_clear': False}, {}):
        reply = _send(service, command='clear_obstacle', obstacle_id='OBS-1',
                      confirmed_by='op-sintetico', **extra)
        assert reply['ok'] is False
        assert reply['state'] == 'BLOCKED'
