"""
Obstáculo simulado: bloqueo lógico y recuperación explícita (SINTÉTICO).

Estas pruebas verifican lógica de software. No demuestran parada física.
"""

from colmibot_duo_core.domain.errors import (
    InvalidTransition, RouteViolation, ValidationError)
from colmibot_duo_core.domain.events import EventType
from colmibot_duo_core.service import ObstacleReport, Outcome
import pytest


def _running_after_s1(service):
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    service.start()
    service.confirm_arrival('S1')


def test_obstacle_blocks_and_is_recorded(make_rig):
    rig = make_rig()
    service = rig.service
    _running_after_s1(service)
    result = service.report_obstacle(
        ObstacleReport(True, 'OBS-1', 'caja simulada en el tramo S1-S2'), event_id='obs-evt')
    assert result.outcome is Outcome.APPLIED
    assert result.state.value == 'BLOCKED'
    event = rig.store.get('obs-evt')
    assert event.event_type is EventType.OBSTACLE_DETECTED
    assert event.zone == 'S2'  # tramo hacia la siguiente estación planificada
    assert event.payload['obstacle_id'] == 'OBS-1'
    assert event.payload['origin'] == 'synthetic'
    with pytest.raises(InvalidTransition):
        service.confirm_arrival('S2')
    assert service.snapshot()['executed_route'] == ['S1']


def test_boolean_report_without_id_derives_a_stable_id(make_rig):
    service = make_rig().service
    _running_after_s1(service)
    first = service.report_obstacle(ObstacleReport(True), event_id='bool-1')
    assert first.events[0].payload['obstacle_id'] == 'OBS-bool-1'
    # El mismo reintento no abre un segundo obstáculo.
    assert service.report_obstacle(ObstacleReport(True), event_id='bool-1').outcome \
        is Outcome.DUPLICATE
    assert service.snapshot()['open_obstacles'] == ['OBS-bool-1']


def test_present_must_be_boolean(make_rig):
    service = make_rig().service
    _running_after_s1(service)
    with pytest.raises(ValidationError):
        service.report_obstacle(ObstacleReport('yes'))


def test_explicit_recovery_then_explicit_resume(make_rig):
    rig = make_rig()
    service = rig.service
    _running_after_s1(service)
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))

    cleared = service.clear_obstacle('OBS-1', confirmed_by='operador-sintetico',
                                     area_clear=True)
    # Despejar no reanuda: queda en PAUSED y el avance sigue prohibido.
    assert cleared.state.value == 'PAUSED'
    with pytest.raises(InvalidTransition):
        service.confirm_arrival('S2')

    assert service.resume().state.value == 'RUNNING'
    service.confirm_arrival('S2')
    assert service.confirm_arrival('S3').state.value == 'COMPLETED'
    assert service.snapshot()['executed_route'] == ['S1', 'S2', 'S3']
    kinds = [e.event_type for e in rig.store.events()]
    assert kinds.index(EventType.OBSTACLE_DETECTED) < kinds.index(EventType.OBSTACLE_CLEARED)
    cleared_event = rig.store.get(cleared.events[0].event_id)
    assert cleared_event.payload['confirmed_by'] == 'operador-sintetico'
    assert cleared_event.payload['area_clear'] is True


@pytest.mark.parametrize(('obstacle_id', 'confirmed_by', 'area_clear', 'error'), [
    ('OBS-OTHER', 'operador-sintetico', True, InvalidTransition),  # no es el obstáculo abierto
    ('OBS-1', '', True, ValidationError),                           # sin responsable
    ('OBS-1', '   ', True, ValidationError),
    ('OBS-1', None, True, ValidationError),
    ('OBS-1', 'operador-sintetico', False, ValidationError),        # área no despejada
    ('OBS-1', 'operador-sintetico', 'true', ValidationError),       # no es booleano True
    ('OBS-1', 'operador-sintetico', 1, ValidationError),
    ('OBS-1', 'operador-sintetico', None, ValidationError),
])
def test_recovery_requires_verifiable_conditions(make_rig, obstacle_id, confirmed_by,
                                                 area_clear, error):
    rig = make_rig()
    service = rig.service
    _running_after_s1(service)
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))
    before = rig.store.events()
    with pytest.raises(error):
        service.clear_obstacle(obstacle_id, confirmed_by=confirmed_by, area_clear=area_clear)
    assert service.snapshot()['state'] == 'BLOCKED'
    assert rig.store.events() == before


def test_each_open_obstacle_must_be_cleared(make_rig):
    service = make_rig().service
    _running_after_s1(service)
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))
    service.report_obstacle(ObstacleReport(True, 'OBS-2'))
    with pytest.raises(InvalidTransition):
        service.report_obstacle(ObstacleReport(True, 'OBS-2'))  # ya abierto
    first = service.clear_obstacle('OBS-1', confirmed_by='op-sintetico', area_clear=True)
    assert first.state.value == 'BLOCKED'
    second = service.clear_obstacle('OBS-2', confirmed_by='op-sintetico', area_clear=True)
    assert second.state.value == 'PAUSED'


def test_obstacle_while_paused_blocks_and_idle_rejects(make_rig):
    service = make_rig().service
    service.create_mission('SYN-M1', ['S1', 'S2'])
    with pytest.raises(InvalidTransition):
        service.report_obstacle(ObstacleReport(True, 'OBS-0'))
    service.start()
    service.pause()
    assert service.report_obstacle(ObstacleReport(True, 'OBS-1')).state.value == 'BLOCKED'
    with pytest.raises(InvalidTransition):
        service.resume()


def test_cancel_from_blocked_is_allowed(make_rig, drive_to):
    service = make_rig().service
    drive_to(service, 'BLOCKED')
    assert service.cancel('abortada en prueba sintética').state.value == 'CANCELLED'


def test_finding_rules(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    with pytest.raises(InvalidTransition):  # IDLE: aún no hay inspección
        service.record_finding('S1', 'marca-sintetica')
    service.start()
    with pytest.raises(RouteViolation):     # estación aún no confirmada
        service.record_finding('S1', 'marca-sintetica')
    service.confirm_arrival('S1')
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))
    # Observar es posible aun bloqueado; no mueve la misión.
    result = service.record_finding('S1', 'marca-sintetica', note='dato sintético',
                                    event_id='fnd-1')
    assert result.state.value == 'BLOCKED'
    with pytest.raises(ValidationError):
        service.record_finding('S1', '   ')
    assert service.snapshot()['findings'] == 1


def test_finding_at_the_last_station_is_possible_after_completion(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1'])
    service.start()
    assert service.confirm_arrival('S1').state.value == 'COMPLETED'
    result = service.record_finding('S1', 'marca-final-sintetica', event_id='fnd-last')
    assert result.state.value == 'COMPLETED'
    assert rig.store.get('fnd-last').zone == 'S1'
    # En una misión cancelada ya no se registra nada.
    service.create_mission('SYN-M2', ['S1'])
    service.cancel('cierre sintético')
    with pytest.raises(InvalidTransition):
        service.record_finding('S1', 'tarde')
