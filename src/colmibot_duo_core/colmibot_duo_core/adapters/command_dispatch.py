"""
Traduce mensajes de comando (diccionarios JSON) a llamadas del servicio de misión.

No depende de ROS: se prueba sin ningún middleware. Cada respuesta es un diccionario
serializable con ok, outcome, state y los event_id involucrados.
"""

from collections.abc import Callable, Mapping
from typing import Any

from colmibot_duo_core.domain.errors import DomainError, ValidationError
from colmibot_duo_core.service import MissionService, ObstacleReport, Result


def dispatch(service: MissionService, message: Any) -> dict:
    """Ejecuta un comando y devuelve siempre una respuesta, sin propagar errores de dominio."""
    try:
        return _run(service, message)
    except DomainError as exc:
        mission = service.mission
        return {'ok': False, 'error': type(exc).__name__, 'detail': str(exc),
                'state': mission.state.value if mission else None}


def _run(service: MissionService, message: Any) -> dict:
    if not isinstance(message, Mapping):
        raise ValidationError('el comando debe ser un objeto JSON')
    name = message.get('command')
    if not isinstance(name, str):
        raise ValidationError('command debe ser texto')
    if name == 'snapshot':
        snapshot = service.snapshot()
        return {'ok': True, 'outcome': 'snapshot', 'state': snapshot['state'],
                'event_ids': [], 'reason': ''}
    handler = _COMMANDS.get(name)
    if handler is None:
        raise ValidationError(f'comando desconocido: {name!r}')
    result = handler(service, message)
    return {'ok': True, 'outcome': result.outcome.value,
            'state': result.state.value if result.state else None,
            'event_ids': [event.event_id for event in result.events],
            'reason': result.reason}


def _need(message: Mapping, key: str) -> Any:
    if key not in message:
        raise ValidationError(f'falta el campo {key!r}')
    return message[key]


def _event_id(message: Mapping) -> str | None:
    return message.get('event_id')


def _create(service: MissionService, message: Mapping) -> Result:
    return service.create_mission(_need(message, 'mission_id'), _need(message, 'route'),
                                  event_id=_event_id(message))


def _arrival(service: MissionService, message: Mapping) -> Result:
    return service.confirm_arrival(_need(message, 'station'), event_id=_event_id(message))


def _obstacle(service: MissionService, message: Mapping) -> Result:
    report = ObstacleReport(present=_need(message, 'present'),
                            obstacle_id=message.get('obstacle_id'),
                            detail=message.get('detail', ''))
    return service.report_obstacle(report, event_id=_event_id(message))


def _clear(service: MissionService, message: Mapping) -> Result:
    return service.clear_obstacle(_need(message, 'obstacle_id'),
                                  confirmed_by=message.get('confirmed_by'),
                                  area_clear=message.get('area_clear'),
                                  event_id=_event_id(message))


def _finding(service: MissionService, message: Mapping) -> Result:
    return service.record_finding(_need(message, 'zone'), _need(message, 'label'),
                                  note=message.get('note', ''),
                                  image_ref=message.get('image_ref'),
                                  event_id=_event_id(message))


def _cancel(service: MissionService, message: Mapping) -> Result:
    return service.cancel(message.get('reason', ''), event_id=_event_id(message))


def _simple(method: str) -> Callable[[MissionService, Mapping], Result]:
    def handler(service: MissionService, message: Mapping) -> Result:
        return getattr(service, method)(event_id=_event_id(message))
    return handler


_COMMANDS: dict[str, Callable[[MissionService, Mapping], Result]] = {
    'create_mission': _create,
    'start': _simple('start'),
    'pause': _simple('pause'),
    'resume': _simple('resume'),
    'confirm_arrival': _arrival,
    'obstacle': _obstacle,
    'clear_obstacle': _clear,
    'finding': _finding,
    'cancel': _cancel,
}
