"""
Máquina de estados de la misión simbólica.

El estado se obtiene aplicando eventos en orden (reductor). Esto permite reconstruir
la misión desde la evidencia persistida. Aquí no hay velocidades, tiempos físicos ni
odometría: solo estaciones confirmadas por eventos sintéticos.
"""

from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import Enum

from colmibot_duo_core.domain.errors import (
    InvalidTransition, IsolationError, RouteViolation, ValidationError)
from colmibot_duo_core.domain.events import (
    Event, EventType, validate_identifier, validate_station)


class MissionState(str, Enum):
    """Estados de la misión."""

    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    PAUSED = 'PAUSED'
    BLOCKED = 'BLOCKED'
    COMPLETED = 'COMPLETED'
    CANCELLED = 'CANCELLED'


TERMINAL_STATES = frozenset({MissionState.COMPLETED, MissionState.CANCELLED})

# Tabla de transiciones simples (estado actual, evento) -> estado siguiente.
# Las transiciones con reglas adicionales (obstáculos, estaciones, cierre) se resuelven
# en Mission.apply y están documentadas en el README.
TRANSITIONS = {
    (MissionState.IDLE, EventType.MISSION_STARTED): MissionState.RUNNING,
    (MissionState.RUNNING, EventType.MISSION_PAUSED): MissionState.PAUSED,
    (MissionState.PAUSED, EventType.MISSION_RESUMED): MissionState.RUNNING,
    (MissionState.RUNNING, EventType.OBSTACLE_DETECTED): MissionState.BLOCKED,
    (MissionState.PAUSED, EventType.OBSTACLE_DETECTED): MissionState.BLOCKED,
    (MissionState.BLOCKED, EventType.OBSTACLE_DETECTED): MissionState.BLOCKED,
    (MissionState.BLOCKED, EventType.OBSTACLE_CLEARED): MissionState.PAUSED,
    (MissionState.RUNNING, EventType.MISSION_COMPLETED): MissionState.COMPLETED,
    (MissionState.IDLE, EventType.MISSION_CANCELLED): MissionState.CANCELLED,
    (MissionState.RUNNING, EventType.MISSION_CANCELLED): MissionState.CANCELLED,
    (MissionState.PAUSED, EventType.MISSION_CANCELLED): MissionState.CANCELLED,
    (MissionState.BLOCKED, EventType.MISSION_CANCELLED): MissionState.CANCELLED,
    # Avanzar a una estación y registrar un hallazgo no cambian el estado.
    (MissionState.RUNNING, EventType.STATION_REACHED): MissionState.RUNNING,
    (MissionState.RUNNING, EventType.FINDING_RECORDED): MissionState.RUNNING,
    (MissionState.PAUSED, EventType.FINDING_RECORDED): MissionState.PAUSED,
    (MissionState.BLOCKED, EventType.FINDING_RECORDED): MissionState.BLOCKED,
    # Anotar un hallazgo en una estación ya visitada sigue siendo posible al completar
    # (la última llegada cierra la misión de forma atómica).
    (MissionState.COMPLETED, EventType.FINDING_RECORDED): MissionState.COMPLETED,
}


@dataclass
class Mission:
    """Misión de un robot: ruta planificada, ruta ejecutada y estado."""

    robot_id: str
    mission_id: str
    planned_route: tuple[str, ...]
    state: MissionState = MissionState.IDLE
    executed_route: list[str] = field(default_factory=list)
    open_obstacles: dict[str, str | None] = field(default_factory=dict)
    finding_event_ids: list[str] = field(default_factory=list)

    @classmethod
    def from_created_event(cls, event: Event) -> 'Mission':
        """Crea la misión a partir de MISSION_CREATED."""
        if event.event_type is not EventType.MISSION_CREATED:
            raise ValidationError('el primer evento de una misión debe ser MISSION_CREATED')
        route = event.payload.get('planned_route')
        if not isinstance(route, list) or not route:
            raise ValidationError('MISSION_CREATED requiere planned_route no vacía')
        planned = tuple(validate_station(name) for name in route)
        if len(set(planned)) != len(planned):
            raise ValidationError('planned_route no admite estaciones repetidas')
        return cls(robot_id=event.robot_id, mission_id=event.mission_id, planned_route=planned)

    @classmethod
    def replay(cls, events: Iterable[Event]) -> 'Mission':
        """Reconstruye la misión aplicando sus eventos en orden."""
        iterator = iter(events)
        try:
            first = next(iterator)
        except StopIteration:
            raise ValidationError('no hay eventos para reconstruir la misión') from None
        mission = cls.from_created_event(first)
        for event in iterator:
            mission.apply(event)
        return mission

    @property
    def next_station(self) -> str | None:
        """Siguiente estación planificada, o None si ya se ejecutó toda la ruta."""
        if len(self.executed_route) < len(self.planned_route):
            return self.planned_route[len(self.executed_route)]
        return None

    @property
    def last_station(self) -> str | None:
        """Última estación confirmada, o None si aún no se confirma ninguna."""
        return self.executed_route[-1] if self.executed_route else None

    @property
    def is_terminal(self) -> bool:
        """Indica si la misión terminó: COMPLETED solo admite anotar hallazgos; CANCELLED, nada."""
        return self.state in TERMINAL_STATES

    def apply(self, event: Event) -> None:
        """Aplica un evento. Si no es válido lanza una excepción y no modifica nada."""
        if event.robot_id != self.robot_id or event.mission_id != self.mission_id:
            raise IsolationError('el evento pertenece a otro robot o a otra misión')
        kind = event.event_type
        if kind is EventType.MISSION_CREATED:
            raise InvalidTransition('la misión ya fue creada')
        target = TRANSITIONS.get((self.state, kind))
        if target is None:
            raise InvalidTransition(f'{kind.value} no es válido en el estado {self.state.value}')
        # Cada método valida todo antes de mutar: si lanza una excepción, no cambió nada.
        if kind is EventType.STATION_REACHED:
            self._apply_station(event)
        elif kind is EventType.OBSTACLE_DETECTED:
            self._apply_obstacle_detected(event)
        elif kind is EventType.OBSTACLE_CLEARED:
            target = self._apply_obstacle_cleared(event)
        elif kind is EventType.FINDING_RECORDED:
            self._apply_finding(event)
        elif kind is EventType.MISSION_COMPLETED:
            if self.next_station is not None:
                raise RouteViolation(f'faltan estaciones por confirmar: {self.next_station}')
        self.state = target

    def snapshot(self) -> dict:
        """Resumen serializable del estado actual."""
        return {
            'robot_id': self.robot_id,
            'mission_id': self.mission_id,
            'state': self.state.value,
            'planned_route': list(self.planned_route),
            'executed_route': list(self.executed_route),
            'next_station': self.next_station,
            'open_obstacles': sorted(self.open_obstacles),
            'findings': len(self.finding_event_ids),
        }

    def _apply_station(self, event: Event) -> None:
        expected = self.next_station
        if expected is None:
            raise RouteViolation('la ruta planificada ya fue ejecutada completa')
        if event.zone != expected:
            raise RouteViolation(
                f'se esperaba la estación {expected!r} y se confirmó {event.zone!r}')
        self.executed_route.append(event.zone)

    def _apply_obstacle_detected(self, event: Event) -> None:
        obstacle_id = validate_identifier('obstacle_id', event.payload.get('obstacle_id'))
        if obstacle_id in self.open_obstacles:
            raise InvalidTransition(f'el obstáculo {obstacle_id!r} ya está abierto')
        self.open_obstacles[obstacle_id] = event.zone

    def _apply_obstacle_cleared(self, event: Event) -> MissionState:
        obstacle_id = validate_identifier('obstacle_id', event.payload.get('obstacle_id'))
        if obstacle_id not in self.open_obstacles:
            raise InvalidTransition(f'el obstáculo {obstacle_id!r} no está abierto')
        confirmed_by = event.payload.get('confirmed_by')
        if not isinstance(confirmed_by, str) or not confirmed_by.strip():
            raise ValidationError('la recuperación requiere confirmed_by (quién la confirma)')
        if event.payload.get('area_clear') is not True:
            raise ValidationError('la recuperación requiere area_clear=true explícito')
        del self.open_obstacles[obstacle_id]
        # Despejar el último obstáculo deja la misión en PAUSED, nunca en RUNNING:
        # reanudar exige un MISSION_RESUMED explícito.
        return MissionState.BLOCKED if self.open_obstacles else MissionState.PAUSED

    def _apply_finding(self, event: Event) -> None:
        if event.zone not in self.executed_route:
            raise RouteViolation(
                f'solo se registran hallazgos en estaciones ya confirmadas, no en {event.zone!r}')
        label = event.payload.get('label')
        if not isinstance(label, str) or not label.strip():
            raise ValidationError('el hallazgo requiere una etiqueta no vacía')
        self.finding_event_ids.append(event.event_id)
