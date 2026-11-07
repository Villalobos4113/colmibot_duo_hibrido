"""
Servicio de misión: valida, persiste y aplica eventos de forma atómica.

Todas las entradas son eventos sintéticos o simulados confirmados por quien llama.
El servicio no calcula velocidades, no usa temporizadores y no emite órdenes de movimiento.
Un reintento con el mismo event_id y el mismo contenido devuelve DUPLICATE sin efecto.
"""

from collections.abc import Callable
import copy
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Any
import uuid

from colmibot_duo_core.domain.errors import (
    EventConflict, InvalidTransition, ValidationError)
from colmibot_duo_core.domain.events import (
    Event, EventType, validate_identifier, validate_station)
from colmibot_duo_core.domain.mission import Mission, MissionState
from colmibot_duo_core.domain.route import normalize_route, StationCatalog
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore


# El evento de cierre se deriva del event_id de la última llegada; estos límites garantizan que
# el id derivado siga siendo un identificador válido y que no choque con ids del llamador.
COMPLETED_SUFFIX = '.completed'
MAX_EVENT_ID_LENGTH = 100


class Outcome(str, Enum):
    """Resultado de un comando."""

    APPLIED = 'applied'
    DUPLICATE = 'duplicate'
    IGNORED = 'ignored'


@dataclass(frozen=True)
class Result:
    """Resultado de un comando: qué pasó, qué eventos y en qué estado quedó la misión."""

    outcome: Outcome
    events: tuple[Event, ...]
    state: MissionState | None
    reason: str = ''


@dataclass(frozen=True)
class ObstacleReport:
    """
    Reporte de obstáculo simulado.

    present=False nunca despeja un bloqueo: la ausencia de lectura no es una recuperación.
    """

    present: bool
    obstacle_id: str | None = None
    detail: str = ''


class MissionService:
    """Casos de uso de una misión para un robot, respaldados por un EvidenceStore."""

    def __init__(self, store: EvidenceStore, catalog: StationCatalog,
                 clock: Callable[[], datetime], *, origin: str,
                 id_factory: Callable[[], str] | None = None):
        """
        Crea el servicio.

        clock devuelve un datetime con zona horaria (se inyecta para pruebas).
        origin etiqueta cada evento (por ejemplo 'synthetic'). No tiene valor por defecto
        para que nunca se mezclen datos sintéticos con datos de otro origen sin querer.
        """
        if not isinstance(origin, str) or not origin.strip():
            raise ValidationError('origin es obligatorio (por ejemplo "synthetic")')
        self._store = store
        self._catalog = catalog
        self._clock = clock
        self._origin = origin
        self._id_factory = id_factory or (lambda: uuid.uuid4().hex)
        self._mission: Mission | None = None

    @property
    def robot_id(self) -> str:
        """Robot al que sirve este servicio."""
        return self._store.robot_id

    @property
    def mission(self) -> Mission | None:
        """Copia de la misión actual (o la última), o None."""
        return copy.deepcopy(self._mission)

    def snapshot(self) -> dict:
        """Resumen serializable de la misión actual."""
        if self._mission is None:
            return {'robot_id': self.robot_id, 'mission_id': None, 'state': None,
                    'planned_route': [], 'executed_route': [], 'next_station': None,
                    'open_obstacles': [], 'findings': 0}
        return self._mission.snapshot()

    def recover(self) -> Mission | None:
        """Reconstruye la última misión del robot desde la evidencia persistida."""
        mission_ids = self._store.mission_ids()
        if not mission_ids:
            self._mission = None
            return None
        self._mission = Mission.replay(self._store.events(mission_ids[-1]))
        return self.mission

    def create_mission(self, mission_id: str, route: list[str], *,
                       event_id: str | None = None) -> Result:
        """Crea una misión en IDLE con su ruta planificada."""
        validate_identifier('mission_id', mission_id)
        planned = normalize_route(route)
        event = self._new_event(EventType.MISSION_CREATED, mission_id=mission_id,
                                event_id=event_id, payload={'planned_route': list(planned)})
        # Un reintento de una creación ya persistida se resuelve en _commit (duplicado o
        # conflicto) sin consultar el catálogo actual: la recuperación conserva la ruta con la
        # que se creó. Solo una creación nueva debe pertenecer al catálogo vigente.
        if self._store.get(event.event_id) is None:
            self._catalog.validate_route(planned)
        return self._commit(event)

    def start(self, *, event_id: str | None = None) -> Result:
        """Inicia la misión (IDLE a RUNNING)."""
        return self._lifecycle(EventType.MISSION_STARTED, event_id)

    def pause(self, *, event_id: str | None = None) -> Result:
        """Pausa la misión (RUNNING a PAUSED)."""
        return self._lifecycle(EventType.MISSION_PAUSED, event_id)

    def resume(self, *, event_id: str | None = None) -> Result:
        """Reanuda la misión (PAUSED a RUNNING). Desde BLOCKED no es válido."""
        return self._lifecycle(EventType.MISSION_RESUMED, event_id)

    def cancel(self, reason: str = '', *, event_id: str | None = None) -> Result:
        """Cancela la misión desde IDLE, RUNNING, PAUSED o BLOCKED."""
        if not isinstance(reason, str):
            raise ValidationError('reason debe ser texto')
        return self._lifecycle(EventType.MISSION_CANCELLED, event_id, {'reason': reason})

    def confirm_arrival(self, station: str, *, event_id: str | None = None) -> Result:
        """
        Confirma (evento sintético) la llegada a la siguiente estación planificada.

        Si era la última, se escribe también MISSION_COMPLETED en la misma transacción.
        """
        validate_station(station)
        mission = self._require_mission()
        event = self._new_event(EventType.STATION_REACHED, mission_id=mission.mission_id,
                                event_id=event_id, zone=station)
        return self._commit(event)

    def report_obstacle(self, report: ObstacleReport, *,
                        event_id: str | None = None) -> Result:
        """Registra un obstáculo simulado y bloquea el avance lógico (BLOCKED)."""
        if not isinstance(report, ObstacleReport) or not isinstance(report.present, bool):
            raise ValidationError('report debe ser un ObstacleReport con present booleano')
        mission = self._require_mission()
        if not report.present:
            return Result(Outcome.IGNORED, (), mission.state,
                          'present=False no despeja un bloqueo; usar clear_obstacle')
        if not isinstance(report.detail, str) or len(report.detail) > 500:
            raise ValidationError('detail debe ser texto de hasta 500 caracteres')
        event_id = self._resolve_event_id(event_id)
        obstacle_id = (report.obstacle_id if report.obstacle_id is not None
                       else f'OBS-{event_id}')
        # Zona derivada: el tramo hacia la siguiente estación (o la última confirmada).
        zone = mission.next_station or mission.last_station
        event = self._new_event(
            EventType.OBSTACLE_DETECTED, mission_id=mission.mission_id, event_id=event_id,
            zone=zone, payload={'obstacle_id': obstacle_id, 'detail': report.detail})
        return self._commit(event)

    def clear_obstacle(self, obstacle_id: str, *, confirmed_by: str, area_clear: bool,
                       event_id: str | None = None) -> Result:
        """
        Recuperación explícita de un obstáculo.

        Condiciones comprobables: la misión está BLOCKED, el obstáculo está abierto,
        confirmed_by identifica a quien confirma y area_clear es exactamente True.
        Al despejar el último obstáculo la misión pasa a PAUSED, no a RUNNING.
        """
        validate_identifier('obstacle_id', obstacle_id)
        mission = self._require_mission()
        event = self._new_event(
            EventType.OBSTACLE_CLEARED, mission_id=mission.mission_id, event_id=event_id,
            zone=mission.last_station,
            payload={'obstacle_id': obstacle_id, 'confirmed_by': confirmed_by,
                     'area_clear': area_clear})
        return self._commit(event)

    def record_finding(self, zone: str, label: str, *, note: str = '',
                       image_ref: str | None = None,
                       event_id: str | None = None) -> Result:
        """
        Registra un hallazgo en una estación ya confirmada.

        image_ref es una referencia opaca opcional. Nunca se genera ni se valida contra
        archivos: este núcleo no produce ni lee imágenes.
        """
        validate_station(zone)
        if not isinstance(label, str) or not isinstance(note, str):
            raise ValidationError('label y note deben ser texto')
        mission = self._require_mission()
        event = self._new_event(
            EventType.FINDING_RECORDED, mission_id=mission.mission_id, event_id=event_id,
            zone=zone, image_ref=image_ref, payload={'label': label, 'note': note})
        return self._commit(event)

    def _lifecycle(self, event_type: EventType, event_id: str | None,
                   payload: dict | None = None) -> Result:
        mission = self._require_mission()
        event = self._new_event(event_type, mission_id=mission.mission_id,
                                event_id=event_id, zone=mission.last_station, payload=payload)
        return self._commit(event)

    def _resolve_event_id(self, event_id: str | None) -> str:
        """Valida el event_id del llamador o genera uno. Reserva el sufijo del cierre."""
        if event_id is None:
            return self._id_factory()
        validate_identifier('event_id', event_id)
        if len(event_id) > MAX_EVENT_ID_LENGTH or event_id.endswith(COMPLETED_SUFFIX):
            raise ValidationError(
                f'event_id de hasta {MAX_EVENT_ID_LENGTH} caracteres y sin el sufijo '
                f'reservado {COMPLETED_SUFFIX!r}')
        return event_id

    def _require_mission(self) -> Mission:
        if self._mission is None:
            raise InvalidTransition('no hay misión; crear una con create_mission')
        return self._mission

    def _new_event(self, event_type: EventType, *, mission_id: str, event_id: str | None,
                   zone: str | None = None, image_ref: str | None = None,
                   payload: dict[str, Any] | None = None) -> Event:
        body = dict(payload or {})
        body['origin'] = self._origin
        return Event(event_id=self._resolve_event_id(event_id), robot_id=self.robot_id,
                     mission_id=mission_id, ts=self._clock(), event_type=event_type,
                     zone=zone, image_ref=image_ref, payload=body)

    def _commit(self, primary: Event) -> Result:
        existing = self._store.get(primary.event_id)
        if existing is not None:
            if existing.content_key() != primary.content_key():
                raise EventConflict(
                    f'event_id {primary.event_id!r} ya existe con contenido distinto')
            same = self._mission is not None and self._mission.mission_id == existing.mission_id
            state = self._mission.state if same else None
            return Result(Outcome.DUPLICATE, (existing,), state, 'reintento sin efecto')
        if primary.event_type is EventType.MISSION_CREATED:
            if self._mission is None and self._store.mission_ids():
                self.recover()  # evita crear sobre una misión persistida sin recuperar
            if self._mission is not None and not self._mission.is_terminal:
                raise InvalidTransition(
                    f'ya hay una misión activa ({self._mission.mission_id}); '
                    'cancelarla o completarla antes de crear otra')
            working = Mission.from_created_event(primary)
        else:
            working = copy.deepcopy(self._require_mission())
            working.apply(primary)
        events = [primary]
        if primary.event_type is EventType.STATION_REACHED and working.next_station is None:
            closing = Event(event_id=f'{primary.event_id}{COMPLETED_SUFFIX}',
                            robot_id=primary.robot_id,
                            mission_id=primary.mission_id, ts=primary.ts,
                            event_type=EventType.MISSION_COMPLETED, zone=working.last_station,
                            payload={'origin': self._origin})
            working.apply(closing)
            events.append(closing)
        # Si la escritura falla, la excepción sale antes de reemplazar el estado en memoria.
        self._store.append_many(events)
        self._mission = working
        return Result(Outcome.APPLIED, tuple(events), working.state)
