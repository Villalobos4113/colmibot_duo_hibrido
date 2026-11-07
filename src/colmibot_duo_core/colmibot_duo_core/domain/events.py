"""Eventos inmutables de misión: unidad de persistencia y de reconstrucción de estado."""

from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
import json
import re
from typing import Any

from colmibot_duo_core.domain.errors import ValidationError

ROBOT_IDS = ('A', 'B')

_IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
_STATION = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$')


def validate_robot_id(value: Any) -> str:
    """Devuelve el robot_id si es 'A' o 'B'."""
    if value not in ROBOT_IDS:
        raise ValidationError(f'robot_id debe ser uno de {ROBOT_IDS}, no {value!r}')
    return value


def validate_identifier(name: str, value: Any) -> str:
    """Devuelve un identificador de evento, misión u obstáculo si es válido."""
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValidationError(f'{name} inválido: {value!r}')
    return value


def validate_station(value: Any) -> str:
    """Devuelve un nombre de estación o zona simbólica si es válido."""
    if not isinstance(value, str) or not _STATION.fullmatch(value):
        raise ValidationError(f'estación inválida: {value!r}')
    return value


class EventType(str, Enum):
    """Tipos de evento persistidos. El valor es el texto guardado en SQLite."""

    MISSION_CREATED = 'MISSION_CREATED'
    MISSION_STARTED = 'MISSION_STARTED'
    MISSION_PAUSED = 'MISSION_PAUSED'
    MISSION_RESUMED = 'MISSION_RESUMED'
    STATION_REACHED = 'STATION_REACHED'
    OBSTACLE_DETECTED = 'OBSTACLE_DETECTED'
    OBSTACLE_CLEARED = 'OBSTACLE_CLEARED'
    FINDING_RECORDED = 'FINDING_RECORDED'
    MISSION_COMPLETED = 'MISSION_COMPLETED'
    MISSION_CANCELLED = 'MISSION_CANCELLED'


# En estos eventos la zona la aporta quien llama. En los demás es contexto derivado
# del estado (última estación confirmada) y no participa en la comparación de reintentos.
ZONE_IS_INPUT = frozenset({EventType.STATION_REACHED, EventType.FINDING_RECORDED})


@dataclass(frozen=True)
class Event:
    """
    Hecho ocurrido en una misión. Una vez creado no cambia.

    ts es una etiqueta de reloj de pared aportada por un reloj inyectado. No se usa para
    medir tiempos físicos ni para decidir transiciones.
    """

    event_id: str
    robot_id: str
    mission_id: str
    ts: datetime
    event_type: EventType
    zone: str | None = None
    image_ref: str | None = None
    payload: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """Valida y normaliza los campos."""
        validate_identifier('event_id', self.event_id)
        validate_robot_id(self.robot_id)
        validate_identifier('mission_id', self.mission_id)
        if not isinstance(self.ts, datetime) or self.ts.tzinfo is None:
            raise ValidationError('ts debe ser un datetime con zona horaria')
        try:
            object.__setattr__(self, 'ts', self.ts.astimezone(timezone.utc))
        except (OverflowError, ValueError) as exc:
            raise ValidationError(f'ts fuera de rango: {self.ts!r}') from exc
        try:
            object.__setattr__(self, 'event_type', EventType(self.event_type))
        except ValueError as exc:
            raise ValidationError(f'tipo de evento desconocido: {self.event_type!r}') from exc
        if self.zone is not None:
            validate_station(self.zone)
        if self.image_ref is not None:
            if (not isinstance(self.image_ref, str) or not self.image_ref
                    or len(self.image_ref) > 256 or not self.image_ref.isprintable()):
                raise ValidationError(f'image_ref inválido: {self.image_ref!r}')
        if not isinstance(self.payload, Mapping):
            raise ValidationError('payload debe ser un mapeo')
        try:
            canonical = json.dumps(self.payload, sort_keys=True, separators=(',', ':'),
                                   ensure_ascii=False, allow_nan=False)
            canonical.encode('utf-8')  # rechaza sustitutos Unicode sueltos
        except RecursionError as exc:
            raise ValidationError('payload demasiado anidado') from exc
        except (TypeError, ValueError) as exc:
            raise ValidationError(f'payload no es JSON serializable: {exc}') from exc
        object.__setattr__(self, 'payload', json.loads(canonical))

    def payload_json(self) -> str:
        """Devuelve el payload como JSON canónico."""
        return json.dumps(self.payload, sort_keys=True, separators=(',', ':'),
                          ensure_ascii=False, allow_nan=False)

    def ts_iso(self) -> str:
        """Devuelve ts en ISO 8601 UTC con microsegundos."""
        return self.ts.isoformat(timespec='microseconds')

    def content_key(self) -> tuple:
        """Contenido que identifica un reintento. Excluye ts y la zona derivada."""
        zone = self.zone if self.event_type in ZONE_IS_INPUT else None
        return (self.event_type.value, self.mission_id, zone, self.image_ref,
                self.payload_json())
