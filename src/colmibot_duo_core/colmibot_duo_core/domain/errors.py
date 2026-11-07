"""Excepciones del dominio. Todas derivan de DomainError."""


class DomainError(Exception):
    """Regla de negocio violada. El estado de la misión no cambia."""


class ValidationError(DomainError):
    """Dato de entrada malformado o fuera de rango."""


class InvalidTransition(DomainError):
    """Evento no permitido en el estado actual de la misión."""


class RouteViolation(DomainError):
    """Evento incompatible con la ruta planificada o con lo ya ejecutado."""


class EventConflict(DomainError):
    """Un event_id ya existe con contenido distinto."""


class IsolationError(DomainError):
    """Intento de mezclar registros de robots distintos."""


class MissionIdInUse(DomainError):
    """El mission_id ya fue usado por otra misión."""


class UnknownMission(DomainError):
    """La misión no existe para este robot."""


class StoreError(DomainError):
    """El archivo de evidencia no tiene el formato esperado."""
