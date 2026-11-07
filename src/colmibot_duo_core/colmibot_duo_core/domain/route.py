"""Catálogo de estaciones simbólicas y validación de rutas planificadas."""

from collections.abc import Iterable

from colmibot_duo_core.domain.errors import ValidationError
from colmibot_duo_core.domain.events import validate_station


def normalize_route(route: Iterable[str]) -> tuple[str, ...]:
    """Valida solo la forma de una ruta: lista o tupla no vacía de estaciones sin repetir."""
    if not isinstance(route, (list, tuple)):
        raise ValidationError('la ruta debe ser una lista o tupla de estaciones')
    planned = tuple(validate_station(name) for name in route)
    if not planned:
        raise ValidationError('la ruta planificada no puede estar vacía')
    if len(set(planned)) != len(planned):
        raise ValidationError('la ruta planificada no admite estaciones repetidas')
    return planned


class StationCatalog:
    """Conjunto de estaciones o zonas conocidas. Son nombres, no coordenadas."""

    def __init__(self, stations: Iterable[str]):
        """Crea el catálogo; rechaza vacío, nombres inválidos y duplicados."""
        names = tuple(validate_station(name) for name in stations)
        if not names:
            raise ValidationError('el catálogo de estaciones no puede estar vacío')
        if len(set(names)) != len(names):
            raise ValidationError('el catálogo de estaciones tiene nombres repetidos')
        self._stations = names

    @property
    def stations(self) -> tuple[str, ...]:
        """Estaciones conocidas, en el orden dado."""
        return self._stations

    def __contains__(self, station: object) -> bool:
        """Indica si la estación es conocida."""
        return station in self._stations

    def validate_route(self, route: Iterable[str]) -> tuple[str, ...]:
        """Devuelve la ruta como tupla si es no vacía, sin repetidas y toda conocida."""
        planned = normalize_route(route)
        unknown = [name for name in planned if name not in self]
        if unknown:
            raise ValidationError(f'estaciones desconocidas en la ruta: {unknown}')
        return planned
