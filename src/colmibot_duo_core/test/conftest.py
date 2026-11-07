"""Fixtures compartidas. Todos los datos de prueba son SINTÉTICOS."""

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import itertools
from pathlib import Path

from colmibot_duo_core.domain.route import StationCatalog
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService, ObstacleReport
import pytest

STATIONS = ('S1', 'S2', 'S3', 'S4')
ROUTE = ['S1', 'S2', 'S3']


class FakeClock:
    """Reloj controlado: cada lectura avanza un segundo desde una fecha fija."""

    def __init__(self, start=None):
        self._now = start or datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)

    def __call__(self):
        value = self._now
        self._now += timedelta(seconds=1)
        return value


def sequential_ids(prefix):
    """Genera event_id deterministas: prefijo-0001, prefijo-0002, ..."""
    counter = itertools.count(1)
    return lambda: f'{prefix}-{next(counter):04d}'


@dataclass
class Rig:
    """Servicio, almacén y ruta del archivo de evidencia de un robot de prueba."""

    service: MissionService
    store: EvidenceStore
    path: Path


@pytest.fixture
def catalog():
    return StationCatalog(STATIONS)


@pytest.fixture
def make_rig(tmp_path, catalog):
    """Fábrica de robots de prueba con SQLite en tmp_path (fuera del código fuente)."""
    stores = []

    def factory(robot_id='A', filename=None, clock=None):
        path = tmp_path / (filename or f'evidence_{robot_id}.sqlite3')
        store = EvidenceStore(path, robot_id)
        stores.append(store)
        service = MissionService(store, catalog, clock or FakeClock(), origin='synthetic',
                                 id_factory=sequential_ids(robot_id))
        return Rig(service, store, path)

    yield factory
    for store in stores:
        store.close()


@pytest.fixture
def drive_to():
    """Lleva un servicio recién creado al estado pedido con una misión sintética."""

    def drive(service, state, mission_id='SYN-M1'):
        service.create_mission(mission_id, ROUTE)
        if state == 'IDLE':
            return
        if state == 'CANCELLED':
            service.cancel('cancelación sintética')
            return
        service.start()
        if state == 'PAUSED':
            service.pause()
        elif state == 'BLOCKED':
            service.report_obstacle(ObstacleReport(True, 'OBS-1', 'obstáculo sintético'))
        elif state == 'COMPLETED':
            for station in ROUTE:
                service.confirm_arrival(station)
        else:
            assert state == 'RUNNING', state

    return drive
