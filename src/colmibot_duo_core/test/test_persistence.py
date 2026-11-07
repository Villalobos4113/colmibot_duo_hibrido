"""Persistencia SQLite: campos, reapertura, reinicio de proceso e idempotencia (SINTÉTICO)."""

from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path
import sqlite3
import subprocess
import sys

import colmibot_duo_core
from colmibot_duo_core.domain.errors import (
    EventConflict, StoreError, UnknownMission, ValidationError)
from colmibot_duo_core.domain.events import Event, EventType
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService, ObstacleReport, Outcome
import pytest

PACKAGE_ROOT = Path(colmibot_duo_core.__file__).resolve().parent.parent
TS = datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)


def _event(event_id, event_type=EventType.MISSION_CREATED, mission_id='SYN-M1', robot_id='A',
           **kwargs):
    kwargs.setdefault('payload', {'planned_route': ['S1'], 'origin': 'synthetic'})
    return Event(event_id=event_id, robot_id=robot_id, mission_id=mission_id, ts=TS,
                 event_type=event_type, **kwargs)


def test_events_carry_the_required_fields(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2'])
    service.start()
    service.confirm_arrival('S1')
    with closing(sqlite3.connect(rig.path)) as raw:
        rows = raw.execute(
            'SELECT event_id, robot_id, mission_id, ts, event_type, zone, image_ref '
            'FROM events ORDER BY seq').fetchall()
    assert rows == [
        ('A-0001', 'A', 'SYN-M1', '2026-01-01T12:00:00.000000+00:00', 'MISSION_CREATED',
         None, None),
        ('A-0002', 'A', 'SYN-M1', '2026-01-01T12:00:01.000000+00:00', 'MISSION_STARTED',
         None, None),
        ('A-0003', 'A', 'SYN-M1', '2026-01-01T12:00:02.000000+00:00', 'STATION_REACHED',
         'S1', None),
    ]
    with closing(sqlite3.connect(rig.path)) as raw:
        origins = {row[0] for row in raw.execute(
            "SELECT json_extract(payload, '$.origin') FROM events")}
    assert origins == {'synthetic'}  # todo evento queda etiquetado como sintético


def test_image_ref_is_optional_and_never_fabricated(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2'])
    service.start()
    service.confirm_arrival('S1')
    # La referencia apunta a un archivo inexistente: el núcleo no la valida ni crea nada.
    ref = 'synthetic://sin-archivo/hallazgo-1'
    service.record_finding('S1', 'marca-sintetica', event_id='f-none')
    service.record_finding('S1', 'marca-sintetica-2', image_ref=ref, event_id='f-ref')
    assert rig.store.get('f-none').image_ref is None
    assert rig.store.get('f-ref').image_ref == ref


def test_reopen_store_in_same_process_keeps_events(make_rig, catalog):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2', 'S3'])
    service.start()
    service.confirm_arrival('S1')
    service.record_finding('S1', 'marca-sintetica', event_id='f-1')
    service.report_obstacle(ObstacleReport(True, 'OBS-1'))
    persisted = rig.store.events()
    snapshot = service.snapshot()
    rig.store.close()

    with EvidenceStore(rig.path, 'A') as reopened:
        assert reopened.events() == persisted
        recovered = MissionService(reopened, catalog, lambda: TS, origin='synthetic')
        assert recovered.recover() is not None
        assert recovered.snapshot() == snapshot
        assert recovered.snapshot()['state'] == 'BLOCKED'
        recovered.clear_obstacle('OBS-1', confirmed_by='op-sintetico', area_clear=True)
        recovered.resume()
        recovered.confirm_arrival('S2')
        assert recovered.confirm_arrival('S3').state.value == 'COMPLETED'


_CHILD = """
import os, sys
from datetime import datetime, timezone
sys.path.insert(0, sys.argv[2])
from colmibot_duo_core.domain.route import StationCatalog
from colmibot_duo_core.persistence.sqlite_store import EvidenceStore
from colmibot_duo_core.service import MissionService, ObstacleReport

clock = lambda: datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
store = EvidenceStore(sys.argv[1], 'A')
service = MissionService(store, StationCatalog(['S1', 'S2', 'S3']), clock, origin='synthetic')
service.create_mission('SYN-M-PROC', ['S1', 'S2', 'S3'], event_id='p-1')
service.start(event_id='p-2')
service.confirm_arrival('S1', event_id='p-3')
service.record_finding('S1', 'marca-sintetica', event_id='p-4')
service.report_obstacle(ObstacleReport(True, 'OBS-P', 'caja sintetica'), event_id='p-5')
os._exit(0)  # salida abrupta: sin close() ni limpieza
"""


def test_events_survive_a_real_process_restart(tmp_path, catalog):
    db = tmp_path / 'proceso.sqlite3'
    done = subprocess.run([sys.executable, '-B', '-I', '-c', _CHILD, str(db), str(PACKAGE_ROOT)],
                          capture_output=True, text=True, check=False)
    assert done.returncode == 0, done.stderr

    with EvidenceStore(db, 'A') as store:
        assert [e.event_id for e in store.events()] == ['p-1', 'p-2', 'p-3', 'p-4', 'p-5']
        service = MissionService(store, catalog, lambda: TS, origin='synthetic')
        service.recover()
        snapshot = service.snapshot()
        assert snapshot['mission_id'] == 'SYN-M-PROC'
        assert snapshot['state'] == 'BLOCKED'
        assert snapshot['executed_route'] == ['S1']
        assert snapshot['open_obstacles'] == ['OBS-P']
        assert snapshot['findings'] == 1
        # El reintento del último evento tras el reinicio no duplica.
        retry = service.report_obstacle(ObstacleReport(True, 'OBS-P', 'caja sintetica'),
                                        event_id='p-5')
        assert retry.outcome is Outcome.DUPLICATE
        assert len(store.events()) == 5


def test_retries_never_duplicate(make_rig, catalog):
    rig = make_rig()
    service = rig.service
    assert service.create_mission('SYN-M1', ['S1', 'S2'], event_id='r-1').outcome \
        is Outcome.APPLIED
    assert service.create_mission('SYN-M1', ['S1', 'S2'], event_id='r-1').outcome \
        is Outcome.DUPLICATE
    service.start(event_id='r-2')
    assert service.start(event_id='r-2').outcome is Outcome.DUPLICATE
    service.confirm_arrival('S1', event_id='r-3')
    done = service.confirm_arrival('S2', event_id='r-4')
    assert done.state.value == 'COMPLETED'
    # Reintento incluso después de completar la misión (y de reabrir el almacén).
    assert service.confirm_arrival('S2', event_id='r-4').outcome is Outcome.DUPLICATE
    rig.store.close()
    with EvidenceStore(rig.path, 'A') as store:
        again = MissionService(store, catalog, lambda: TS, origin='synthetic')
        again.recover()
        assert again.confirm_arrival('S2', event_id='r-4').outcome is Outcome.DUPLICATE
        assert [e.event_id for e in store.events()] == [
            'r-1', 'r-2', 'r-3', 'r-4', 'r-4.completed']


def test_same_event_id_with_different_content_conflicts(make_rig):
    rig = make_rig()
    service = rig.service
    service.create_mission('SYN-M1', ['S1', 'S2'], event_id='c-1')
    service.start(event_id='c-2')
    service.confirm_arrival('S1', event_id='c-3')
    before = rig.store.events()
    with pytest.raises(EventConflict):
        service.confirm_arrival('S2', event_id='c-3')   # otra estación, mismo id
    with pytest.raises(EventConflict):
        service.pause(event_id='c-2')                    # otro tipo, mismo id
    assert rig.store.events() == before
    assert service.snapshot()['executed_route'] == ['S1']


def test_store_append_is_idempotent_and_atomic(tmp_path):
    with EvidenceStore(tmp_path / 'a.sqlite3', 'A') as store:
        created = _event('e-1')
        assert store.append(created) is True
        assert store.append(created) is False
        started = _event('e-2', EventType.MISSION_STARTED, payload={'origin': 'synthetic'})
        clash = _event('e-1', EventType.MISSION_STARTED, payload={'origin': 'synthetic'})
        with pytest.raises(EventConflict):
            store.append_many([started, clash])   # el segundo falla: no se escribe ninguno
        assert [e.event_id for e in store.events()] == ['e-1']
        with pytest.raises(UnknownMission):
            store.append(_event('e-3', EventType.MISSION_STARTED, mission_id='SYN-OTRA',
                                payload={'origin': 'synthetic'}))


def test_evidence_is_append_only(make_rig):
    rig = make_rig()
    rig.service.create_mission('SYN-M1', ['S1'])
    with closing(sqlite3.connect(rig.path)) as raw:
        with pytest.raises(sqlite3.DatabaseError):
            raw.execute("UPDATE events SET zone = 'S9'")
        with pytest.raises(sqlite3.DatabaseError):
            raw.execute('DELETE FROM events')
    assert len(rig.store.events()) == 1


def test_foreign_or_future_database_is_refused(tmp_path):
    foreign = tmp_path / 'ajena.sqlite3'
    with closing(sqlite3.connect(foreign)) as raw:
        raw.execute('CREATE TABLE otra (x INTEGER)')
    with pytest.raises(StoreError):
        EvidenceStore(foreign, 'A')
    future = tmp_path / 'futura.sqlite3'
    with closing(sqlite3.connect(future)) as raw:
        raw.execute('PRAGMA user_version = 99')
    with pytest.raises(StoreError):
        EvidenceStore(future, 'A')


def test_timestamps_come_from_the_injected_clock(tmp_path, catalog):
    with EvidenceStore(tmp_path / 'naive.sqlite3', 'A') as store:
        service = MissionService(store, catalog, lambda: datetime(2026, 1, 1),
                                 origin='synthetic')
        with pytest.raises(ValidationError):
            service.create_mission('SYN-M1', ['S1'])
        assert store.events() == []


def test_origin_is_mandatory(tmp_path, catalog):
    with EvidenceStore(tmp_path / 'o.sqlite3', 'A') as store:
        with pytest.raises(ValidationError):
            MissionService(store, catalog, lambda: TS, origin='')
