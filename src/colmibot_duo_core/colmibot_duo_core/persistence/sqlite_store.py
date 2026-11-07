"""
Almacén de eventos en SQLite estándar.

Propiedades:
- Solo anexar: triggers impiden UPDATE y DELETE emitidos por SQL ordinario. No protegen
  contra INSERT OR REPLACE, DROP TRIGGER ni la edición directa del archivo: no son una
  garantía forense de inmutabilidad.
- Idempotente: la clave (robot_id, event_id) es única; repetir un evento idéntico no
  duplica y repetirlo con contenido distinto lanza EventConflict.
- Aislado por robot: cada EvidenceStore está atado a un robot_id y solo lee y escribe
  sus propios registros, aunque dos robots compartan el mismo archivo.
- Atómico: append_many escribe todo o nada.
"""

from collections.abc import Iterable
from datetime import datetime
import json
from pathlib import Path
import sqlite3

from colmibot_duo_core.domain.errors import (
    EventConflict, IsolationError, MissionIdInUse, StoreError, UnknownMission)
from colmibot_duo_core.domain.events import Event, EventType, validate_robot_id

SCHEMA_VERSION = 1

_SCHEMA = """
CREATE TABLE events (
    seq        INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id   TEXT NOT NULL CHECK (robot_id IN ('A', 'B')),
    event_id   TEXT NOT NULL,
    mission_id TEXT NOT NULL,
    ts         TEXT NOT NULL,
    event_type TEXT NOT NULL,
    zone       TEXT,
    image_ref  TEXT,
    payload    TEXT NOT NULL,
    UNIQUE (robot_id, event_id)
);
CREATE UNIQUE INDEX ux_mission_created ON events (mission_id)
    WHERE event_type = 'MISSION_CREATED';
CREATE INDEX ix_events_robot_mission ON events (robot_id, mission_id, seq);
CREATE TRIGGER events_no_update BEFORE UPDATE ON events
BEGIN SELECT RAISE(ABORT, 'events es solo-anexar: UPDATE prohibido'); END;
CREATE TRIGGER events_no_delete BEFORE DELETE ON events
BEGIN SELECT RAISE(ABORT, 'events es solo-anexar: DELETE prohibido'); END;
"""

_COLUMNS = 'robot_id, event_id, mission_id, ts, event_type, zone, image_ref, payload'


class EvidenceStore:
    """Evidencia local de un robot (A o B) en un archivo SQLite."""

    def __init__(self, path: str | Path, robot_id: str):
        """Abre o crea el archivo. Usa ':memory:' solo para pruebas efímeras."""
        self._robot_id = validate_robot_id(robot_id)
        self._conn = sqlite3.connect(str(path), isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute('PRAGMA synchronous = FULL')
        self._init_schema()

    @property
    def robot_id(self) -> str:
        """Robot al que pertenece este almacén."""
        return self._robot_id

    def close(self) -> None:
        """Cierra la conexión."""
        self._conn.close()

    def __enter__(self) -> 'EvidenceStore':
        """Permite usar el almacén como gestor de contexto."""
        return self

    def __exit__(self, *exc_info) -> None:
        """Cierra al salir del contexto."""
        self.close()

    def append(self, event: Event) -> bool:
        """Anexa un evento. Devuelve False si ya existía idéntico."""
        return self.append_many([event])[0]

    def append_many(self, events: Iterable[Event]) -> list[bool]:
        """Anexa varios eventos en una sola transacción. Todo o nada."""
        batch = list(events)
        for event in batch:
            if event.robot_id != self._robot_id:
                raise IsolationError(
                    f'este almacén es del robot {self._robot_id}; el evento es de otro robot')
        self._conn.execute('BEGIN IMMEDIATE')
        try:
            inserted = [self._insert(event) for event in batch]
            self._conn.execute('COMMIT')
        except BaseException:
            self._conn.execute('ROLLBACK')
            raise
        return inserted

    def get(self, event_id: str) -> Event | None:
        """Devuelve el evento de este robot con ese event_id, o None."""
        row = self._conn.execute(
            f'SELECT {_COLUMNS} FROM events WHERE robot_id = ? AND event_id = ?',
            (self._robot_id, event_id)).fetchone()
        return None if row is None else _to_event(row)

    def events(self, mission_id: str | None = None) -> list[Event]:
        """Eventos de este robot en orden de escritura, opcionalmente de una misión."""
        sql = f'SELECT {_COLUMNS} FROM events WHERE robot_id = ?'
        args: list = [self._robot_id]
        if mission_id is not None:
            sql += ' AND mission_id = ?'
            args.append(mission_id)
        rows = self._conn.execute(sql + ' ORDER BY seq', args).fetchall()
        return [_to_event(row) for row in rows]

    def mission_ids(self) -> list[str]:
        """Misiones creadas por este robot, de la más antigua a la más reciente."""
        rows = self._conn.execute(
            'SELECT mission_id FROM events WHERE robot_id = ? AND event_type = ? ORDER BY seq',
            (self._robot_id, EventType.MISSION_CREATED.value)).fetchall()
        return [row['mission_id'] for row in rows]

    def _init_schema(self) -> None:
        # La comprobación y la creación van dentro de la misma transacción de escritura:
        # dos procesos que abren a la vez un archivo nuevo no se pisan.
        self._conn.execute('BEGIN IMMEDIATE')
        try:
            version = self._conn.execute('PRAGMA user_version').fetchone()[0]
            if version != SCHEMA_VERSION:
                if version != 0:
                    raise StoreError(f'versión de esquema no soportada: {version}')
                has_tables = self._conn.execute(
                    "SELECT 1 FROM sqlite_master WHERE type = 'table'").fetchone()
                if has_tables:
                    raise StoreError('el archivo contiene tablas ajenas a este esquema')
                for statement in _SCHEMA.split(';\n'):
                    if statement.strip():
                        self._conn.execute(statement.strip())
                self._conn.execute(f'PRAGMA user_version = {SCHEMA_VERSION}')
            self._conn.execute('COMMIT')
        except BaseException:
            self._conn.execute('ROLLBACK')
            raise

    def _insert(self, event: Event) -> bool:
        existing = self.get(event.event_id)
        if existing is not None:
            if existing.content_key() != event.content_key():
                raise EventConflict(
                    f'event_id {event.event_id!r} ya existe con contenido distinto')
            return False
        created = event.event_type is EventType.MISSION_CREATED
        if not created and not self._mission_exists(event.mission_id):
            raise UnknownMission(
                f'la misión {event.mission_id!r} no existe para el robot {self._robot_id}')
        try:
            self._conn.execute(
                f'INSERT INTO events ({_COLUMNS}) VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
                (event.robot_id, event.event_id, event.mission_id, event.ts_iso(),
                 event.event_type.value, event.zone, event.image_ref, event.payload_json()))
        except sqlite3.IntegrityError as exc:
            if created:
                raise MissionIdInUse(f'mission_id {event.mission_id!r} ya está en uso') from exc
            raise
        return True

    def _mission_exists(self, mission_id: str) -> bool:
        row = self._conn.execute(
            'SELECT 1 FROM events WHERE robot_id = ? AND mission_id = ? AND event_type = ?',
            (self._robot_id, mission_id, EventType.MISSION_CREATED.value)).fetchone()
        return row is not None


def _to_event(row: sqlite3.Row) -> Event:
    return Event(
        event_id=row['event_id'],
        robot_id=row['robot_id'],
        mission_id=row['mission_id'],
        ts=datetime.fromisoformat(row['ts']),
        event_type=EventType(row['event_type']),
        zone=row['zone'],
        image_ref=row['image_ref'],
        payload=json.loads(row['payload']))
