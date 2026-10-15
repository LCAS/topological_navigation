"""SQLite persistence module for topological navigation traversal statistics.

This module provides the ``NavStatsDB`` class which handles all database
operations for recording and querying edge traversal statistics, as well as
storing the topological maps themselves.  It is designed to be used by the
navigation server and is intentionally kept independent of any ROS 2 imports
so that it can also be used from standalone analysis scripts.

Schema
------
``topological_maps`` table stores one row per unique map version:

    id              – auto-increment primary key
    map_name        – human-readable map name (from the ``name`` top-level key)
    map_hash        – SHA-1 of the serialised map YAML (UNIQUE version key)
    latitude        – origin latitude extracted from ``meta.origin.latitude``
    longitude       – origin longitude extracted from ``meta.origin.longitude``
    map_data        – full YAML text stored as a BLOB for later retrieval
    added_at        – UTC timestamp of when this map version was first stored

``traversals`` table stores one row per traversal attempt:

    id              – auto-increment primary key
    map_name        – topological map pointset name
    map_hash        – SHA-1 of the serialised map YAML (version fingerprint)
    edge_id         – edge identifier (e.g. ``nodeA_nodeB``)
    origin          – source node name
    target          – destination node name
    status          – ``'success'``, ``'failed'``, or ``'aborted'``
    failure_reason  – ``'none'``, ``'cancelled'``, ``'lookup_failed'``, etc.
    start_time      – ISO-8601 UTC timestamp
    end_time        – ISO-8601 UTC timestamp
    duration_s      – traversal duration in seconds
    avg_speed       – edge_length / duration_s (m/s); NULL if unknown
    is_segment      – 1 if this traversal was part of a multi-edge segment
    segment_edges   – JSON array of all edge IDs in the segment (or NULL)
    created_at      – row insertion timestamp (UTC)
"""

import hashlib
import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------

_CREATE_TOPOLOGICAL_MAPS = """
CREATE TABLE IF NOT EXISTS topological_maps (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    map_name   TEXT    NOT NULL DEFAULT '',
    map_hash   TEXT    NOT NULL UNIQUE,
    latitude   REAL,
    longitude  REAL,
    map_data   BLOB    NOT NULL,
    added_at   TEXT    NOT NULL DEFAULT (datetime('now'))
);
"""

_CREATE_IDX_TOPOMAP_HASH = """
CREATE UNIQUE INDEX IF NOT EXISTS idx_topomaps_map_hash
    ON topological_maps (map_hash);
"""

_CREATE_IDX_TOPOMAP_NAME = """
CREATE INDEX IF NOT EXISTS idx_topomaps_map_name
    ON topological_maps (map_name);
"""

_INSERT_TOPOMAP = """
INSERT OR IGNORE INTO topological_maps
    (map_name, map_hash, latitude, longitude, map_data)
VALUES
    (:map_name, :map_hash, :latitude, :longitude, :map_data);
"""

_CREATE_TRAVERSALS = """
CREATE TABLE IF NOT EXISTS traversals (
    id             INTEGER PRIMARY KEY AUTOINCREMENT,
    map_name       TEXT    NOT NULL DEFAULT '',
    map_hash       TEXT    NOT NULL DEFAULT '',
    edge_id        TEXT    NOT NULL DEFAULT '',
    origin         TEXT    NOT NULL DEFAULT '',
    target         TEXT    NOT NULL DEFAULT '',
    status         TEXT    NOT NULL DEFAULT 'unknown',
    failure_reason TEXT    NOT NULL DEFAULT 'none',
    start_time     TEXT,
    end_time       TEXT,
    duration_s     REAL,
    avg_speed      REAL,
    is_segment     INTEGER NOT NULL DEFAULT 0,
    segment_edges  TEXT,
    created_at     TEXT    NOT NULL DEFAULT (datetime('now'))
);
"""

_CREATE_IDX_EDGE = """
CREATE INDEX IF NOT EXISTS idx_traversals_edge_id
    ON traversals (edge_id);
"""

_CREATE_IDX_MAP = """
CREATE INDEX IF NOT EXISTS idx_traversals_map_name
    ON traversals (map_name);
"""

_INSERT_TRAVERSAL = """
INSERT INTO traversals (
    map_name, map_hash, edge_id, origin, target,
    status, failure_reason,
    start_time, end_time, duration_s, avg_speed,
    is_segment, segment_edges
) VALUES (
    :map_name, :map_hash, :edge_id, :origin, :target,
    :status, :failure_reason,
    :start_time, :end_time, :duration_s, :avg_speed,
    :is_segment, :segment_edges
);
"""


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def compute_map_hash(map_yaml_str: str) -> str:
    """Return a short SHA-1 hex digest of the serialised map YAML string."""
    return hashlib.sha1(map_yaml_str.encode("utf-8", errors="replace")).hexdigest()[:12]


def _iso(dt: Optional[datetime]) -> Optional[str]:
    """Convert *dt* to an ISO-8601 UTC string, or return ``None``."""
    if dt is None:
        return None
    if dt.tzinfo is None:
        # Assume local time; store as-is with 'Z' suffix omitted
        return dt.isoformat(timespec="seconds")
    return dt.astimezone(timezone.utc).isoformat(timespec="seconds")


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class NavStatsDB:
    """Thin wrapper around an SQLite database for navigation statistics.

    Parameters
    ----------
    db_path:
        Path to the SQLite file.  The file (and any parent directories)
        are created automatically if they do not exist.
    """

    def __init__(self, db_path: str) -> None:
        path = Path(db_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = str(path)
        self._conn: Optional[sqlite3.Connection] = None
        self._open()
        self._create_schema()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _open(self) -> None:
        """Open (or re-open) the SQLite connection with WAL mode."""
        self._conn = sqlite3.connect(
            self._path,
            check_same_thread=False,
            timeout=10.0,
        )
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute("PRAGMA foreign_keys=ON;")

    def _create_schema(self) -> None:
        """Create tables and indexes if they do not yet exist."""
        with self._conn:
            self._conn.execute(_CREATE_TOPOLOGICAL_MAPS)
            self._conn.execute(_CREATE_IDX_TOPOMAP_HASH)
            self._conn.execute(_CREATE_IDX_TOPOMAP_NAME)
            self._conn.execute(_CREATE_TRAVERSALS)
            self._conn.execute(_CREATE_IDX_EDGE)
            self._conn.execute(_CREATE_IDX_MAP)

    def close(self) -> None:
        """Close the database connection."""
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    # ------------------------------------------------------------------
    # Map store/retrieve API
    # ------------------------------------------------------------------

    def store_map(self, map_yaml_str: str) -> str:
        """Persist a topological map to the database (idempotent).

        If a map with the same hash already exists, this is a no-op.

        Parameters
        ----------
        map_yaml_str:
            Raw YAML text of the topological map as received from the ROS
            topic or read from a file.

        Returns
        -------
        str
            The SHA-1 hash of the stored map (12 hex characters).
        """
        map_hash = compute_map_hash(map_yaml_str)
        try:
            parsed: Dict[str, Any] = yaml.safe_load(map_yaml_str) or {}
        except Exception:
            parsed = {}

        map_name = parsed.get("name") or parsed.get("pointset") or ""
        origin = (parsed.get("meta") or {}).get("origin") or {}
        latitude = origin.get("latitude")
        longitude = origin.get("longitude")

        row = {
            "map_name": map_name,
            "map_hash": map_hash,
            "latitude": latitude,
            "longitude": longitude,
            "map_data": map_yaml_str,
        }
        with self._conn:
            self._conn.execute(_INSERT_TOPOMAP, row)
        return map_hash

    def get_map(self, identifier: str) -> Optional[Dict[str, Any]]:
        """Retrieve a stored map by name or hash.

        When *identifier* matches multiple names, the most-recently-added
        row is returned.

        Parameters
        ----------
        identifier:
            Either the full ``map_hash`` (12-char hex) or the ``map_name``.

        Returns
        -------
        dict or None
            Row dict with keys: id, map_name, map_hash, latitude, longitude,
            map_data, added_at.  Returns ``None`` if not found.
        """
        rows = self.query(
            "SELECT * FROM topological_maps WHERE map_hash = ? "
            "ORDER BY added_at DESC LIMIT 1",
            (identifier,),
        )
        if rows:
            return rows[0]
        rows = self.query(
            "SELECT * FROM topological_maps WHERE map_name = ? "
            "ORDER BY added_at DESC LIMIT 1",
            (identifier,),
        )
        return rows[0] if rows else None

    def list_maps(self) -> List[Dict[str, Any]]:
        """Return all stored maps ordered by name then added_at."""
        return self.query(
            "SELECT id, map_name, map_hash, latitude, longitude, added_at "
            "FROM topological_maps ORDER BY map_name, added_at",
        )

    def delete_map(self, identifier: str) -> int:
        """Delete a stored map by name or hash.

        Parameters
        ----------
        identifier:
            Either the full ``map_hash`` or the ``map_name``.

        Returns
        -------
        int
            Number of rows deleted.
        """
        with self._conn:
            cur = self._conn.execute(
                "DELETE FROM topological_maps WHERE map_hash = ?",
                (identifier,),
            )
            if cur.rowcount:
                return cur.rowcount
            cur = self._conn.execute(
                "DELETE FROM topological_maps WHERE map_name = ?",
                (identifier,),
            )
            return cur.rowcount

    # ------------------------------------------------------------------
    # Write API
    # ------------------------------------------------------------------

    def record_traversal(
        self,
        *,
        map_name: str,
        map_hash: str,
        edge_id: str,
        origin: str,
        target: str,
        status: str,
        failure_reason: str = "none",
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        duration_s: Optional[float] = None,
        edge_length: Optional[float] = None,
        is_segment: bool = False,
        segment_edges: Optional[List[str]] = None,
    ) -> int:
        """Insert one traversal record and return the new row id.

        Parameters
        ----------
        map_name:      Topological map pointset name.
        map_hash:      Short hash of the map YAML for version tracking.
        edge_id:       Edge identifier (``'nodeA_nodeB'``).
        origin:        Source node name.
        target:        Destination node name.
        status:        ``'success'``, ``'failed'``, or ``'aborted'``.
        failure_reason: Human-readable failure category.
        start_time:    Traversal start (``datetime`` object).
        end_time:      Traversal end (``datetime`` object).
        duration_s:    Duration in seconds (derived from timestamps if omitted).
        edge_length:   Euclidean distance between nodes (metres).  Used to
                       compute ``avg_speed = edge_length / duration_s``.
        is_segment:    Whether this traversal was part of a multi-edge segment.
        segment_edges: All edge IDs belonging to the enclosing segment.
        """
        # Derive duration from timestamps if not provided
        if duration_s is None and start_time is not None and end_time is not None:
            duration_s = (end_time - start_time).total_seconds()

        # Average speed (m/s)
        avg_speed: Optional[float] = None
        if (
            edge_length is not None
            and duration_s is not None
            and duration_s > 0.0
        ):
            avg_speed = edge_length / duration_s

        row = {
            "map_name": map_name,
            "map_hash": map_hash,
            "edge_id": edge_id,
            "origin": origin,
            "target": target,
            "status": status,
            "failure_reason": failure_reason,
            "start_time": _iso(start_time),
            "end_time": _iso(end_time),
            "duration_s": duration_s,
            "avg_speed": avg_speed,
            "is_segment": 1 if is_segment else 0,
            "segment_edges": json.dumps(segment_edges) if segment_edges else None,
        }

        with self._conn:
            cur = self._conn.execute(_INSERT_TRAVERSAL, row)
        return cur.lastrowid

    # ------------------------------------------------------------------
    # Read API (used by the standalone stats script)
    # ------------------------------------------------------------------

    def query(self, sql: str, params=()):
        """Execute an arbitrary SELECT and return all rows as dicts."""
        cur = self._conn.execute(sql, params)
        cols = [d[0] for d in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    def edge_ids(self, where: str = "") -> List[str]:
        """Return a sorted list of distinct edge IDs, optionally filtered."""
        sql = "SELECT DISTINCT edge_id FROM traversals"
        if where:
            sql += " WHERE " + where
        sql += " ORDER BY edge_id"
        return [r["edge_id"] for r in self.query(sql)]

    def merge_from(self, other_db_path: str) -> Dict[str, int]:
        """Merge all maps and traversals from *other_db_path* into this database.

        Each run of the navigation system typically creates its own
        database file; this method allows several such databases to be
        combined into one so that coverage can be analysed across runs.

        Topological maps are merged idempotently: a map already present
        (matched by ``map_hash``) is skipped. Traversal records are always
        copied across (they represent distinct historical events), each
        getting a new row id in the destination database.

        Parameters
        ----------
        other_db_path:
            Path to the source SQLite database to merge from. It is opened
            read-only and left untouched.

        Returns
        -------
        dict
            ``{"maps_added": int, "maps_skipped": int, "traversals_added": int}``
        """
        other = NavStatsDB(other_db_path)
        try:
            maps_added = 0
            maps_skipped = 0
            for m in other.list_maps():
                full = other.get_map(m["map_hash"])
                if full is None:
                    continue
                existing = self.query(
                    "SELECT id FROM topological_maps WHERE map_hash = ?",
                    (full["map_hash"],),
                )
                if existing:
                    maps_skipped += 1
                    continue
                with self._conn:
                    self._conn.execute(_INSERT_TOPOMAP, {
                        "map_name": full["map_name"],
                        "map_hash": full["map_hash"],
                        "latitude": full.get("latitude"),
                        "longitude": full.get("longitude"),
                        "map_data": full["map_data"],
                    })
                maps_added += 1

            traversals = other.query("SELECT * FROM traversals")
            traversals_added = 0
            for t in traversals:
                row = {
                    "map_name": t.get("map_name") or "",
                    "map_hash": t.get("map_hash") or "",
                    "edge_id": t.get("edge_id") or "",
                    "origin": t.get("origin") or "",
                    "target": t.get("target") or "",
                    "status": t.get("status") or "unknown",
                    "failure_reason": t.get("failure_reason") or "none",
                    "start_time": t.get("start_time"),
                    "end_time": t.get("end_time"),
                    "duration_s": t.get("duration_s"),
                    "avg_speed": t.get("avg_speed"),
                    "is_segment": t.get("is_segment") or 0,
                    "segment_edges": t.get("segment_edges"),
                }
                with self._conn:
                    self._conn.execute(_INSERT_TRAVERSAL, row)
                traversals_added += 1
        finally:
            other.close()

        return {
            "maps_added": maps_added,
            "maps_skipped": maps_skipped,
            "traversals_added": traversals_added,
        }

    def edge_stats(self, edge_id: str, where: str = "") -> dict:
        """Aggregate statistics for *edge_id*.

        Returns a dict with keys:
            edge_id, total, success, failed, aborted,
            avg_duration_s, min_duration_s, max_duration_s,
            avg_speed, last_traversal
        """
        base = "edge_id = :eid"
        if where:
            base += " AND (" + where + ")"

        sql = """
            SELECT
                edge_id,
                COUNT(*)                                    AS total,
                SUM(CASE WHEN status='success' THEN 1 ELSE 0 END) AS success,
                SUM(CASE WHEN status='failed'  THEN 1 ELSE 0 END) AS failed,
                SUM(CASE WHEN status='aborted' THEN 1 ELSE 0 END) AS aborted,
                AVG(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                            AS avg_duration_s,
                MIN(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                            AS min_duration_s,
                MAX(CASE WHEN duration_s IS NOT NULL THEN duration_s END)
                                                            AS max_duration_s,
                AVG(avg_speed)                              AS avg_speed,
                MAX(end_time)                               AS last_traversal
            FROM traversals
            WHERE """ + base + """
            GROUP BY edge_id
        """
        rows = self.query(sql, {"eid": edge_id})
        return rows[0] if rows else {}
