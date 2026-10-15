#!/usr/bin/env python3
"""Merge multiple ``.tmap2.yaml`` topological maps into a single map.

Node positions are reprojected into the first ("reference") map's GPS
``meta.origin`` frame, assuming every map's local x/y Cartesian frame is
already East/North aligned (ENU, no heading correction). Node and edge
names are kept globally unique, suffixing later duplicates with a running
number and warning about the collision. Top-level metadata (``meta``,
``name``, ``metric_map``, ``pointset``, ``transformation``, ``definitions``,
``actions``) is merged with a "first map to define a value wins" rule,
with special handling for ``meta.origin`` (always the reference map's
value) and ``meta.fields`` (concatenated, renumbered so nothing is lost).
Optionally (``--connect-closest``), each map is linked to the merged map
with a new bidirectional edge between their closest pair of nodes, so the
result is a single connected graph instead of one sub-map per input.

Usage::

    python3 map_merger.py map_a.tmap2.yaml map_b.tmap2.yaml -o merged.tmap2.yaml
"""

import argparse
import datetime
import logging
import math
import os
import sys
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required. Install with: pip install pyyaml")
    sys.exit(2)

try:
    from pyproj import Transformer
except ImportError:
    print("Error: pyproj is required. Install with: pip install pyproj")
    sys.exit(2)

from topological_navigation.tmap_utils import NoAliasDumper, load_tmap2_file
from topological_navigation.validate_map import validate_map

_LOGGER = logging.getLogger(__name__)

_TIMESTAMP_FORMAT = "%d-%m-%Y_%H-%M-%S"


def _get_time() -> str:
    """Current time in the format used elsewhere for ``meta.last_updated``."""
    return datetime.datetime.now().strftime(_TIMESTAMP_FORMAT)


def _is_origin_set(origin: Optional[Dict[str, Any]]) -> bool:
    """False if *origin* is missing, or its lat/long are both exactly 0."""
    if not origin:
        return False
    return bool(origin.get("latitude", 0.0)) or bool(origin.get("longitude", 0.0))


def _local_offset_metres(ref_lat: float, ref_lon: float, lat: float, lon: float) -> Tuple[float, float]:
    """(east, north) metre offset of *(lat, lon)* from *(ref_lat, ref_lon)*.

    Uses an azimuthal-equidistant projection centred on the reference point,
    which gives ENU-aligned metre offsets directly without any manual
    trigonometry or UTM-zone handling. Only valid for the ENU/no-rotation
    assumption documented above.
    """
    aeqd_crs = f"+proj=aeqd +lat_0={ref_lat} +lon_0={ref_lon} +datum=WGS84 +units=m +no_defs"
    transformer = Transformer.from_crs("EPSG:4326", aeqd_crs, always_xy=True)
    east, north = transformer.transform(lon, lat)
    return east, north


def _next_unique_name(base: str, used: set) -> str:
    """*base* if unused, else ``base_2``, ``base_3``, ... until unused."""
    if base not in used:
        return base
    n = 2
    while f"{base}_{n}" in used:
        n += 1
    return f"{base}_{n}"


@dataclass
class MergeResult:
    """Outcome of running :func:`merge_maps`."""

    input_files: List[str]
    output_file: str
    num_maps: int
    total_nodes: int = 0
    total_edges: int = 0
    node_renames: List[Tuple[str, str, str]] = field(default_factory=list)
    edge_renames: List[Tuple[str, str, str]] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    reference_origin: Dict[str, Any] = field(default_factory=dict)
    per_map_offsets: List[Tuple[str, float, float, float, bool]] = field(default_factory=list)
    fields_merged: int = 0
    connecting_edges: List[Tuple[str, str, str, str, float]] = field(default_factory=list)
    schema_valid: Optional[bool] = None
    schema_message: str = ""

    def format_report(self) -> str:
        """Return a human-readable merge report."""
        lines = [f"Merge report: {', '.join(self.input_files)} -> {self.output_file}", "=" * 60]
        lines.append(f"  Maps merged:  {self.num_maps}")
        lines.append(f"  Total nodes:  {self.total_nodes}")
        lines.append(f"  Total edges:  {self.total_edges}")
        lines.append(f"  Fields merged: {self.fields_merged}")

        lines.append("")
        lines.append("[Reference origin]")
        if self.reference_origin:
            lines.append(f"  {self.reference_origin}")
        else:
            lines.append("  none")

        lines.append("")
        lines.append("[Per-map reprojection offsets]")
        for map_file, east, north, alt, reprojected in self.per_map_offsets:
            status = "reprojected" if reprojected else "unchanged (no GPS origin)"
            lines.append(f"  - {map_file}: east={east:.3f} north={north:.3f} alt={alt:.3f} ({status})")

        lines.append("")
        lines.append("[Node renames]")
        if self.node_renames:
            for source_file, old, new in self.node_renames:
                lines.append(f"  - {source_file}: '{old}' -> '{new}'")
        else:
            lines.append("  none")

        lines.append("")
        lines.append("[Edge renames]")
        if self.edge_renames:
            for source_file, old, new in self.edge_renames:
                lines.append(f"  - {source_file}: '{old}' -> '{new}'")
        else:
            lines.append("  none")

        lines.append("")
        lines.append("[Connecting edges]")
        if self.connecting_edges:
            for node_a, node_b, edge_id_ab, edge_id_ba, dist in self.connecting_edges:
                lines.append(
                    f"  - '{node_a}' <-> '{node_b}' (dist={dist:.3f}m): '{edge_id_ab}' / '{edge_id_ba}'"
                )
        else:
            lines.append("  none")

        lines.append("")
        lines.append("[Warnings]")
        if self.warnings:
            for msg in self.warnings:
                lines.append(f"  - {msg}")
        else:
            lines.append("  none")

        if self.schema_valid is not None:
            lines.append("")
            lines.append("[Schema validation of merged output]")
            status = "PASS" if self.schema_valid else "FAIL"
            lines.append(f"  {status}: {self.schema_message}")

        return "\n".join(lines)


def _derive_output_path(map_files: List[str]) -> str:
    """Derive a sibling ``<name>.merged.<ext>`` output path from the first input."""
    directory, base = os.path.split(map_files[0])
    stem, sep, rest = base.partition(".")
    derived = f"{stem}.merged{sep}{rest}" if sep else f"{stem}.merged"
    return os.path.join(directory, derived) if directory else derived


def _merge_fields(metas: List[Dict[str, Any]], warn) -> Tuple[List[Dict[str, Any]], bool]:
    """Concatenate every map's ``meta.fields``, renumbering ``field_number`` to stay unique."""
    merged_fields: List[Dict[str, Any]] = []
    seen_numbers = set()
    renumbered = False
    for meta in metas:
        for entry in meta.get("fields", []) or []:
            entry = deepcopy(entry)
            number = entry.get("field_number")
            if number is None or number in seen_numbers:
                new_number = 1
                while new_number in seen_numbers:
                    new_number += 1
                if number is not None:
                    renumbered = True
                entry["field_number"] = new_number
                number = new_number
            seen_numbers.add(number)
            merged_fields.append(entry)
    if renumbered:
        warn("meta.fields contained duplicate field_number values; renumbered to stay unique")
    return merged_fields, renumbered


def _merge_meta(metas: List[Dict[str, Any]], warn) -> Dict[str, Any]:
    """Merge ``meta`` blocks: first-definer-wins per key, with special-cased subkeys."""
    merged: Dict[str, Any] = {}
    for meta in metas:
        for key, value in meta.items():
            if key in ("origin", "fields", "last_updated"):
                continue
            if key not in merged:
                merged[key] = deepcopy(value)
            elif merged[key] != value:
                warn(f"meta.{key} differs between maps; keeping the first-defined value")

    merged["origin"] = deepcopy(metas[0].get("origin", {}))
    fields, _ = _merge_fields(metas, warn)
    if fields:
        merged["fields"] = fields
    merged["last_updated"] = _get_time()
    return merged


def _merge_top_level_scalars(docs: List[Dict[str, Any]], warn) -> Dict[str, Any]:
    """Merge every top-level key except ``nodes``/``meta``: first-definer-wins, warn on conflict."""
    merged: Dict[str, Any] = {}
    for doc in docs:
        for key, value in doc.items():
            if key in ("nodes", "meta"):
                continue
            if key not in merged:
                merged[key] = deepcopy(value)
            elif merged[key] != value:
                warn(f"Top-level '{key}' differs between maps; keeping the first-defined value")
    return merged


def _rename_nodes_and_edges(
    doc: Dict[str, Any],
    source_file: str,
    used_node_names: set,
    used_edge_ids: set,
    merged_name: str,
    merged_pointset: str,
    node_renames: List[Tuple[str, str, str]],
    edge_renames: List[Tuple[str, str, str]],
    warn,
) -> List[Dict[str, Any]]:
    """Dedup node names/edge_ids within *doc*, rewriting same-map edge targets.

    Returns the (deep-copied, renamed, re-parented) list of node entries.
    """
    entries = deepcopy(doc.get("nodes", []))

    # Pass 1: assign unique node names.
    local_rename: Dict[str, str] = {}
    for entry in entries:
        node = entry["node"]
        original_name = node["name"]
        new_name = _next_unique_name(original_name, used_node_names)
        if new_name != original_name:
            warn(f"Node name '{original_name}' from {source_file} collides; renamed to '{new_name}'")
            node_renames.append((source_file, original_name, new_name))
        used_node_names.add(new_name)
        local_rename[original_name] = new_name

        node["name"] = new_name
        entry.setdefault("meta", {})["node"] = new_name
        entry["meta"]["map"] = merged_name
        entry["meta"]["pointset"] = merged_pointset

    # Pass 2: rewrite edge targets (same source map only) and dedup edge_ids.
    for entry in entries:
        for edge in entry["node"].get("edges", []):
            target = edge.get("node")
            if target in local_rename:
                edge["node"] = local_rename[target]

            original_edge_id = edge.get("edge_id")
            if original_edge_id is None:
                continue
            new_edge_id = _next_unique_name(original_edge_id, used_edge_ids)
            if new_edge_id != original_edge_id:
                warn(
                    f"Edge id '{original_edge_id}' from {source_file} collides; "
                    f"renamed to '{new_edge_id}'"
                )
                edge_renames.append((source_file, original_edge_id, new_edge_id))
            used_edge_ids.add(new_edge_id)
            edge["edge_id"] = new_edge_id

    return entries


def _apply_offset(entries: List[Dict[str, Any]], east: float, north: float, alt: float) -> None:
    """Shift every node's ``pose.position`` by *(east, north, alt)`` metres."""
    if east == 0.0 and north == 0.0 and alt == 0.0:
        return
    for entry in entries:
        position = entry["node"].get("pose", {}).get("position")
        if not position:
            continue
        position["x"] = position.get("x", 0.0) + east
        position["y"] = position.get("y", 0.0) + north
        position["z"] = position.get("z", 0.0) + alt


def _node_position(entry: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    """Return *entry*'s ``(x, y, z)`` position, or None if it has no pose."""
    position = entry.get("node", {}).get("pose", {}).get("position")
    if not position:
        return None
    return (
        float(position.get("x", 0.0)),
        float(position.get("y", 0.0)),
        float(position.get("z", 0.0)),
    )


def _closest_node_pair(
    entries_a: List[Dict[str, Any]], entries_b: List[Dict[str, Any]]
) -> Optional[Tuple[str, str, float]]:
    """Return ``(name_a, name_b, distance)`` for the closest pair of nodes across the two lists."""
    best: Optional[Tuple[str, str, float]] = None
    for entry_a in entries_a:
        pos_a = _node_position(entry_a)
        if pos_a is None:
            continue
        for entry_b in entries_b:
            pos_b = _node_position(entry_b)
            if pos_b is None:
                continue
            dist = math.sqrt(sum((ca - cb) ** 2 for ca, cb in zip(pos_a, pos_b)))
            if best is None or dist < best[2]:
                best = (entry_a["node"]["name"], entry_b["node"]["name"], dist)
    return best


def _append_edge(
    entries: List[Dict[str, Any]], from_name: str, to_name: str, edge_id: str, action: str, action_type: str
) -> None:
    """Append a new edge from the node named *from_name* to *to_name* within *entries*."""
    for entry in entries:
        if entry["node"]["name"] == from_name:
            entry["node"].setdefault("edges", []).append(
                {"edge_id": edge_id, "node": to_name, "action": action, "action_type": action_type}
            )
            return
    raise KeyError(f"Node '{from_name}' not found when adding a connecting edge")


def merge_maps(
    map_files: List[str],
    output_file: Optional[str] = None,
    schema_file: Optional[str] = None,
    name: Optional[str] = None,
    connect_closest: bool = False,
    connect_action: str = "navigate_to_pose",
    connect_action_type: str = "nav2_msgs/action/NavigateToPose",
    logger: Optional[logging.Logger] = None,
) -> MergeResult:
    """Merge *map_files* into a single schema-valid map and return a :class:`MergeResult`.

    Positions of every map after the first are reprojected into the first
    map's ``meta.origin`` GPS frame (ENU, no rotation). Node names and edge
    ids are kept globally unique by suffixing collisions with a running
    number. Top-level metadata is merged with a first-map-wins policy (see
    :func:`_merge_meta` / :func:`_merge_top_level_scalars` for the exceptions).

    If *connect_closest* is set, each map (after the first) is linked to the
    growing set of already-merged maps with a new bidirectional edge between
    their closest pair of nodes, so the final graph is a single connected
    component rather than one disconnected sub-map per input file.
    """
    log = logger or _LOGGER

    if len(map_files) < 2:
        raise ValueError("At least two map files are required to merge")

    invalid = []
    for map_file in map_files:
        is_valid, message = validate_map(map_file, schema_file)
        if not is_valid:
            invalid.append(f"{map_file}: {message}")
    if invalid:
        raise ValueError("Cannot merge invalid input map(s):\n" + "\n".join(invalid))

    docs = [load_tmap2_file(map_file) for map_file in map_files]

    warnings: List[str] = []

    def warn(message: str) -> None:
        warnings.append(message)
        log.warning(message)

    merged_doc = _merge_top_level_scalars(docs, warn)
    merged_doc["meta"] = _merge_meta([doc.get("meta", {}) for doc in docs], warn)

    if name:
        merged_doc["name"] = name
        merged_doc["metric_map"] = name
        merged_doc["pointset"] = name

    merged_name = merged_doc.get("name", "")
    merged_pointset = merged_doc.get("pointset", "")

    reference_origin = docs[0].get("meta", {}).get("origin", {})
    reference_set = _is_origin_set(reference_origin)
    ref_lat = float(reference_origin.get("latitude", 0.0)) if reference_origin else 0.0
    ref_lon = float(reference_origin.get("longitude", 0.0)) if reference_origin else 0.0
    ref_alt = float(reference_origin.get("altitude", 0.0)) if reference_origin else 0.0

    if not reference_set:
        warn(
            f"Reference map {map_files[0]} has no real GPS origin (meta.origin); "
            "no reprojection will be applied to any input, positions are merged unchanged"
        )

    used_node_names: set = set()
    used_edge_ids: set = set()
    per_map_offsets: List[Tuple[str, float, float, float, bool]] = []
    all_entries: List[Dict[str, Any]] = []
    node_renames: List[Tuple[str, str, str]] = []
    edge_renames: List[Tuple[str, str, str]] = []

    for map_file, doc in zip(map_files, docs):
        origin = doc.get("meta", {}).get("origin", {})
        east = north = alt = 0.0
        reprojected = False
        if reference_set and origin is not reference_origin:
            if _is_origin_set(origin):
                lat = float(origin.get("latitude", 0.0))
                lon = float(origin.get("longitude", 0.0))
                alt_val = float(origin.get("altitude", 0.0))
                east, north = _local_offset_metres(ref_lat, ref_lon, lat, lon)
                alt = alt_val - ref_alt
                reprojected = True
            else:
                warn(
                    f"Map {map_file} has no real GPS origin (meta.origin); assuming it shares "
                    "the reference map's local frame, no position offset applied"
                )
        per_map_offsets.append((map_file, east, north, alt, reprojected))

        entries = _rename_nodes_and_edges(
            doc, map_file, used_node_names, used_edge_ids,
            merged_name, merged_pointset,
            node_renames, edge_renames,
            warn,
        )
        _apply_offset(entries, east, north, alt)
        all_entries.append(entries)

    connecting_edges: List[Tuple[str, str, str, str, float]] = []
    if connect_closest:
        connected_pool: List[Dict[str, Any]] = list(all_entries[0])
        for map_file, entries in zip(map_files[1:], all_entries[1:]):
            pair = _closest_node_pair(connected_pool, entries)
            if pair is None:
                warn(f"Map {map_file} has no nodes with a valid pose; could not connect it to the merged map")
            else:
                node_a, node_b, dist = pair
                edge_id_ab = _next_unique_name(f"connect_{node_a}_{node_b}", used_edge_ids)
                used_edge_ids.add(edge_id_ab)
                edge_id_ba = _next_unique_name(f"connect_{node_b}_{node_a}", used_edge_ids)
                used_edge_ids.add(edge_id_ba)
                _append_edge(connected_pool, node_a, node_b, edge_id_ab, connect_action, connect_action_type)
                _append_edge(entries, node_b, node_a, edge_id_ba, connect_action, connect_action_type)
                connecting_edges.append((node_a, node_b, edge_id_ab, edge_id_ba, dist))
            connected_pool = connected_pool + entries

    merged_doc["nodes"] = [entry for entries in all_entries for entry in entries]

    total_nodes = len(merged_doc["nodes"])
    total_edges = sum(len(entry["node"].get("edges", [])) for entry in merged_doc["nodes"])
    fields_merged = len(merged_doc["meta"].get("fields", []))

    out_path = output_file or _derive_output_path(map_files)
    directory = os.path.dirname(os.path.abspath(out_path))
    if directory:
        os.makedirs(directory, exist_ok=True)

    header = (
        f"# Merged from: {', '.join(map_files)}\n"
        f"# Generated: {_get_time()}\n"
    )
    dumped = yaml.dump(
        merged_doc, Dumper=NoAliasDumper, default_flow_style=False, sort_keys=False,
        allow_unicode=True,
    )
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(header + dumped)

    schema_valid, schema_message = validate_map(out_path, schema_file)

    result = MergeResult(
        input_files=list(map_files),
        output_file=out_path,
        num_maps=len(map_files),
        total_nodes=total_nodes,
        total_edges=total_edges,
        node_renames=node_renames,
        edge_renames=edge_renames,
        warnings=warnings,
        reference_origin=reference_origin if reference_set else {},
        per_map_offsets=per_map_offsets,
        fields_merged=fields_merged,
        connecting_edges=connecting_edges,
        schema_valid=schema_valid,
        schema_message=schema_message,
    )
    log.info(
        "Merged %d map(s) -> %s: %d node(s), %d edge(s)",
        len(map_files), out_path, total_nodes, total_edges,
    )
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Merge multiple topological map (.tmap2.yaml) files into one.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s map_a.tmap2.yaml map_b.tmap2.yaml -o merged.tmap2.yaml
  %(prog)s map_a.tmap2.yaml map_b.tmap2.yaml map_c.tmap2.yaml --name my_merged_map
  %(prog)s map_a.tmap2.yaml map_b.tmap2.yaml --connect-closest
        """,
    )
    parser.add_argument("map_files", nargs="+", help="Paths to two or more topological map YAML files")
    parser.add_argument("--output", "-o", help="Output file path (default: <first-map-name>.merged.<ext>)")
    parser.add_argument("--schema", "-s", help="Path to the schema YAML file (optional)")
    parser.add_argument("--name", help="Override the merged map's name/metric_map/pointset")
    parser.add_argument(
        "--connect-closest", action="store_true",
        help="Link each map to the merged map with a bidirectional edge between their closest nodes",
    )
    parser.add_argument(
        "--connect-action", default="navigate_to_pose",
        help="Action name used for connecting edges (default: %(default)s)",
    )
    parser.add_argument(
        "--connect-action-type", default="nav2_msgs/action/NavigateToPose",
        help="Action type used for connecting edges (default: %(default)s)",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if len(args.map_files) < 2:
        print("Error: At least two map files are required to merge")
        sys.exit(2)

    for map_file in args.map_files:
        if not os.path.isfile(map_file):
            print(f"Error: Map file not found: {map_file}")
            sys.exit(2)

    try:
        result = merge_maps(
            args.map_files, output_file=args.output, schema_file=args.schema, name=args.name,
            connect_closest=args.connect_closest, connect_action=args.connect_action,
            connect_action_type=args.connect_action_type,
        )
    except Exception as exc:  # noqa: BLE001 - report any load/merge error to the user
        print(f"Error merging maps: {exc}")
        sys.exit(2)

    print(result.format_report())
    sys.exit(0 if result.schema_valid else 1)


if __name__ == "__main__":
    main()
