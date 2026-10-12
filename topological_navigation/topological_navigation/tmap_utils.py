#!/usr/bin/env python3
"""Topological map utility functions.

Provides helper functions for YAML loading, split-map handling, and node/edge
access in the tmap2 format.
"""
from copy import deepcopy
import os

import yaml

# ===== YAML Loader =====


class CustomSafeLoader(yaml.SafeLoader):
    """Custom YAML loader that ensures poses and translations are float-type.

    ROS 2 messages (Vector3, Pose, etc.) have assertions for float-type
    [x, y, z, w] keys.
    """

    def construct_mapping(self, node, deep=False):
        """Construct mapping, converting int to float for pose keys."""
        mapping = super().construct_mapping(node, deep=deep)

        # Convert int to float for pose/vector keys
        for key in ['x', 'y', 'z', 'w', 'yaw_goal_tolerance', 'xy_goal_tolerance']:
            if key in mapping and isinstance(mapping[key], int):
                mapping[key] = float(mapping[key])

        return mapping


# ========================


class NoAliasDumper(yaml.SafeDumper):
    """YAML dumper that disables aliases/anchors for cleaner output."""

    def ignore_aliases(self, data):
        """Return True for all data to disable aliases."""
        return True


# ========================

NAVIGATION_CONFIG_FILE_KEY = "navigation_config_file"
DEFAULT_NAVIGATION_CONFIG_FILENAME = "topological_navigation_config.yaml"
_EXTERNAL_TMAP_SECTIONS = ("definitions", "actions")


def _load_yaml_document(filepath):
    """Load a YAML document from *filepath* using ``CustomSafeLoader``."""
    try:
        with open(filepath, 'r', encoding='utf-8') as stream:
            data = yaml.load(stream, Loader=CustomSafeLoader)
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid YAML syntax in {filepath}: {exc}") from exc
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"File not found: {filepath}") from exc
    except OSError as exc:
        raise RuntimeError(f"Error reading file {filepath}: {exc}") from exc

    return data


def _resolve_reference_path(map_filepath, reference):
    """Resolve *reference* relative to *map_filepath* when needed."""
    if os.path.isabs(reference):
        return os.path.normpath(reference)

    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(map_filepath)), reference)
    )


def _write_yaml_document(filepath, data, dumper):
    """Write *data* as YAML to *filepath*, creating parent directories."""
    directory = os.path.dirname(os.path.abspath(filepath))
    if directory:
        os.makedirs(directory, exist_ok=True)

    with open(filepath, 'w', encoding='utf-8') as stream:
        yaml.dump(data, stream, default_flow_style=False, Dumper=dumper)


def _log_info(logger, message):
    """Log *message* through *logger* when available."""
    if logger is not None:
        logger.info(message)


def load_tmap2_file(
    filepath,
    logger=None,
    return_layout=False,
    navigation_config_file=None,
):
    """Load a tmap2 YAML file, optionally pulling actions/definitions from a sidecar file.

    The main map file can point at a second YAML document via the top-level
    ``navigation_config_file`` key. The referenced file is resolved relative to
    the main map file unless an absolute path is given.

    Only the ``definitions`` and ``actions`` sections are imported from the
    sidecar file. If either section already exists in the main map file, the
    inline value is kept.

    Args:
        filepath: Path to the main topological map YAML file.
        logger: Optional ROS logger.
        return_layout: When ``True``, also return metadata describing which
            sections came from the main map versus the sidecar file.
        navigation_config_file: Optional explicit path to the sidecar YAML file.
            When provided, this takes precedence over the map's
            ``navigation_config_file`` key.

    Returns:
        dict or tuple[dict, dict]: Loaded map, plus optional layout metadata.
    """
    loaded = _load_yaml_document(filepath)
    if not isinstance(loaded, dict):
        raise TypeError(f"Expected dict from {filepath}, got {type(loaded)}")

    layout = {
        "main_path": os.path.abspath(filepath),
        NAVIGATION_CONFIG_FILE_KEY: (
            navigation_config_file or loaded.get(NAVIGATION_CONFIG_FILE_KEY)
        ),
        "config_path": None,
        "section_sources": {},
    }

    config_ref = navigation_config_file or loaded.get(NAVIGATION_CONFIG_FILE_KEY)
    if not config_ref:
        for section in _EXTERNAL_TMAP_SECTIONS:
            layout["section_sources"][section] = "main" if section in loaded else "missing"
        return (loaded, layout) if return_layout else loaded

    config_path = _resolve_reference_path(filepath, config_ref)
    layout["config_path"] = config_path
    _log_info(
        logger,
        f"Loading navigation config from {config_path}",
    )

    config_data = _load_yaml_document(config_path)
    if config_data is None:
        config_data = {}
    if not isinstance(config_data, dict):
        raise TypeError(
            f"Expected dict from navigation config {config_path}, got {type(config_data)}"
        )

    merged = deepcopy(loaded)
    merged[NAVIGATION_CONFIG_FILE_KEY] = config_ref
    for section in _EXTERNAL_TMAP_SECTIONS:
        if section in loaded:
            layout["section_sources"][section] = "main"
        elif section in config_data:
            merged[section] = deepcopy(config_data[section])
            layout["section_sources"][section] = "external"
        else:
            layout["section_sources"][section] = "missing"

    return (merged, layout) if return_layout else merged


def save_tmap2_file(tmap, filepath, no_alias=False, layout=None, logger=None):
    """Save a tmap2 YAML file, preserving split navigation config when present.

    Args:
        tmap: Topological map dictionary to write.
        filepath: Output path for the main map YAML file.
        no_alias: Use ``NoAliasDumper`` when ``True``.
        layout: Optional metadata returned by :func:`load_tmap2_file`.
        logger: Optional ROS logger.
    """
    dumper = NoAliasDumper if no_alias else yaml.SafeDumper
    map_copy = deepcopy(tmap)
    config_ref = map_copy.get(NAVIGATION_CONFIG_FILE_KEY)
    if not config_ref:
        _write_yaml_document(filepath, map_copy, dumper)
        return

    section_sources = {}
    if layout is not None:
        section_sources = dict(layout.get("section_sources", {}))

    if not section_sources:
        section_sources = {
            section: "external"
            for section in _EXTERNAL_TMAP_SECTIONS
            if section in map_copy
        }
    has_external_sections = any(
        section_sources.get(section) == "external"
        for section in _EXTERNAL_TMAP_SECTIONS
    )

    original_main_path = None
    if layout is not None:
        original_main_path = layout.get("main_path")

    output_ref = config_ref
    if os.path.isabs(config_ref):
        if original_main_path and os.path.abspath(filepath) != os.path.abspath(original_main_path):
            output_ref = os.path.basename(config_ref)
            _log_info(
                logger,
                "Saving split map to a new location; rewriting absolute "
                f"{NAVIGATION_CONFIG_FILE_KEY} to {output_ref}",
            )
        config_path = (
            config_ref
            if output_ref == config_ref
            else os.path.join(os.path.dirname(os.path.abspath(filepath)), output_ref)
        )
    else:
        config_path = _resolve_reference_path(filepath, output_ref)

    config_doc = {}
    for section in _EXTERNAL_TMAP_SECTIONS:
        if section_sources.get(section) == "external" and section in map_copy:
            config_doc[section] = map_copy.pop(section)

    if output_ref != config_ref:
        map_copy[NAVIGATION_CONFIG_FILE_KEY] = output_ref

    _write_yaml_document(filepath, map_copy, dumper)

    if has_external_sections:
        _write_yaml_document(config_path, config_doc, dumper)
        _log_info(logger, f"Saved navigation config to {config_path}")


# ========================


def get_node_from_tmap2(top_map, node_name):
    """Given a topological map 2 and a node name return the node object."""
    for i in top_map["nodes"]:
        if i["node"]["name"] == node_name:
            return i
    return None


def get_edge_from_id_tmap2(top_map, node_name, edge_id):
    """Given a node and the edge_id return the edge object for topomap 2."""
    node = get_node_from_tmap2(top_map, node_name)
    if node:
        for i in node["node"]["edges"]:
            if i["edge_id"] == edge_id:
                return i
    return None
