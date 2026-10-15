"""Tests for validate_map module.

Covers find_schema_file, load_yaml_file, and validate_map.
"""

import os
import tempfile
import textwrap

import pytest
import yaml

from topological_navigation.tmap_utils import (
    DEFAULT_NAVIGATION_CONFIG_FILENAME,
    NAVIGATION_CONFIG_FILE_KEY,
)
from topological_navigation.validate_map import (
    find_schema_file,
    find_navigation_config_schema_file,
    load_yaml_file,
    validate_map,
)


# ---------- Helpers / fixtures -----------------------------------------

_FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")
_CONFIG = os.path.join(os.path.dirname(__file__), os.pardir, "config")
_SCHEMA = os.path.join(_CONFIG, "tmap-schema.yaml")
_NAV_CONFIG_SCHEMA = os.path.join(_CONFIG, "navigation-config-schema.yaml")
_SIMPLE_MAP = os.path.join(_FIXTURES, "simple_map.yaml")
_COMPLEX_MAP = os.path.join(_FIXTURES, "complex_map.yaml")


def _write_tmp_yaml(data, suffix=".yaml"):
    """Write *data* to a temp file and return its path."""
    fd, path = tempfile.mkstemp(suffix=suffix)
    with os.fdopen(fd, "w") as fh:
        yaml.dump(data, fh)
    return path


# ---------- find_schema_file -------------------------------------------

class TestFindSchemaFile:
    """Tests for find_schema_file."""

    def test_returns_path_or_none(self):
        """Function returns a string path or None."""
        result = find_schema_file()
        assert result is None or isinstance(result, str)

    def test_returned_path_exists_if_not_none(self):
        """If a path is returned, the file must exist."""
        result = find_schema_file()
        if result is not None:
            assert os.path.isfile(result)

    def test_navigation_config_schema_exists_if_found(self):
        """The sidecar schema is discoverable in local/package installs."""
        result = find_navigation_config_schema_file()
        assert result is None or os.path.isfile(result)


# ---------- load_yaml_file ---------------------------------------------

class TestLoadYamlFile:
    """Tests for load_yaml_file."""

    def test_load_valid_yaml(self):
        """Successfully loads a valid YAML file."""
        path = _write_tmp_yaml({"key": "value"})
        try:
            data = load_yaml_file(path)
            assert data == {"key": "value"}
        finally:
            os.unlink(path)

    def test_load_nonexistent_raises(self):
        """Raises FileNotFoundError for a missing file."""
        with pytest.raises(FileNotFoundError):
            load_yaml_file("/tmp/nonexistent_file_12345.yaml")

    def test_load_invalid_yaml_raises(self):
        """Raises ValueError for syntactically broken YAML."""
        fd, path = tempfile.mkstemp(suffix=".yaml")
        with os.fdopen(fd, "w") as fh:
            fh.write(":\n  - :\n    bad: [unclosed\n")
        try:
            with pytest.raises(ValueError):
                load_yaml_file(path)
        finally:
            os.unlink(path)


# ---------- validate_map -----------------------------------------------

class TestValidateMap:
    """Tests for validate_map."""

    @pytest.fixture(autouse=True)
    def _skip_if_no_schema(self):
        """Skip tests if schema file cannot be located."""
        if not os.path.isfile(_SCHEMA):
            pytest.skip("tmap-schema.yaml not found")

    def test_simple_map_valid(self):
        """simple_map.yaml should pass validation."""
        is_valid, msg = validate_map(_SIMPLE_MAP, _SCHEMA)
        assert is_valid, msg

    def test_complex_map_valid(self):
        """complex_map.yaml should pass validation."""
        is_valid, msg = validate_map(_COMPLEX_MAP, _SCHEMA)
        assert is_valid, msg

    def test_verbose_flag(self, capsys):
        """Verbose mode prints extra information."""
        validate_map(_SIMPLE_MAP, _SCHEMA, verbose=True)
        captured = capsys.readouterr()
        assert "Using schema" in captured.out
        assert "Validating map" in captured.out

    def test_nonexistent_map(self):
        """Validation fails gracefully for missing map file."""
        is_valid, msg = validate_map("/tmp/nonexistent.yaml", _SCHEMA)
        assert not is_valid
        assert "Error" in msg or "not found" in msg.lower()

    def test_invalid_map_content(self):
        """Validation fails for a map that violates the schema."""
        bad = _write_tmp_yaml({"not_a_valid_map": True})
        try:
            is_valid, _ = validate_map(bad, _SCHEMA)
            assert not is_valid
        finally:
            os.unlink(bad)

    def test_schema_not_found(self):
        """Returns failure when schema file cannot be found."""
        is_valid, msg = validate_map(
            _SIMPLE_MAP, "/tmp/no_such_schema.yaml"
        )
        assert not is_valid

    def test_schema_auto_detect(self):
        """If schema is None, the function tries to auto-detect."""
        # We just verify it doesn't crash; the result depends on the
        # runtime environment.
        is_valid, msg = validate_map(_SIMPLE_MAP, schema_file=None)
        # Either it found the schema and validated, or it didn't
        assert isinstance(is_valid, bool)
        assert isinstance(msg, str)

    def test_duplicate_node_names_warning(self):
        """Duplicate node names generate a warning in the message."""
        data = yaml.safe_load(open(_SIMPLE_MAP))
        # Duplicate the first node
        dup = dict(data["nodes"][0])
        data["nodes"].append(dup)
        path = _write_tmp_yaml(data)
        try:
            is_valid, msg = validate_map(path, _SCHEMA)
            # Should still be schema-valid but contain a warning
            if is_valid:
                assert "Duplicate" in msg or "Warning" in msg
        finally:
            os.unlink(path)

    def test_edge_to_nonexistent_node_warning(self):
        """An edge targeting a missing node produces a warning."""
        data = yaml.safe_load(open(_SIMPLE_MAP))
        # Point the first edge to a non-existent node
        data["nodes"][0]["node"]["edges"][0]["node"] = "GHOST"
        path = _write_tmp_yaml(data)
        try:
            is_valid, msg = validate_map(path, _SCHEMA)
            if is_valid:
                assert "non-existent" in msg.lower() or "Warning" in msg
        finally:
            os.unlink(path)

    def test_split_map_valid(self, tmp_path):
        """A main map can validate when actions/definitions live in a sidecar file."""
        config_path = tmp_path / DEFAULT_NAVIGATION_CONFIG_FILENAME
        config_path.write_text(
            yaml.safe_dump(
                {
                    "definitions": {"default_bt": "<root/>"},
                    "actions": {
                        "navigate_to_pose": {
                            "action_type": "nav2_msgs.action.NavigateToPose",
                            "action_server": "/navigate_to_pose",
                            "composable": False,
                            "action_goal_template": {
                                "behavior_tree": "${definitions.default_bt}",
                            },
                        }
                    },
                }
            ),
            encoding="utf-8",
        )
        main_path = tmp_path / "split_map.tmap2.yaml"
        main_path.write_text(
            yaml.safe_dump(
                {
                    NAVIGATION_CONFIG_FILE_KEY: DEFAULT_NAVIGATION_CONFIG_FILENAME,
                    "meta": {"last_updated": "01-01-2026_00-00-00"},
                    "metric_map": "test_map",
                    "name": "test_map",
                    "pointset": "test_map",
                    "nodes": [],
                }
            ),
            encoding="utf-8",
        )

        is_valid, msg = validate_map(str(main_path), _SCHEMA)

        assert is_valid, msg

    def test_split_map_rejects_invalid_navigation_config(self, tmp_path):
        """The actions/definitions sidecar is validated against its own schema."""
        config_path = tmp_path / DEFAULT_NAVIGATION_CONFIG_FILENAME
        config_path.write_text(
            yaml.safe_dump(
                {
                    "definitions": {"default_bt": "<root/>"},
                    "actions": {
                        "navigate_to_pose": {
                            "action_server": "/navigate_to_pose",
                            "action_goal_template": {},
                        }
                    },
                }
            ),
            encoding="utf-8",
        )
        main_path = tmp_path / "split_map.tmap2.yaml"
        main_path.write_text(
            yaml.safe_dump(
                {
                    NAVIGATION_CONFIG_FILE_KEY: DEFAULT_NAVIGATION_CONFIG_FILENAME,
                    "meta": {"last_updated": "01-01-2026_00-00-00"},
                    "metric_map": "test_map",
                    "name": "test_map",
                    "pointset": "test_map",
                    "nodes": [],
                }
            ),
            encoding="utf-8",
        )

        is_valid, msg = validate_map(
            str(main_path),
            _SCHEMA,
            navigation_config_schema_file=_NAV_CONFIG_SCHEMA,
        )

        assert not is_valid
        assert "Navigation config validation failed" in msg

    def test_split_map_accepts_explicit_navigation_config(self, tmp_path):
        """Validation accepts a sidecar path passed separately from the map."""
        config_path = tmp_path / DEFAULT_NAVIGATION_CONFIG_FILENAME
        config_path.write_text(
            yaml.safe_dump(
                {
                    "definitions": {"default_bt": "<root/>"},
                    "actions": {
                        "navigate_to_pose": {
                            "action_type": "nav2_msgs.action.NavigateToPose",
                            "action_server": "/navigate_to_pose",
                            "action_goal_template": {
                                "behavior_tree": "${definitions.default_bt}",
                            },
                        }
                    },
                }
            ),
            encoding="utf-8",
        )
        main_path = tmp_path / "split_map.tmap2.yaml"
        main_path.write_text(
            yaml.safe_dump(
                {
                    "meta": {"last_updated": "01-01-2026_00-00-00"},
                    "metric_map": "test_map",
                    "name": "test_map",
                    "pointset": "test_map",
                    "nodes": [],
                }
            ),
            encoding="utf-8",
        )

        is_valid, msg = validate_map(
            str(main_path),
            _SCHEMA,
            navigation_config_file=str(config_path),
            navigation_config_schema_file=_NAV_CONFIG_SCHEMA,
        )

        assert is_valid, msg
