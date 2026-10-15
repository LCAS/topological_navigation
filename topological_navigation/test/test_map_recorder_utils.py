"""Tests for map_recorder_utils module (map recording core logic)."""

import pytest

from topological_navigation.map_recorder_utils import (
    MAP_SOURCE_PROPERTY_VALUE,
    MapRecorderCore,
    distance_xy,
    find_loop_closure_node,
    init_topomap,
    make_bidirectional_edges,
    make_node_dict,
    topomap_from_source,
)


# ---------- Fixtures ----------------------------------------------------

SQUARE_VERTS = [
    {"x": -0.5, "y": -0.5},
    {"x": 0.5, "y": -0.5},
    {"x": 0.5, "y": 0.5},
    {"x": -0.5, "y": 0.5},
]


@pytest.fixture
def template_node():
    return {
        "meta": {"map": "map_2d", "node": "NodeName", "pointset": "PointSet"},
        "node": {
            "edges": [],
            "name": "NodeName",
            "nav_frame": "map",
            "pose": {
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "properties": {"xy_goal_tolerance": 0.3, "yaw_goal_tolerance": 6.29},
            "verts": [dict(v) for v in SQUARE_VERTS],
        },
    }


@pytest.fixture
def template_edge():
    return {
        "action": "navigate_to_pose",
        "edge_id": "origin_destination",
        "node": "destination",
        "properties": {},
    }


@pytest.fixture
def template_action():
    return {
        "definitions": {"default_bt": "<root/>"},
        "actions": {
            "navigate_to_pose": {
                "action_type": "nav2_msgs.action.NavigateToPose",
            }
        },
    }


@pytest.fixture
def recorder(template_node, template_edge, template_action):
    return MapRecorderCore(
        template_node, template_edge, template_action,
        pointset="test_map", site_name="test_map",
    )


def pose(x, y, z=0.0):
    return {"x": x, "y": y, "z": z}


# ---------- init_topomap / topomap_from_source ---------------------------

def test_init_topomap_empty(template_action):
    tmap = init_topomap("my_map", "site", template_action)
    assert tmap["nodes"] == []
    assert tmap["pointset"] == "my_map"
    assert tmap["actions"] == template_action["actions"]


def test_topomap_from_source_inherits_metadata(template_action):
    source = {
        "metric_map": "site_a",
        "name": "site_a",
        "pointset": "site_a",
        "transformation": {"topo_frame_id": "site_a", "parent": "map"},
        "actions": {"custom_action": {}},
    }
    tmap = topomap_from_source(source, "", "", template_action)
    assert tmap["pointset"] == "site_a"
    assert tmap["transformation"]["topo_frame_id"] == "site_a"
    assert tmap["actions"] == {"custom_action": {}}


def test_topomap_from_source_none(template_action):
    tmap = topomap_from_source(None, "my_map", "site", template_action)
    assert tmap["pointset"] == "my_map"


# ---------- distance_xy ---------------------------------------------------

def test_distance_xy():
    assert distance_xy(pose(0, 0), pose(3, 4)) == pytest.approx(5.0)
    assert distance_xy(pose(1, 1), pose(1, 1)) == 0.0


# ---------- make_node_dict / make_bidirectional_edges ----------------------

def test_make_node_dict_sets_position_and_properties(template_node):
    node_dict = make_node_dict(
        template_node, "node0", pose(1.0, 2.0), "test_map", "test_map",
        extra_properties={"map": {"source": "recording"}},
    )
    nd = node_dict["node"]
    assert nd["name"] == "node0"
    assert nd["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 0.0}
    assert nd["properties"]["map"] == {"source": "recording"}
    # Template defaults are preserved alongside the new property.
    assert nd["properties"]["xy_goal_tolerance"] == 0.3


def test_make_bidirectional_edges(template_edge):
    fwd, rev = make_bidirectional_edges(template_edge, "node0", "node1")
    assert fwd["edge_id"] == "node0_node1"
    assert fwd["node"] == "node1"
    assert rev["edge_id"] == "node1_node0"
    assert rev["node"] == "node0"


# ---------- find_loop_closure_node -----------------------------------------

def test_find_loop_closure_node_inside_zone(template_node, template_action):
    tmap = init_topomap("test_map", "test_map", template_action)
    tmap["nodes"].append(
        make_node_dict(template_node, "node0", pose(0.0, 0.0), "test_map", "test_map")
    )
    assert find_loop_closure_node(tmap, pose(0.1, 0.1)) == "node0"
    assert find_loop_closure_node(tmap, pose(5.0, 5.0)) is None


def test_find_loop_closure_node_empty_map(template_action):
    tmap = init_topomap("test_map", "test_map", template_action)
    assert find_loop_closure_node(tmap, pose(0.0, 0.0)) is None


# ---------- MapRecorderCore: add_node --------------------------------------

def test_add_node_creates_first_node(recorder):
    name, created, _ = recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    assert created is True
    assert name == "node0"
    assert recorder.num_nodes() == 1
    assert recorder.last_node_name() == "node0"

    node = recorder._get_node("node0")["node"]
    assert node["properties"]["map"]["source"] == MAP_SOURCE_PROPERTY_VALUE
    assert node["edges"] == []


def test_add_node_respects_node_distance(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    name, created, msg = recorder.add_node(pose(0.2, 0.0), node_distance=1.0)
    assert created is False
    assert name == "node0"
    assert "too close" in msg
    assert recorder.num_nodes() == 1


def test_add_node_creates_bidirectional_edge_to_previous_node(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    name, created, _ = recorder.add_node(pose(2.0, 0.0), node_distance=1.0)
    assert created is True
    assert name == "node1"

    node0 = recorder._get_node("node0")["node"]
    node1 = recorder._get_node("node1")["node"]
    assert any(e["node"] == "node1" for e in node0["edges"])
    assert any(e["node"] == "node0" for e in node1["edges"])


def test_add_node_far_apart_not_linked_directly(recorder):
    """Only successive nodes are linked, not every pair."""
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    recorder.add_node(pose(2.0, 0.0), node_distance=1.0)
    recorder.add_node(pose(4.0, 0.0), node_distance=1.0)

    node0 = recorder._get_node("node0")["node"]
    node2 = recorder._get_node("node2")["node"]
    assert not any(e["node"] == "node2" for e in node0["edges"])
    assert not any(e["node"] == "node0" for e in node2["edges"])


def test_add_node_force_bypasses_distance_check(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=5.0)
    # (1.0, 1.0) is outside node0's influence zone but well within the 5m
    # node_distance threshold, so only `force=True` allows a new node here.
    name, created, _ = recorder.add_node(pose(1.0, 1.0), node_distance=5.0, force=True)
    assert created is True
    assert name == "node1"


def test_add_node_loop_closure_links_existing_node(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)   # node0
    recorder.add_node(pose(2.0, 0.0), node_distance=1.0)   # node1
    recorder.add_node(pose(4.0, 0.0), node_distance=1.0)   # node2

    # Driving back close to node0's influence zone should reuse node0,
    # not create a new node, and link node2 <-> node0.
    name, created, msg = recorder.add_node(pose(0.05, 0.05), node_distance=1.0)
    assert created is False
    assert name == "node0"
    assert "loop closure" in msg
    assert recorder.num_nodes() == 3  # unique nodes: node0, node1, node2
    assert recorder.num_visits() == 4  # visits include the repeated node0

    node0 = recorder._get_node("node0")["node"]
    node2 = recorder._get_node("node2")["node"]
    assert any(e["node"] == "node0" for e in node2["edges"])
    assert any(e["node"] == "node2" for e in node0["edges"])


def test_add_node_stays_in_same_zone_does_not_duplicate(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    name, created, msg = recorder.add_node(pose(0.05, 0.05), node_distance=0.0, force=True)
    assert created is False
    assert name == "node0"
    assert recorder.num_nodes() == 1


# ---------- MapRecorderCore: delete_last_node ------------------------------

def test_delete_last_node(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    recorder.add_node(pose(2.0, 0.0), node_distance=1.0)

    success, _ = recorder.delete_last_node()
    assert success is True
    assert recorder.num_nodes() == 1
    assert recorder.last_node_name() == "node0"

    node0 = recorder._get_node("node0")["node"]
    assert node0["edges"] == []


def test_delete_last_node_empty_map(recorder):
    success, msg = recorder.delete_last_node()
    assert success is False
    assert "no nodes" in msg


def test_delete_last_node_after_loop_closure_keeps_reused_node(recorder):
    """Undoing a loop-closure "visit" must not delete the still-referenced node."""
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)   # node0
    recorder.add_node(pose(2.0, 0.0), node_distance=1.0)   # node1
    recorder.add_node(pose(4.0, 0.0), node_distance=1.0)   # node2

    name, created, _ = recorder.add_node(pose(0.05, 0.05), node_distance=1.0)
    assert created is False
    assert name == "node0"
    assert recorder.num_nodes() == 3
    assert recorder.num_visits() == 4

    success, message = recorder.delete_last_node()
    assert success is True
    assert "node0" in message
    # node0 must still exist (it is still referenced by the earlier visit).
    assert recorder._get_node("node0") is not None
    assert recorder.num_nodes() == 3
    assert recorder.num_visits() == 3
    assert recorder.last_node_name() == "node2"

    # The loop-closure edge between node2 and node0 must have been undone.
    node0 = recorder._get_node("node0")["node"]
    node2 = recorder._get_node("node2")["node"]
    assert not any(e["node"] == "node0" for e in node2["edges"])
    assert not any(e["node"] == "node2" for e in node0["edges"])
    # But node0's original edge to node1 (from the first pass) is untouched.
    assert any(e["node"] == "node1" for e in node0["edges"])


# ---------- MapRecorderCore: reset / load ----------------------------------

def test_reset_clears_map(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    recorder.reset()
    assert recorder.num_nodes() == 0
    assert recorder.tmap["nodes"] == []


def test_load_seeds_map_for_extension(recorder, template_action):
    source_tmap = init_topomap("existing_map", "existing_map", template_action)
    source_tmap["nodes"].append(
        make_node_dict(recorder.template_node, "existing0", pose(10.0, 10.0),
                       "existing_map", "existing_map")
    )

    recorder.load(source_tmap)
    assert recorder.num_nodes() == 1
    assert recorder.last_node_name() == "existing0"

    name, created, _ = recorder.add_node(pose(12.0, 10.0), node_distance=1.0)
    assert created is True
    assert name == "node1"

    existing0 = recorder._get_node("existing0")["node"]
    assert any(e["node"] == "node1" for e in existing0["edges"])


# ---------- MapRecorderCore: to_yaml ----------------------------------------

def test_to_yaml_round_trips(recorder):
    recorder.add_node(pose(0.0, 0.0), node_distance=1.0)
    yaml_text = recorder.to_yaml()
    assert "node0" in yaml_text
    assert "pointset: test_map" in yaml_text
