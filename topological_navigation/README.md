Topological Navigation
======================

> **ROS 2 only.** This is the ROS 2 version of topological navigation (v4.0.0+); all ROS 1 code has been removed.
> For ROS 1 support, use version 3.x or earlier.

This is the core package of the topological navigation framework (originally devised for the STRANDS system). It provides map management, localisation, route planning and navigation execution on a graph of nodes and edges.

See the [repository README](../README.md) for the architecture overview, launch files, node parameters, topics, services and actions. Per-component documentation is in [doc/](doc/):

| Document | Content |
|----------|---------|
| [NAVIGATION2.md](doc/NAVIGATION2.md) | `navigation2.py`, the map-driven navigation server |
| [SET_ROS_PARAMS.md](doc/SET_ROS_PARAMS.md) | Setting ROS parameters on edge traversal |
| [MAP_ANALYSER.md](doc/MAP_ANALYSER.md) | `map_analyser.py`: analyse, check, render, minify and merge maps |
| [TOPO_STATS.md](doc/TOPO_STATS.md) | `topo_stats.py`: navigation statistics database tools |
| [WEB_APP.md](doc/WEB_APP.md) | Browser-based web app (`topological_navigation_visual`) |

## Flexible Properties System

Nodes and edges support an optional, freeform `properties` dictionary for custom metadata (navigation tolerances, semantic labels, boundaries, namespaced application settings) without any schema change. Always access properties defensively, e.g. `node["node"].get("properties", {}).get("xy_goal_tolerance", 0.5)`.

## Launching Localisation and Navigation

The complete stack (map manager, localisation, navigation, simulator, visualiser, RViz and web app) starts with:

```bash
ros2 launch topological_navigation topological_navigation.launch.py map_path:=/path/to/map.tmap2.yaml
```

To run the nodes individually (for example on a real robot with Nav2):

```bash
ros2 run topological_navigation map_manager2.py /path/to/map.tmap2.yaml
ros2 run topological_navigation localisation2.py
ros2 run topological_navigation navigation2.py
```

Every action server referenced by the `actions` section of the map (for example the Nav2 `navigate_to_pose` server) has to be running.

## Navigate

Send the robot to a node with the `GotoNode` action:

```bash
ros2 action send_goal /topological_navigation \
    topological_navigation_msgs/action/GotoNode "{target: 'node_name'}"
```

You can also click a node in the web app (`topological_web_app.py`, port 8083) or use the **Navigate Here** context menu of the `topological_map_visualiser.py` markers in RViz.

## Creating Topological Maps

Maps are `.tmap2.yaml` / `.tmap3.yaml` files; validate them with `ros2 run topological_navigation validate_map.py <map>`.

1. **Record while driving**: `ros2 run topological_navigation map_recorder.py`, then start the `/record` action (`topological_navigation_msgs/action/RecordMap`). Nodes are added every `node_distance` metres and the result is saved with the `/map_recorder/save_map` service.
2. **Joystick**: `ros2 run topological_navigation manual_topomapping.py` adds waypoints at the robot's pose with joystick buttons and publishes the map on `/topological_map_2`.
3. **Edit in RViz**: `ros2 launch topological_navigation_visual topological_map_visualiser.launch.py map_file:=<map> edit_mode:=true`, drag nodes in RViz and save with the `/topological_map_visualiser/save_map` service.
4. **Convert a legacy map**: `ros2 run topological_navigation convert_tmap.py old.yaml -o new.tmap3.yaml`.
5. **Merge maps**: `map_analyser.py merge a.tmap2.yaml b.tmap2.yaml -o merged.tmap2.yaml`.

## Using Localise by Topic

A node can define a `localise_by_topic` JSON string (empty by default, meaning the robot is localised by its pose only). The following fields are parsed by `update_loc_by_topic_nx()` in `networkx_utils.py`:

- *topic*: the name of the topic used for localisation
- *field*: the field of the topic that is compared
- *val*: the value of *field* that sets the current node

```JSON
{"topic": "/battery_state", "field": "charging", "val": true}
```

Optional fields:

- *localise_anywhere*: (default **true**) if **true**, the node is reported as the current node regardless of the robot's pose; if **false**, the robot must also be inside the node's influence zone
- *persistency*: (default **10**) number of consecutive messages with the correct value required

Note: a node configured this way is never reported as the *closest node* unless it is also the *current node*. `localisation2.py` currently reads this configuration but does not subscribe to `topic` or evaluate `field`/`val`, so such a node is always treated as active.
