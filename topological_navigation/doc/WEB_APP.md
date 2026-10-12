# Topological Navigation Web App

`topological_web_app.py` (package `topological_navigation_visual`) is a ROS 2
node that serves a browser-based UI for monitoring and commanding a robot on a
topological map. It works on desktops, tablets and phones, needs no internet
connection, and serves everything from a single port (default **8083**).

```bash
ros2 run topological_navigation_visual topological_web_app.py
# then open http://<robot>:8083/
```

The web app is started by default by
`ros2 launch topological_navigation topological_navigation.launch.py`
(disable it with `web_app:=false`, change the port with `web_app_port:=<port>`).

## Features

- **Map graph.** The current topological map (`/topological_map_2`) is drawn
  as a graph at the nodes' metric positions. You can zoom with the mouse wheel
  or pinch, and pan by dragging. **Fit map** and **Centre on robot** reset the
  view.
- **Roboflow nodes.** A node whose properties contain `roboflow.enabled: true`
  (or the shorthand `roboflow: true`) is drawn in green instead of blue.
- **Node details on hover.** Hovering a node shows its name, pose, outgoing
  edges (with their actions) and the full `properties` dictionary.
- **Live robot position.** The panel shows the current node and the closest
  node. The map shows the robot's metric pose, looked up in TF between the
  map frame and `base_frame`, as a dark disc with a heading arrow.
- **Speed.** Linear speed (m/s) and turn rate (rad/s) are taken from the
  odometry topic.
- **Destination and current goal.** The *destination* is the final target of
  the navigation goal. The *current goal* is the node the robot is heading for
  on the current edge. The current edge is highlighted in red.
- **Send the robot.** Clicking a node and confirming sends a `GotoNode` goal.
- **Cancel.** **Cancel goal** cancels the active navigation, including goals
  sent by other clients such as the CLI or RViz.
- **Route highlighting.** While the robot is navigating, the planned route
  (`/topological_navigation/Route`) is highlighted in orange.

## Stateless, synchronised clients

The backend holds all state. Browsers keep only purely visual state, such as
the zoom level or the hovered node. They render whatever the backend pushes
over a [Server-Sent Events](https://developer.mozilla.org/docs/Web/API/Server-sent_events)
stream. Every client that has the page open shows the same map, robot, goal
and route. A goal sent or cancelled from one browser, or from outside the web
app, appears in all of them right away. Clients reconnect automatically if the
connection drops.

## HTTP API

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | The web UI |
| `GET` | `/api/state` | JSON snapshot of robot and navigation state |
| `GET` | `/api/map` | JSON graph of the current map (`nodes`, `edges`, `frame`, `name`) |
| `GET` | `/api/events` | SSE stream with `map` events (when the map changes) and `state` events (rate-limited by `update_rate`) |
| `POST` | `/api/goto` | Body `{"node": "<name>"}`: send the robot to a node (`202`, `404` for unknown node, `503` if the action server is unavailable) |
| `POST` | `/api/cancel` | Cancel all goals on the navigation action server |

POST requests must use `Content-Type: application/json`.

## Offline assets

The front end uses [React](https://react.dev/) 18 with
[htm](https://github.com/developit/htm) as a build-free JSX alternative, and
[cytoscape.js](https://js.cytoscape.org/) for the graph view. The
production builds of these libraries are bundled in
`topological_navigation_visual/web/vendor/` (licences in
`THIRD_PARTY_LICENSES.txt`) and installed as Python package data. Nothing is
loaded from the internet, and no Node.js or npm build step is needed.

## Deploying behind a reverse proxy (URL prefix)

All URLs used by the page (assets, API and event stream) are **relative**, so
the app can be served under any URL prefix, such as
`https://example.org/robot1/topo/`. If the page is opened without the trailing
slash, it redirects to the URL with one so that relative paths resolve below
the prefix. Configure the proxy to strip the prefix and to not buffer the
event stream. For example, with nginx:

```nginx
location /robot1/topo/ {
    proxy_pass http://robot1:8083/;   # trailing slash strips the prefix
    proxy_http_version 1.1;
    proxy_set_header Connection "";
    proxy_buffering off;              # required for Server-Sent Events
    proxy_read_timeout 1h;
}
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | int | `8083` | HTTP port serving both the UI and the API |
| `host` | string | `"0.0.0.0"` | Interface to bind to (use `127.0.0.1` to restrict to local access) |
| `odom_topic` | string | `"/odometry/global"` | `nav_msgs/Odometry` topic for the speed display |
| `base_frame` | string | `"base_link"` | Robot frame looked up in TF for the metric position |
| `map_frame` | string | `""` | Fixed frame for the TF lookup; empty uses the map's `transformation.topo_frame_id` (or `map`) |
| `action_name` | string | `"/topological_navigation"` | `GotoNode` action server used to send and cancel goals |
| `pose_rate` | double | `5.0` | Rate (Hz) of the TF pose lookup |
| `update_rate` | double | `10.0` | Maximum rate (Hz) at which state updates are pushed to each client |
| `web_root` | string | `""` | Directory with an alternative front end; empty uses the bundled assets |

## ROS interfaces

**Subscribed topics**

| Topic | Type | Purpose |
|-------|------|---------|
| `/topological_map_2` | `std_msgs/String` | Topological map (JSON) |
| `/current_node`, `/closest_node` | `std_msgs/String` | Topological localisation |
| `<odom_topic>` | `nav_msgs/Odometry` | Robot speed |
| `/topological_navigation/Route` | `TopologicalRoute` | Planned route (highlighted while navigating) |
| `/current_edge` | `std_msgs/String` | Edge currently being traversed |
| `/topological_navigation/move_action_status` | `std_msgs/String` | Current goal and final destination (JSON) |
| `/robot_operation_current_status` | `std_msgs/String` | Navigation status (e.g. `EXECUTING`, `SUCCEEDED`) |
| `<action_name>/_action/status` | `action_msgs/GoalStatusArray` | Detects active goals, including those from other clients |

**TF:** `<map frame>` → `base_frame`.

**Action and service clients**

| Name | Type | Purpose |
|------|------|---------|
| `<action_name>` | `topological_navigation_msgs/action/GotoNode` | Send the robot to the clicked node |
| `<action_name>/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | Cancel all active goals |

## Security note

The web app has no authentication. Anyone who can reach the port can send the
robot to a node or cancel navigation. On untrusted networks, bind it to
`127.0.0.1` (`host` parameter) and put an authenticating reverse proxy in
front of it.

## Development

Edit `web/app.js`, `web/style.css` or `web/index.html` directly; there is no
build step. With `colcon build --symlink-install`, reloading the browser is
enough to pick up changes. The backend is split into ROS-independent modules
that can be tested with plain `pytest`:

- `web_state.py`: thread-safe state store and map conversion
- `web_server.py`: HTTP server, static files, SSE and command endpoints

```bash
cd topological_navigation_visual
python -m pytest test/test_web_state.py test/test_web_server.py -v
```

---

**Last Updated**: 2026-10-12
