# topological_navigation

## Purpose

`topological_navigation` is the core behavior package for this repository. It
connects a graph-based view of the world (nodes and edges) to Nav2 motion
execution, so goals can be expressed as named waypoints rather than raw poses.

## Core Functionalities

- Provides a named-waypoint action interface for clients.
- Resolves waypoint names to graph node IDs, plans routes via `nav2_route`,
  and executes them segment-by-segment through Nav2.
- Applies per-edge behavior policy: default, slow, reverse, or custom BT
  selection per segment.
- Publishes graph context (e.g. closest node) so other components can reason
  in topological terms.
- Supports RViz operator tooling: click-to-route selection, route preview
  rendering, and edge behavior visualization.

## How `nav2_route` Is Connected

`nav2_route` is the graph path planner for this package.
`topological_navigation` calls its `ComputeRoute` action (`/compute_route`)
for each waypoint leg and applies edge behavior policy to the returned route
before issuing Nav2 execution goals. The full request/response flow is
documented in the [Interface And Data Flow](#interface-and-data-flow) section
below.

## Interface And Data Flow

### 1. What You Can Send To `topological_navigation`

Top-level action:

- `/execute_named_waypoints`
- Type: `topological_navigation_msgs/action/ExecuteNamedWaypoints`

Goal fields:

- `waypoint_names: string[]`
- `execute_navigation: bool`

Supported request shapes:

- Single leg: two names, e.g. `['A', 'B']`
- Multi-leg route: two or more names, e.g. `['A', 'C', 'F', 'H']`

Notes:

- A request must contain at least 2 waypoint names.
- Multi-waypoint requests are executed as consecutive legs.

### 2. What `topological_navigation` Sends To `nav2_route`

For each consecutive pair of waypoint names, `topological_navigation` resolves
names to node IDs and sends one `ComputeRoute` request:

- Action: `/compute_route`
- Type: `nav2_msgs/action/ComputeRoute`
- Fields used per leg: `start_id`, `goal_id`

So a request like `['A', 'C', 'F']` becomes two route computations:

- leg 1: `A -> C`
- leg 2: `C -> F`

### 3. What `nav2_route` Returns

Each `ComputeRoute` result includes:

- `route` (`nav2_msgs/Route`)
- `path` (`nav_msgs/Path`)
- `planning_time`
- `error_code`, `error_msg`

`route` contains ordered:

- `nodes` (`RouteNode[]`)
- `edges` (`RouteEdge[]`)

### 4. What `topological_navigation` Sends To Nav2 Executors

After receiving route(s), `topological_navigation` builds execution segments and
sends motion goals to Nav2 execution actions:

- `NavigateThroughPoses` for multi-pose segments
- `NavigateToPose` for single-pose segments

It also applies edge behavior policy while executing by selecting controllers
and behavior settings per segment (for example default/slow/reverse/custom BT).


