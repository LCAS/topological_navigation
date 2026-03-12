# topological_navigation_msgs

## Purpose

Interface package defining custom ROS messages and actions used by topological navigation.

## Core Functionalities

- Defines the named-waypoint execution action interface.
- Defines message types used to report graph-localization context (for example, closest node output).
- Provides a stable contract between navigation logic, visual tools, and client nodes.

## Custom Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `msg/ClosestNode` | Message | Reports the nearest graph node to the robot and the distance to that node. |
| `action/ExecuteNamedWaypoints` | Action | Accepts a list of waypoint names and requests route planning/execution between them. |
