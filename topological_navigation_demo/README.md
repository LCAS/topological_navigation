# topological_navigation_demo

## Purpose

`topological_navigation_demo` deploys a TurtleBot3 in the standard TurtleBot3
simulation world with a pre-built topological graph overlaid on the
environment. Once launched, you can click on any graph node in RViz to send
the robot there. The demo exists to give a runnable reference for the full
stack and to act as a template for adapting the system to a new environment.

**Startup note:** after launching, wait for the TurtleBot3 model in Gazebo to
change from white to grey before interacting — this indicates that all
Nav2 and localisation nodes have finished initialising.

## What It Contains

- A topological graph (nodes and edges) that covers the TurtleBot3 world.
- Edge behavior configuration: which edges use default, slow, reverse, or
  custom BT traversal.
- Nav2 and controller parameters tuned for stable edge-following in
  simulation.
- Launch files and environment startup helpers.

## Using Toponav In Your Own Project

This package is the recommended starting point for adopting the stack in a
real project. The core `topological_navigation` package is the reusable
engine; this demo package is the environment-specific wrapper around it.

To adapt it:

- Create a package (e.g. `my_robot_toponav`) modelled on this one.
- Replace the graph, edge behavior config, and Nav2 parameters with your own.
- Keep `topological_navigation` and `topological_navigation_msgs` unchanged.

### What To Provide

- A topological graph for your environment (nodes and traversable edges).
- Edge behavior policy: which edges use slow, reverse, or custom BT traversal.
- Nav2 parameters tuned for your robot's footprint, dynamics, and environment.
- A localisation source that keeps the robot pose and map frame stable.

### Typical Adaptation Flow

1. Copy this package's structure as a starting point.
2. Replace graph and config files with your environment-specific versions.
3. Tune routing and navigation until paths and edge-following are stable.
4. Clients drive the robot using named waypoint goals — no code changes needed.

