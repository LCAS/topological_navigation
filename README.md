# Topological Navigation Repository

This repository provides a graph-based navigation workflow on top of Nav2.
It focuses on sending the robot between named graph nodes while keeping edge
behavior configurable (default, slow, reverse, or custom BT per edge).

## Repository Outline

- `topological_navigation`: core routing and visualization logic
- `topological_navigation_demo`: simulation-focused demo package and configs
- `topological_navigation_msgs`: custom ROS interfaces used by the stack
- `docker-compose.yml` and `Dockerfile`: containerized demo environment

## Start The Demo (X11 + Docker)

From the host machine:

```bash
xhost +local:docker
docker compose -f topological_navigation/docker-compose.yml up --build
```

To stop:

```bash
docker compose -f topological_navigation/docker-compose.yml down
xhost -local:docker
```

If you are in `topological_navigation/`, you can use
`docker compose up --build` directly.
