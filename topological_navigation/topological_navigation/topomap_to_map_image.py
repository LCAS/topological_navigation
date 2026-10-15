#!/usr/bin/env python3
"""Generate a Nav2 map image from a topological map.

Usage:
    ros2 run topological_navigation topomap_to_map_image.py input.tmap3.yaml \
        --white-extension-m 2.0 \
        --border-width-m 0.25 \
        -o output.png \
        --map-yaml output.yaml

The output YAML can be passed to Nav2/localization as the map file.  Increase
``--white-extension-m`` to widen the free corridor around topomap edges, and
increase ``--border-width-m`` to thicken the black occupied border.

The generated image uses the occupancy-map convention expected by Nav2:
white pixels are free space, black pixels are occupied borders, and grey
pixels are unknown outer space.  Free space is produced by buffering the
topological graph edges by a configured distance in metres.
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw
import yaml


Point = Tuple[float, float]
Edge = Tuple[str, str]

UNKNOWN_VALUE = 205
OCCUPIED_VALUE = 0
FREE_VALUE = 255


@dataclass(frozen=True)
class TopoNode:
    """Node geometry used for image generation."""

    name: str
    point: Point
    verts: Tuple[Point, ...]


@dataclass(frozen=True)
class MapGeometry:
    """Topomap geometry transformed into the output map frame."""

    nodes: Dict[str, TopoNode]
    edges: Tuple[Edge, ...]
    frame_id: str


@dataclass(frozen=True)
class RasterResult:
    """Generated map image plus metadata needed by Nav2 map_server."""

    image: Image.Image
    origin: Point
    resolution: float


@dataclass(frozen=True)
class RouteEdgeSpec:
    """Route edge geometry and corridor widths for rasterization."""

    source: str
    target: str
    left_m: float
    right_m: float


def _as_float(value, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid numeric value for {field_name}: {value!r}") from exc


def _yaw_from_quaternion(rotation: dict) -> float:
    x = _as_float(rotation.get("x", 0.0), "transformation.rotation.x")
    y = _as_float(rotation.get("y", 0.0), "transformation.rotation.y")
    z = _as_float(rotation.get("z", 0.0), "transformation.rotation.z")
    w = _as_float(rotation.get("w", 1.0), "transformation.rotation.w")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _transform_point(point: Point, translation: dict, yaw: float) -> Point:
    x, y = point
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    tx = _as_float(translation.get("x", 0.0), "transformation.translation.x")
    ty = _as_float(translation.get("y", 0.0), "transformation.translation.y")
    return (
        tx + (cos_yaw * x) - (sin_yaw * y),
        ty + (sin_yaw * x) + (cos_yaw * y),
    )


def _node_position(node: dict, node_name: str) -> Point:
    pose = node.get("pose", {})
    position = pose.get("position", {})
    if "x" not in position or "y" not in position:
        raise ValueError(f"Node {node_name!r} is missing pose.position x/y")
    return (
        _as_float(position["x"], f"nodes.{node_name}.pose.position.x"),
        _as_float(position["y"], f"nodes.{node_name}.pose.position.y"),
    )


def _node_verts(node: dict, node_name: str) -> Tuple[Point, ...]:
    verts = node.get("verts", [])
    if not verts:
        return tuple()
    if not isinstance(verts, list):
        raise ValueError(f"Node {node_name!r} has non-list verts")

    parsed = []
    for index, vert in enumerate(verts):
        if not isinstance(vert, dict) or "x" not in vert or "y" not in vert:
            raise ValueError(f"Node {node_name!r} has invalid vert at index {index}")
        parsed.append((
            _as_float(vert["x"], f"nodes.{node_name}.verts.{index}.x"),
            _as_float(vert["y"], f"nodes.{node_name}.verts.{index}.y"),
        ))
    return tuple(parsed)


def geometry_from_tmap(tmap_data: dict, apply_transform: bool = True) -> MapGeometry:
    """Extract drawable nodes and edges from a tmap2/tmap3 YAML dictionary."""

    if not isinstance(tmap_data, dict):
        raise ValueError("Topological map YAML must be a mapping")

    raw_nodes = tmap_data.get("nodes", [])
    if not isinstance(raw_nodes, list) or not raw_nodes:
        raise ValueError("Topological map must contain a non-empty 'nodes' list")

    transformation = tmap_data.get("transformation", {})
    parent_frame = transformation.get("parent", "map")
    topo_frame = transformation.get(
        "topo_frame_id", transformation.get("child", parent_frame)
    )
    output_frame = parent_frame if apply_transform else topo_frame
    translation = transformation.get("translation", {}) if apply_transform else {}
    rotation = transformation.get("rotation", {}) if apply_transform else {}
    yaw = _yaw_from_quaternion(rotation)

    nodes: Dict[str, TopoNode] = {}
    raw_edge_refs: List[Edge] = []

    for entry_index, entry in enumerate(raw_nodes):
        if not isinstance(entry, dict) or not isinstance(entry.get("node"), dict):
            raise ValueError(f"Invalid node entry at index {entry_index}")

        node = entry["node"]
        node_name = node.get("name") or entry.get("meta", {}).get("node")
        if not node_name:
            raise ValueError(f"Node entry at index {entry_index} is missing a name")

        local_point = _node_position(node, node_name)
        local_verts = _node_verts(node, node_name)
        absolute_verts = [
            (local_point[0] + vert[0], local_point[1] + vert[1])
            for vert in local_verts
        ]

        point = _transform_point(local_point, translation, yaw)
        verts = tuple(_transform_point(vert, translation, yaw) for vert in absolute_verts)
        nodes[node_name] = TopoNode(name=node_name, point=point, verts=verts)

        edges = node.get("edges", [])
        if not isinstance(edges, list):
            raise ValueError(f"Node {node_name!r} has non-list edges")
        for edge in edges:
            if not isinstance(edge, dict):
                continue
            target_name = edge.get("node")
            if target_name:
                raw_edge_refs.append((node_name, target_name))

    drawable_edges = tuple(
        (source, target)
        for source, target in raw_edge_refs
        if source in nodes and target in nodes
    )
    return MapGeometry(nodes=nodes, edges=drawable_edges, frame_id=output_frame)


def _all_geometry_points(
    geometry: MapGeometry,
    include_node_polygons: bool,
) -> Iterable[Point]:
    for node in geometry.nodes.values():
        yield node.point
        if include_node_polygons:
            yield from node.verts

    for source, target in geometry.edges:
        yield geometry.nodes[source].point
        yield geometry.nodes[target].point


def _route_quad_points(
    start: Point,
    end: Point,
    left_m: float,
    right_m: float,
) -> Tuple[Point, Point, Point, Point]:
    """Build an oriented corridor quad around one directed route edge."""

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    seg_len = math.hypot(dx, dy)
    if seg_len <= 1e-9:
        raise ValueError("Route edge has zero length")

    # Left normal for edge direction (start -> end).
    nx = -dy / seg_len
    ny = dx / seg_len

    start_left = (start[0] + (nx * left_m), start[1] + (ny * left_m))
    end_left = (end[0] + (nx * left_m), end[1] + (ny * left_m))
    end_right = (end[0] - (nx * right_m), end[1] - (ny * right_m))
    start_right = (start[0] - (nx * right_m), start[1] - (ny * right_m))
    return (start_left, end_left, end_right, start_right)


def _edge_width(value: object, default_value: float) -> float:
    if value is None:
        return default_value
    width = _as_float(value, "boundary width")
    if width < 0.0:
        raise ValueError("Boundary widths must be >= 0")
    return width


def route_specs_from_edge_data(
    route_edges: Sequence[dict],
    default_left_m: float,
    default_right_m: float,
) -> Tuple[RouteEdgeSpec, ...]:
    """Build route edge specs from route edge dictionaries.

    Each route edge may optionally contain ``properties.boundary_left`` and
    ``properties.boundary_right``. Missing values fall back to defaults.
    """

    if default_left_m < 0.0 or default_right_m < 0.0:
        raise ValueError("Default boundary widths must be >= 0")

    specs: List[RouteEdgeSpec] = []
    for edge in route_edges:
        source = edge.get("source")
        target = edge.get("target")
        if not source or not target:
            continue

        props = edge.get("properties", {}) or {}
        left_m = _edge_width(props.get("boundary_left"), default_left_m)
        right_m = _edge_width(props.get("boundary_right"), default_right_m)
        specs.append(
            RouteEdgeSpec(
                source=str(source),
                target=str(target),
                left_m=left_m,
                right_m=right_m,
            )
        )

    return tuple(specs)


def _draw_line_with_round_caps(
    draw: ImageDraw.ImageDraw,
    start: Tuple[int, int],
    end: Tuple[int, int],
    radius_px: int,
    fill_value: int,
) -> None:
    width_px = max(1, (2 * radius_px) + 1)
    draw.line([start, end], fill=fill_value, width=width_px)
    if radius_px > 0:
        for x, y in (start, end):
            draw.ellipse(
                [x - radius_px, y - radius_px, x + radius_px, y + radius_px],
                fill=fill_value,
            )


def rasterize_geometry(
    geometry: MapGeometry,
    white_extension_m: float,
    border_width_m: float = 0.25,
    resolution: float = 0.05,
    padding_m: Optional[float] = None,
    include_node_polygons: bool = False,
    unknown_value: int = UNKNOWN_VALUE,
    occupied_value: int = OCCUPIED_VALUE,
    free_value: int = FREE_VALUE,
) -> RasterResult:
    """Rasterize topomap geometry into a Nav2 occupancy-style image."""

    if white_extension_m < 0.0:
        raise ValueError("white_extension_m must be >= 0")
    if border_width_m < 0.0:
        raise ValueError("border_width_m must be >= 0")
    if resolution <= 0.0:
        raise ValueError("resolution must be > 0")
    if padding_m is None:
        padding_m = white_extension_m
    if padding_m < 0.0:
        raise ValueError("padding_m must be >= 0")
    for name, value in (
        ("unknown_value", unknown_value),
        ("occupied_value", occupied_value),
        ("free_value", free_value),
    ):
        if value < 0 or value > 255:
            raise ValueError(f"{name} must be in the range [0, 255]")

    points = list(_all_geometry_points(geometry, include_node_polygons))
    if not points:
        raise ValueError("No drawable geometry found in topological map")

    bounds_margin = white_extension_m + border_width_m + padding_m
    min_x = min(point[0] for point in points) - bounds_margin
    max_x = max(point[0] for point in points) + bounds_margin
    min_y = min(point[1] for point in points) - bounds_margin
    max_y = max(point[1] for point in points) + bounds_margin

    width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
    height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
    image_top_y = min_y + ((height - 1) * resolution)

    def world_to_pixel(point: Point) -> Tuple[int, int]:
        px = int(round((point[0] - min_x) / resolution))
        py = int(round((image_top_y - point[1]) / resolution))
        return px, py

    image = Image.new("L", (width, height), unknown_value)
    draw = ImageDraw.Draw(image)
    white_radius_px = int(math.ceil(white_extension_m / resolution))
    border_radius_px = int(math.ceil((white_extension_m + border_width_m) / resolution))

    if include_node_polygons:
        for node in geometry.nodes.values():
            if len(node.verts) < 3:
                continue
            pixels = [world_to_pixel(point) for point in node.verts]
            draw.polygon(pixels, fill=occupied_value)
            for index, start in enumerate(pixels):
                end = pixels[(index + 1) % len(pixels)]
                _draw_line_with_round_caps(
                    draw, start, end, border_radius_px, occupied_value
                )
            draw.polygon(pixels, fill=free_value)
            for index, start in enumerate(pixels):
                end = pixels[(index + 1) % len(pixels)]
                _draw_line_with_round_caps(
                    draw, start, end, white_radius_px, free_value
                )

    for source, target in geometry.edges:
        _draw_line_with_round_caps(
            draw,
            world_to_pixel(geometry.nodes[source].point),
            world_to_pixel(geometry.nodes[target].point),
            border_radius_px,
            occupied_value,
        )

    for source, target in geometry.edges:
        _draw_line_with_round_caps(
            draw,
            world_to_pixel(geometry.nodes[source].point),
            world_to_pixel(geometry.nodes[target].point),
            white_radius_px,
            free_value,
        )

    return RasterResult(image=image, origin=(min_x, min_y), resolution=resolution)


def rasterize_route_geometry(
    geometry: MapGeometry,
    route_edges: Sequence[RouteEdgeSpec],
    border_width_m: float = 0.25,
    resolution: float = 0.05,
    padding_m: float = 0.0,
    unknown_value: int = UNKNOWN_VALUE,
    occupied_value: int = OCCUPIED_VALUE,
    free_value: int = FREE_VALUE,
    route_start: Optional[Point] = None,
) -> RasterResult:
    """Rasterize route edges with per-edge widths.

    When ``route_start`` is supplied, it replaces the source-node position of
    the first valid edge. This lets a live route corridor start at the robot's
    current position while subsequent edges remain anchored to topological
    nodes.
    """

    if border_width_m < 0.0:
        raise ValueError("border_width_m must be >= 0")
    if resolution <= 0.0:
        raise ValueError("resolution must be > 0")
    if padding_m < 0.0:
        raise ValueError("padding_m must be >= 0")

    for name, value in (
        ("unknown_value", unknown_value),
        ("occupied_value", occupied_value),
        ("free_value", free_value),
    ):
        if value < 0 or value > 255:
            raise ValueError(f"{name} must be in the range [0, 255]")

    valid_edges: List[Tuple[RouteEdgeSpec, Point, Point]] = []
    bounds_points: List[Point] = []

    for edge in route_edges:
        if edge.source not in geometry.nodes or edge.target not in geometry.nodes:
            continue
        if edge.left_m < 0.0 or edge.right_m < 0.0:
            raise ValueError("Boundary widths must be >= 0")

        start = (
            route_start
            if route_start is not None and not valid_edges
            else geometry.nodes[edge.source].point
        )
        end = geometry.nodes[edge.target].point
        try:
            quad = _route_quad_points(start, end, edge.left_m, edge.right_m)
        except ValueError:
            continue

        valid_edges.append((edge, start, end))
        bounds_points.extend(quad)
        radius = max(edge.left_m, edge.right_m) + border_width_m
        bounds_points.extend(
            [
                (start[0] - radius, start[1] - radius),
                (start[0] + radius, start[1] + radius),
                (end[0] - radius, end[1] - radius),
                (end[0] + radius, end[1] + radius),
            ]
        )

    if not valid_edges or not bounds_points:
        raise ValueError("No valid route edges to rasterize")

    min_x = min(p[0] for p in bounds_points) - padding_m
    max_x = max(p[0] for p in bounds_points) + padding_m
    min_y = min(p[1] for p in bounds_points) - padding_m
    max_y = max(p[1] for p in bounds_points) + padding_m

    width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
    height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
    image_top_y = min_y + ((height - 1) * resolution)

    def world_to_pixel(point: Point) -> Tuple[int, int]:
        px = int(round((point[0] - min_x) / resolution))
        py = int(round((image_top_y - point[1]) / resolution))
        return px, py

    image = Image.new("L", (width, height), unknown_value)
    draw = ImageDraw.Draw(image)

    # Draw occupied corridor border first.
    for edge, start, end in valid_edges:
        border_quad = _route_quad_points(
            start,
            end,
            edge.left_m + border_width_m,
            edge.right_m + border_width_m,
        )
        draw.polygon([world_to_pixel(p) for p in border_quad], fill=occupied_value)
        border_radius_px = int(
            math.ceil((max(edge.left_m, edge.right_m) + border_width_m) / resolution)
        )
        if border_radius_px > 0:
            for point in (start, end):
                cx, cy = world_to_pixel(point)
                draw.ellipse(
                    [
                        cx - border_radius_px,
                        cy - border_radius_px,
                        cx + border_radius_px,
                        cy + border_radius_px,
                    ],
                    fill=occupied_value,
                )

    # Draw free-space corridor inside border.
    for edge, start, end in valid_edges:
        free_quad = _route_quad_points(start, end, edge.left_m, edge.right_m)
        draw.polygon([world_to_pixel(p) for p in free_quad], fill=free_value)
        free_radius_px = int(math.ceil(max(edge.left_m, edge.right_m) / resolution))
        if free_radius_px > 0:
            for point in (start, end):
                cx, cy = world_to_pixel(point)
                draw.ellipse(
                    [
                        cx - free_radius_px,
                        cy - free_radius_px,
                        cx + free_radius_px,
                        cy + free_radius_px,
                    ],
                    fill=free_value,
                )

    return RasterResult(image=image, origin=(min_x, min_y), resolution=resolution)


def write_map_yaml(
    yaml_path: str,
    image_path: str,
    raster: RasterResult,
    negate: int = 0,
    occupied_thresh: float = 0.65,
    free_thresh: float = 0.196,
) -> None:
    """Write a Nav2 map YAML next to the generated image."""

    yaml_dir = os.path.dirname(os.path.abspath(yaml_path)) or os.getcwd()
    image_ref = os.path.relpath(os.path.abspath(image_path), yaml_dir)
    data = {
        "image": image_ref,
        "resolution": float(raster.resolution),
        "origin": [float(raster.origin[0]), float(raster.origin[1]), 0.0],
        "negate": int(negate),
        "occupied_thresh": float(occupied_thresh),
        "free_thresh": float(free_thresh),
    }
    with open(yaml_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)


def generate_map_image(
    tmap_path: str,
    image_path: str,
    white_extension_m: float,
    border_width_m: float = 0.25,
    resolution: float = 0.05,
    padding_m: Optional[float] = None,
    include_node_polygons: bool = False,
    apply_transform: bool = True,
) -> RasterResult:
    """Load a topological map and write the generated PNG image."""

    with open(tmap_path, "r", encoding="utf-8") as stream:
        tmap_data = yaml.safe_load(stream)

    geometry = geometry_from_tmap(tmap_data, apply_transform=apply_transform)
    raster = rasterize_geometry(
        geometry,
        white_extension_m=white_extension_m,
        border_width_m=border_width_m,
        resolution=resolution,
        padding_m=padding_m,
        include_node_polygons=include_node_polygons,
    )
    os.makedirs(os.path.dirname(os.path.abspath(image_path)), exist_ok=True)
    raster.image.save(image_path)
    return raster


def _default_output_path(tmap_path: str) -> str:
    base, _ = os.path.splitext(tmap_path)
    if base.endswith(".tmap3") or base.endswith(".tmap2"):
        base, _ = os.path.splitext(base)
    return f"{base}_topomap_map.png"


def _default_yaml_path(image_path: str) -> str:
    base, _ = os.path.splitext(image_path)
    return f"{base}.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a Nav2 map image from topological map edges. White "
            "pixels mark buffered topomap corridors, black pixels mark "
            "occupied borders, and grey pixels mark unknown outer space."
        )
    )
    parser.add_argument("tmap", help="Path to a .tmap2.yaml or .tmap3.yaml file")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output PNG path. Defaults to '<topomap>_topomap_map.png'.",
    )
    parser.add_argument(
        "--white-extension-m",
        "--edge-buffer-m",
        dest="white_extension_m",
        type=float,
        default=2.0,
        help="Metres of white free space to extend from each topomap edge.",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="Output map resolution in metres per pixel.",
    )
    parser.add_argument(
        "--border-width-m",
        type=float,
        default=0.25,
        help="Metres of black occupied border outside the white topomap corridor.",
    )
    parser.add_argument(
        "--padding-m",
        type=float,
        default=None,
        help=(
            "Extra grey padding around the generated map in metres. "
            "Defaults to the white extension distance."
        ),
    )
    parser.add_argument(
        "--map-yaml",
        default=None,
        help="Output Nav2 map YAML path. Defaults to the PNG path with .yaml.",
    )
    parser.add_argument(
        "--no-map-yaml",
        action="store_true",
        help="Only write the PNG image.",
    )
    parser.add_argument(
        "--include-node-polygons",
        action="store_true",
        help="Also draw topological node influence polygons.",
    )
    parser.add_argument(
        "--no-transform",
        action="store_true",
        help="Keep geometry in the topological frame instead of applying map transformation.",
    )
    return parser


def main(args: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    parsed = parser.parse_args(args)

    image_path = parsed.output or _default_output_path(parsed.tmap)
    yaml_path = parsed.map_yaml or _default_yaml_path(image_path)

    try:
        raster = generate_map_image(
            parsed.tmap,
            image_path,
            white_extension_m=parsed.white_extension_m,
            border_width_m=parsed.border_width_m,
            resolution=parsed.resolution,
            padding_m=parsed.padding_m,
            include_node_polygons=parsed.include_node_polygons,
            apply_transform=not parsed.no_transform,
        )

        if not parsed.no_map_yaml:
            write_map_yaml(yaml_path, image_path, raster)

    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(
        "Wrote {image} ({width}x{height}, resolution={resolution:.3f}, "
        "origin=[{origin_x:.3f}, {origin_y:.3f}, 0.0])".format(
            image=image_path,
            width=raster.image.width,
            height=raster.image.height,
            resolution=raster.resolution,
            origin_x=raster.origin[0],
            origin_y=raster.origin[1],
        )
    )
    if not parsed.no_map_yaml:
        print(f"Wrote {yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
