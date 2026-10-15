# Topological Map Analyser

`map_analyser.py` is a stand-alone command-line tool for analysing
`.tmap2.yaml` topological map files. It requires no ROS 2 runtime and
only depends on `pyyaml`, `jsonschema`, `networkx` and (for the `merge`
subcommand) `pyproj`, so it can be run locally or in a CI pipeline
(e.g. a GitHub Actions workflow).

## What it checks

* **Schema compliance** – validates the map against
  `config/tmap-schema.yaml` (delegates to `validate_map.py`).
* **Orphaned nodes** – nodes with no incoming edges, i.e. nodes that
  can never be reached by navigating the graph.
* **Disconnected sub-maps** – weakly connected components of the
  topological graph. More than one component means the map contains
  islands of nodes that cannot reach each other.
* **Map statistics** – node/edge counts, total and average edge
  length, a breakdown of edge counts per action, and the number of
  bidirectional vs. unidirectional edges.
* **Overlapping influence zones** – pairs of nodes whose influence
  zone polygons (`verts`) overlap, including the case where a node's
  position falls inside another node's polygon.
* **Grid angle deviation** *(disabled by default)* – for a configurable
  set of nodes, checks that each node's own incoming/outgoing edges are
  mutually separated by multiples of 90 degrees (i.e. the node looks
  like a right-angle grid corner/cross/T, regardless of how the node's
  local neighbourhood is oriented relative to the rest of the map).
  Edges whose bearing deviates from the node's best-fit local
  right-angle pattern by more than a configurable threshold (default 5
  degrees) are flagged. See
  [Grid angle deviation check](#grid-angle-deviation-check) below.
* **SVG rendering** – a full-map SVG image with nodes as circles,
  edges colour-coded by action, bidirectional edges drawn as plain
  lines (no arrow head), and unidirectional edges drawn with an arrow
  head indicating direction of travel.
* **Minification** – writes a smaller, semantically-identical copy of
  the map by collapsing repeated data structures (e.g. shared
  `properties` or `verts` blocks) into named YAML anchors, and
  optionally stripping nodes/edges unreachable from a given node.
* **Merging** – combines two or more maps into a single schema-valid
  map, reprojecting node positions into the first map's GPS origin,
  deduplicating node/edge names, and merging metadata (see
  [`merge` command](#merge-command) below).
* **Grid alignment** – in a separate mode (the `grid-align` command,
  as opposed to a `check`), the tool can reposition the nodes selected
  for the grid angle deviation check so that their own incident edges
  become mutually right-angled, and save the adjusted map (see
  [`grid-align` command](#grid-align-command) below).

## Usage

```bash
# Full human-readable analysis report
ros2 run topological_navigation map_analyser.py analyse my_map.tmap2.yaml

# Full report + SVG rendering
ros2 run topological_navigation map_analyser.py analyse my_map.tmap2.yaml --svg my_map.svg

# CI-friendly validity check (exit code reflects the result)
ros2 run topological_navigation map_analyser.py check my_map.tmap2.yaml

# Treat disconnected sub-maps as an error, and skip the influence-zone check
ros2 run topological_navigation map_analyser.py check my_map.tmap2.yaml \
  --sub-map-separation=error --influence-zone-overlap=false

# SVG rendering only
ros2 run topological_navigation map_analyser.py svg my_map.tmap2.yaml -o my_map.svg

# Minify (writes my_map.min.tmap2.yaml alongside the input by default)
ros2 run topological_navigation map_analyser.py minify my_map.tmap2.yaml

# Merge two or more maps into one (writes map_a.merged.tmap2.yaml by default)
ros2 run topological_navigation map_analyser.py merge map_a.tmap2.yaml map_b.tmap2.yaml -o merged.tmap2.yaml

# Check that nodes named Row* have mutually right-angled edges
ros2 run topological_navigation map_analyser.py check my_map.tmap2.yaml \
  --grid-angle-deviation=error --grid-angle-filter "name:Row*"

# Reposition the nodes matched by the filter to minimise the deviation of
# their own edges from a 90 degree multiple of one another
# (writes my_map.gridalign.tmap2.yaml alongside the input by default)
ros2 run topological_navigation map_analyser.py grid-align my_map.tmap2.yaml \
  --grid-angle-filter "name:Row*"
```

## Running without ROS 2

`map_analyser.py` (and the `map_merger.py`/`validate_map.py`/
`convert_tmap.py` modules it builds on) have no ROS 2 runtime
dependency — they only import pure-Python packages, so the whole
`ros2 run topological_navigation map_analyser.py ...` prefix above can
be replaced with a plain `python3 .../map_analyser.py ...` call, with
no ROS 2 workspace build or `source install/setup.bash` required. This
is useful for running the tool in a plain `venv`, in CI (see
`.github/workflows/map-analyser.yaml`), or on a machine that doesn't
have ROS 2 installed at all.

```bash
# From the repository root
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Then call the script directly, with the same subcommands/options as above
python3 topological_navigation/topological_navigation/map_analyser.py analyse my_map.tmap2.yaml
python3 topological_navigation/topological_navigation/map_analyser.py check my_map.tmap2.yaml
python3 topological_navigation/topological_navigation/map_analyser.py merge map_a.tmap2.yaml map_b.tmap2.yaml -o merged.tmap2.yaml
```

The only dependencies are `pyyaml`, `jsonschema`, `networkx` and (for
the `merge` subcommand) `pyproj`, all listed in
[requirements.txt](../../requirements.txt) at the repository root.

## Configuring check severities

Each of the checks can be independently turned off, or have its
severity set to `warning` or `error`, via a `--<check>={false,warning,error}`
switch on the `analyse`/`check` subcommands:

| Switch | Check | Default severity |
|--------|-------|-------------------|
| `--schema-check` | Schema compliance | `error` |
| `--orphaned-node` | Orphaned nodes | `error` |
| `--sub-map-separation` | Disconnected sub-maps | `warning` |
| `--influence-zone-overlap` | Overlapping influence zones | `warning` |
| `--grid-angle-deviation` | Grid angle deviation | disabled (`false`) |

* `false` (also accepted: `off`, `disable`, `disabled`, `none`) disables
  the check entirely: it is neither run nor printed in the report.
* `warning` runs the check and prints its result, but a failure does not
  affect the `check` command's exit code.
* `error` (also accepted: `true`, `on`) runs the check and a failure
  causes `check` to exit with code `1`.

Example: fail CI only on schema errors and orphaned nodes, ignore
disconnected sub-maps and influence zone overlaps entirely:

```bash
map_analyser.py check my_map.tmap2.yaml \
  --sub-map-separation=false --influence-zone-overlap=false
```

## Grid angle deviation check

Many topological maps are expected to have nodes that look locally
like right-angle grid corners/crosses/T-junctions, e.g. rows in a
field, aisles in a warehouse — even if the whole layout is rotated
relative to the map's x/y axes, or different areas of the map are
rotated relative to one another. The grid-angle-deviation check looks
at the bearing of every edge incident to a node (both incoming and
outgoing) and flags any edge whose direction deviates by more than a
threshold from the *node's own* best-fit right-angle pattern (i.e. the
angle **between** that node's edges), since such edges often indicate
a mapping mistake. It does **not** compare edge bearings to the map's
global 0/90/180/270 degree axes: a node whose edges point at, say,
30/120/210/300 degrees passes the check just as well as one whose
edges point at 0/90/180/270, because in both cases every edge is a
90-degree multiple away from every other edge at that node. A node
needs at least two incident edges for there to be an angle between
edges to check; nodes with zero or one incident edge are never
flagged.

This check is **disabled by default**, since not every map (or every
node in a map) is expected to have right-angled edges. Enable it with
`--grid-angle-deviation=warning` (report only) or
`--grid-angle-deviation=error` (also fail `check`), and restrict which
nodes it applies to with one or more `--grid-angle-filter`/
`--grid-angle-exclude` switches:

| Switch | Meaning | Default |
|--------|---------|---------|
| `--grid-angle-threshold DEG` | Maximum allowed deviation (degrees) of an edge from the node's own best-fit 90 degree multiple pattern before it is flagged | `5.0` |
| `--grid-angle-filter FILTER` | Select nodes expected to have mutually right-angled edges. Repeatable; the selected set is the union (OR) of every match | every node |
| `--grid-angle-exclude FILTER` | Remove nodes matching `FILTER` from the selected set (applied after all `--grid-angle-filter` matches are unioned). Repeatable | none |

Both `--grid-angle-filter` and `--grid-angle-exclude` accept the same
`FILTER` syntax, which is a single term or a boolean expression
combining several terms:

* `name:<glob>` – matches node names against a shell-style wildcard
  pattern (`*`, `?`, `[seq]`), e.g. `name:Row*` or `name:Junction?`.
* `tag:<glob>` – matches any of the node's `meta.tag` entries against
  a shell-style wildcard pattern, e.g. `tag:row_entry`.
* `property:<key>` – matches nodes whose `properties` dict has a
  truthy value at `<key>` (dotted for nested keys, e.g.
  `property:roboflow.enabled`).
* `property:<key>=<value>` – matches nodes whose `properties` dict has
  a value at `<key>` that case-insensitively equals `<value>` as a
  string, e.g. `property:semantics=row_entry` or `property:field=1`.

Terms can be combined with `and`, `or`, `not` (case-insensitive) and
parentheses to express arbitrary logical combinations, e.g. select
every node with roboflow enabled that is in field 1 or field 2:

```
property:roboflow.enabled and (property:field=1 or property:field=2)
```

Operator precedence, from lowest to highest, is `or`, `and`, `not`;
use parentheses to override it. A `FILTER` with no operators behaves
exactly like a single term as before.

If no `--grid-angle-filter` is given, every node in the map is
selected (minus any `--grid-angle-exclude` matches). Multiple
`--grid-angle-filter` switches are combined with OR (disjunction); the
final selected set is that disjunction minus every node matched by any
`--grid-angle-exclude`.

```bash
# Only check nodes whose name starts with "Row", ignoring the parking bay
map_analyser.py check my_map.tmap2.yaml --grid-angle-deviation=error \
  --grid-angle-filter "name:Row*" --grid-angle-exclude "name:RowParking"

# Only check nodes explicitly tagged as grid-aligned via a property
map_analyser.py check my_map.tmap2.yaml --grid-angle-deviation=error \
  --grid-angle-filter "property:grid_aligned"

# Only check nodes with roboflow enabled that are in field 1 or 2
map_analyser.py check my_map.tmap2.yaml --grid-angle-deviation=error \
  --grid-angle-filter "property:roboflow.enabled and (property:field=1 or property:field=2)"

# Use a looser 10 degree threshold
map_analyser.py check my_map.tmap2.yaml --grid-angle-deviation=warning \
  --grid-angle-threshold 10
```

## `grid-align` command

`grid-align` automatically repositions the nodes selected by
`--grid-angle-filter`/`--grid-angle-exclude` (same syntax as the
grid-angle-deviation check above) to minimise the deviation of their
incident edges from a 90 degree multiple of one another, and writes
the result to a new map file. Nodes not selected by the filters (i.e.
the neighbours used as reference points) are never moved.

```bash
map_analyser.py grid-align my_map.tmap2.yaml
map_analyser.py grid-align my_map.tmap2.yaml -o aligned.tmap2.yaml
map_analyser.py grid-align my_map.tmap2.yaml --grid-angle-filter "name:Row*"
map_analyser.py grid-align my_map.tmap2.yaml --grid-angle-threshold 10
```

If `-o`/`--output` is omitted, the output path is derived from the
input file by inserting `.gridalign` before the extension, e.g.
`my_map.tmap2.yaml` -> `my_map.gridalign.tmap2.yaml`.

For each selected node, the node's own best-fit local right-angle
pattern is first computed from the bearings of all of its own
incident edges (before any node is moved). Every neighbour it shares
an edge with (incoming or outgoing) is then classified as lying
roughly "along" or "across" that local pattern, and the node is moved
onto the average alignment of those neighbours within that same
locally-oriented frame (e.g. a node with one neighbour roughly along
its pattern and one neighbour roughly across it is moved to share
each neighbour's corresponding coordinate in that frame). This works
the same way regardless of how the node's local neighbourhood happens
to be rotated relative to the rest of the map — it never snaps nodes
onto the map's global x/y axes. Neighbouring nodes themselves are
never moved by this process, even if they are also part of the
selected set: each selected node is repositioned independently, in a
single pass, based on the original (pre-alignment) positions of all
its neighbours.

If, after being repositioned, a node still has at least one incident
edge outside the angle threshold (e.g. because its neighbours are not
themselves consistently aligned), the node is left at its **original**
position and reported as **unresolved**; these nodes are listed in the
report and the command exits with code `1` (the output map is still
written with whatever adjustments could be made to the other nodes).

```bash
$ map_analyser.py grid-align my_map.tmap2.yaml --grid-angle-filter "name:Row*"
Grid-align report: my_map.tmap2.yaml -> my_map.gridalign.tmap2.yaml
============================================================
  Angle threshold: 5 deg
  Nodes adjusted:  2
    - Row1Start: (1.200, 0.950) -> (1.200, 1.000) [max deviation 4.2 deg -> 0.0 deg]
    - Row1End: (11.100, 1.050) -> (11.100, 1.000) [max deviation 3.1 deg -> 0.0 deg]
  Unresolved nodes: 1 (could not be aligned):
    - Row2End
  Schema check on output: PASS: Validation successful
```

`grid-align` shares the `--schema`/`-s` switch with the other commands
(to validate against a non-default schema file) and, like `minify`,
re-parses and schema-validates its own output before reporting
success.

## `minify` command

Topological maps often contain many repeated data structures (e.g. the
same `properties` dict or influence-zone `verts` polygon reused across
dozens of nodes/edges). `minify` rewrites the map into an equivalent but
smaller file by finding these repeated subtrees and replacing them with
YAML anchors/aliases, collected under a top-level `__yaml_anchors:` key
(configurable via `--anchors-key`).

```bash
map_analyser.py minify my_map.tmap2.yaml
map_analyser.py minify my_map.tmap2.yaml -o compact.tmap2.yaml
map_analyser.py minify my_map.tmap2.yaml --strip-unreachable Charging
```

If `-o`/`--output` is omitted, the output path is derived from the
input file by inserting `.min` before the extension, e.g.
`my_map.tmap2.yaml` -> `my_map.min.tmap2.yaml`.

Anchor names are chosen from the map's schema/content (e.g.
`properties_row_entry`, `verts_2x2`) rather than being generic
auto-generated ids, so the minified file stays human-readable.

| Switch | Meaning | Default |
|--------|---------|---------|
| `--anchors` / `--no-anchors` | Collapse repeated subtrees into named anchors | enabled |
| `--anchors-key KEY` | Top-level key name to collect anchors under | `__yaml_anchors` |
| `--strip-comments` | Drop the file's leading comment block | disabled (comments kept) |
| `--flowstyle` | Emit compact flow-style YAML instead of block style | disabled |
| `--min-size N` | Minimum serialised size (chars) of a subtree to qualify for anchoring | `100` |
| `--min-occurrences N` | Minimum number of repeats required for a subtree to qualify for anchoring | `5` |
| `--strip-unreachable NODE` | Also drop nodes/edges not reachable via a directed path from `NODE` | disabled |

Notes:

* Only the leading top-of-file comment block is preserved; inline/
  per-node comments are not (the tool re-serialises the map from its
  parsed data structure, like the rest of this codebase).
* `--strip-unreachable NODE` computes the directed descendants of
  `NODE` (plus `NODE` itself) and keeps only those. This guarantees
  the result has no orphaned nodes (other than possibly `NODE` itself,
  if nothing points back to it) and is a single connected sub-map, so
  it always passes the `orphaned-node` and `sub-map-separation` checks.
* The tool re-parses its own output and schema-validates the written file, to catch any minification bug; it logs a summary of the size savings achieved.

## `merge` command

Combines two or more `.tmap2.yaml` maps into a single schema-valid map,
for example when several fields/sites were mapped independently and
need to be navigated as one graph.

```bash
map_analyser.py merge map_a.tmap2.yaml map_b.tmap2.yaml
map_analyser.py merge map_a.tmap2.yaml map_b.tmap2.yaml -o merged.tmap2.yaml
map_analyser.py merge map_a.tmap2.yaml map_b.tmap2.yaml map_c.tmap2.yaml --name my_merged_map
map_analyser.py merge map_a.tmap2.yaml map_b.tmap2.yaml --connect-closest
```

If `-o`/`--output` is omitted, the output path is derived from the
first input file by inserting `.merged` before the extension, e.g.
`map_a.tmap2.yaml` -> `map_a.merged.tmap2.yaml`.

Merge rules:

* **Reference origin** \u2013 the first map's `meta.origin` (GPS
  latitude/longitude/altitude) is used as the reference frame. Every
  other map's nodes are reprojected (via `pyproj`, using an azimuthal
  equidistant projection centred on the reference point) into metres
  offset from that origin, and shifted accordingly. If a map has no
  real GPS origin (missing, or `latitude`/`longitude` both `0`), its
  positions are merged unchanged and a warning is recorded; if the
  *reference* map itself has no real origin, reprojection is skipped
  for all maps.
* **Node/edge name deduplication** \u2013 node names and edge IDs must be
  globally unique in the merged map. Colliding names from the second
  and subsequent maps are renamed with a numeric suffix (`WP1` ->
  `WP1_2` -> `WP1_3`, ...), a warning is recorded for each rename, and
  every edge referencing a renamed node is updated to point at the new
  name.
* **Connecting the maps** – by default, each input map remains its own
  disconnected sub-map in the merged graph (see `--sub-map-separation`
  above). Pass `--connect-closest` to automatically link every map to
  the growing merged map with a new bidirectional edge between their
  closest pair of nodes (straight-line distance in the merged/
  reprojected frame), producing a single connected graph. The action
  and action type used for these edges default to `navigate_to_pose` /
  `nav2_msgs/action/NavigateToPose` and can be overridden with
  `--connect-action` / `--connect-action-type`. Connecting edge IDs are
  named `connect_<node_a>_<node_b>`; the merge report lists every
  connecting edge under `[Connecting edges]` with the distance covered.
* **Metadata merge** – top-level keys (`name`, `metric_map`,
  `pointset`, `transformation`, `definitions`, `actions`,
  `navigation_config_file`) and `meta` subkeys use
  first-map-precedence: the first map's value is kept and a warning is
  recorded if a later map defines a conflicting value. Two exceptions:
  `meta.origin` always comes from the reference map, and
  `meta.fields` (field/boundary definitions) are concatenated across
  all maps with `field_number` renumbered sequentially to avoid
  collisions. `meta.last_updated` is always regenerated.
* **Schema validity** \u2013 the merged output is always re-validated
  against `config/tmap-schema.yaml` before the command reports
  success; the merge report includes a `[Schema validation of merged
  output]` section.

The `--name` argument overrides the merged map's
`name`/`metric_map`/`pointset` fields (otherwise the first map's values
are kept, per the metadata merge rule above).

## `check` command exit codes

The `check` command is designed for use in a GitHub Actions workflow:

| Exit code | Meaning |
|-----------|---------|
| `0` | Map is valid |
| `1` | Map is invalid |
| `2` | Map file not found, or another error occurred while loading it |

With the default severities, a map is considered **valid** if:

1. It is compliant with the JSON schema.
2. It has no orphaned nodes (nodes unreachable via any incoming edge).
3. No two influence zones overlap, and no node's position falls
   inside another node's influence zone.

Disconnected sub-maps are **not** treated as a validity failure: they
are printed as a warning in the report, since it is often expected for
a map to contain isolated auxiliary nodes (e.g. no-go zones, topic
localisation markers) that are not part of the main navigable graph.

## Example GitHub workflow

See `.github/workflows/map-analyser.yaml` for a workflow that runs
`map_analyser.py check` against every `.tmap2.yaml`/fixture map in the
repository and uploads the generated SVG files as a workflow artefact.
