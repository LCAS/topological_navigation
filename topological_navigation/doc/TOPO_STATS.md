# Topological Navigation Statistics CLI

`topo_stats.py` is a stand-alone command-line tool for inspecting the
SQLite database of stored topological maps and recorded traversal
statistics produced by the navigation server (`nav_stats_db.py`). Like
`map_analyser.py`, it requires no ROS 2 runtime -- it only depends on
`pyyaml`, `networkx` (via `networkx_utils.py`) and, for the `coverage`
command's node filters, the helpers shared with `map_analyser.py` -- so
it can be run locally or in CI.

```bash
ros2 run topological_navigation topo_stats.py /data/nav_stats.db <command> [sub-command] [options]
# or, without ROS 2:
python3 topological_navigation/topological_navigation/scripts/topo_stats.py \
    /data/nav_stats.db <command> [sub-command] [options]
```

There are three top-level commands: [`map`](#map-command),
[`traversals`](#traversals-command) and [`coverage`](#coverage-command).

## `map` command

Operations on the `topological_maps` table (each row is one version of
a map, keyed by a short SHA-1 hash of its YAML content).

| Sub-command | Description |
|-------------|-------------|
| `list` | List all stored maps as a Markdown table. |
| `show <id>` | Show metadata (name, hash, GPS origin, added_at) for one map. |
| `export <id>` | Print the stored map's full YAML to stdout. |
| `import <file>` | Import a `.tmap2.yaml` file into the database. |
| `rm <id>` | Delete a stored map. |
| `stats <id>` | Road-network statistics (node/edge counts, total length, bounding-box area). |
| `merge <db> [<db> ...]` | Merge one or more other database files' maps and traversals into this database. |

`<id>` is either a map's `map_name` or its `map_hash`.

```bash
topo_stats.py nav_stats.db map list
topo_stats.py nav_stats.db map show my_field_map
topo_stats.py nav_stats.db map export my_field_map > my_field_map.tmap2.yaml
topo_stats.py nav_stats.db map import /path/to/map.yaml
topo_stats.py nav_stats.db map rm my_field_map
topo_stats.py nav_stats.db map stats my_field_map
```

### `map merge` sub-command

Each run of the navigation system typically records its traversals
into its own database file. Before computing coverage across multiple
runs (see [`coverage`](#coverage-command) below), merge all the run
databases into one:

```bash
topo_stats.py site.db map merge run2.db run3.db run4.db
```

Merging is idempotent for maps: a map already present in the target
database (matched by `map_hash`) is skipped, so merging the same
source database more than once does not duplicate map rows. Traversal
records are always appended (each historical traversal is a distinct
event), so merging the same source twice **will** duplicate its
traversal rows -- only merge each run's database once.

## `traversals` command

Analyses the `traversals` table recorded by the navigation server for
a **single** map.

| Sub-command | Description |
|-------------|-------------|
| `summary` | Overview of traversal counts per map (Markdown table). |
| `edge_stats <map_id>` | Per-edge statistics for one map (YAML output). |
| `map_stats <map_id>` | Whole-map traversal statistics, with optional top-N failure/success/aborted edge lists (YAML output). |

All three accept an optional `--filter "SQL_EXPR"` applied as a SQL
`WHERE` expression against the `traversals` table, e.g.:

```bash
topo_stats.py nav_stats.db traversals summary --filter "start_time > '2024-01-01'"
topo_stats.py nav_stats.db traversals edge_stats my_field_map --filter "status = 'success'"
topo_stats.py nav_stats.db traversals map_stats my_field_map \
    --filter "start_time BETWEEN '2024-01-01' AND '2024-06-01'" \
    --topn_failures=5 --topn_success=5 --topn_aborted=5
```

## `coverage` command

Computes **route sign-off coverage**: for a set of selected edges
(across one or more maps), how many have at least a configurable
minimum number of recorded *successful* traversals ("covered"), and
how many traversals each edge has recorded in total. This supports a
mapping/deployment QA workflow where a site is only signed off once a
sufficient fraction of its routes have been demonstrably (and
repeatedly) tested.

| Sub-command | Description |
|-------------|-------------|
| `summary` | Brief Markdown coverage summary (overall + per-tag). |
| `report` | Comprehensive Markdown sign-off report (overall/per-tag/per-edge tables, plus a list of uncovered edges), to stdout or a file. |
| `svg` | Full-map SVG, edges colour-coded by coverage. |

### Selecting maps

Every `coverage` sub-command requires exactly one of:

| Switch | Meaning |
|--------|---------|
| `-m, --map MAP_ID` | Include this map (repeatable, name or hash). |
| `-a, --all` | Include every map stored in the database. |

When more than one map is selected, their graphs are merged first:
nodes and edges that share the same name/`edge_id` across maps are
treated as a single node/edge for coverage purposes (e.g. a shared
headland or connecting route recorded once in each of several
per-field databases is not double-counted).

### Selecting nodes (and their incident edges)

By default every edge in the selected map(s) is included in the
report. To restrict the report to a subset -- e.g. only the rows of
one crop, or only nodes tagged for a particular semantic -- pass one
or more `--filter`/`--exclude` node selectors, using the same DSL as
`map_analyser.py`'s `--grid-angle-filter`:

* `name:<glob>` -- node name matches a shell-style wildcard, e.g. `name:Row*`.
* `tag:<glob>` -- any of the node's `meta.tag` entries matches a
  shell-style wildcard, e.g. `tag:row_entry` or `tag:vineyard_*`.
* `property:<key>[=<value>]` -- the node's `properties` dict has
  (optionally, a specific value at) the given (dotted) key, e.g.
  `property:roboflow.enabled` or `property:field=1`.

Each `--filter`/`--exclude` value can combine several of the terms
above with `and`, `or`, `not` (case-insensitive) and parentheses to
express arbitrary logical combinations, e.g. select nodes that have
roboflow enabled *and* are in field 1 or field 2:

```
property:roboflow.enabled and (property:field=1 or property:field=2)
```

Operator precedence, from lowest to highest, is `or`, `and`, `not`;
use parentheses to override it. A `--filter`/`--exclude` value with no
operators behaves exactly like a single term.

A node is selected if it matches any `--filter` (or if no `--filter`
is given, every node is selected), minus any node matched by
`--exclude`. **An edge is selected -- and therefore counted towards
coverage -- if either of its endpoint nodes is selected**, so
filtering by, say, `tag:row_entry` also pulls in the edges leaving the
row-entry nodes towards their neighbours, not just edges between two
row-entry nodes.

An additional `--sql-filter "SQL_EXPR"` restricts which traversal rows
count towards coverage (e.g. a time window), analogous to
`traversals`' `--filter`.

### Sign-off threshold (`--min-success`)

An edge only counts as **covered** once it has at least `--min-success`
recorded successful traversals (default: **2**). This threshold is
configurable per invocation with `--min-success N` on every `coverage`
sub-command (`summary`, `report` and `svg`), and applies consistently
to the covered/uncovered counts, the per-tag breakdown, the "Uncovered
Edges" list and the SVG colour-coding (an edge with at least one, but
fewer than `--min-success`, successes is drawn amber rather than
green -- see the `coverage svg` colour table below).

```bash
# Require at least 3 successful traversals before an edge counts as signed off
topo_stats.py site.db coverage report -a --min-success 3 -o signoff.md

# A single successful traversal is enough (more permissive than the default)
topo_stats.py site.db coverage summary -a --min-success 1
```

```bash
# Coverage across every stored map, filtered to nodes tagged row_entry
topo_stats.py site.db coverage summary -a --filter "tag:row_entry"

# Sign-off report for two named maps, excluding a decommissioned row
topo_stats.py site.db coverage report -m field_a -m field_b \
    --filter "name:Row*" --exclude "name:RowZ*" -o signoff.md

# Sign-off report restricted to nodes with roboflow enabled in field 1 or 2
topo_stats.py site.db coverage report -a \
    --filter "property:roboflow.enabled and (property:field=1 or property:field=2)" \
    -o field_1_2_signoff.md

# Coverage for tunnel 17, but only where roboflow is enabled
topo_stats.py site.db coverage summary -a \
    --filter "property:tunnel=17 and property:roboflow.enabled"

# SVG for a single map, restricted to a time window
topo_stats.py site.db coverage svg -m field_a -o coverage.svg \
    --sql-filter "start_time > '2024-06-01'"
```

### `coverage summary`

Prints a short Markdown table to stdout: total/covered/uncovered
selected edges, overall coverage percentage, traversal/outcome
counts, and a per-tag breakdown (one row per distinct `meta.tag` value
found on any selected node).

### `coverage report`

A comprehensive Markdown report suitable as sign-off evidence,
containing:

1. An **Overall Coverage** table (same metrics as `summary`, plus
   traversal counts split by outcome).
2. A **Coverage by Tag** table.
3. A **Per-Edge Detail** table listing every selected edge's id,
   endpoints, action, traversal/outcome counts and covered status.
4. An **Uncovered Edges** list naming every selected edge with fewer
   than `--min-success` recorded successful traversals -- the concrete
   list of routes that still need to be driven before sign-off.

Pass `-o/--output FILE` to write the report to a file instead of
stdout.

### `coverage svg`

Renders the full merged map as an SVG (`-o/--output FILE` is
required), colour-coding edges by coverage:

| Colour | Meaning |
|--------|---------|
| Green | Selected edge with at least `--min-success` (default 2) successful traversals -- signed off. |
| Amber | Selected edge with at least one, but fewer than `--min-success`, successful traversals. |
| Red | Selected edge with zero successful traversals. |
| Grey | Edge (or node) not selected by the `--filter`/`--exclude` node selectors. |

Every node and edge in the map is still drawn -- unselected ones are
simply faded to grey rather than omitted -- so the SVG always shows
the whole map in context, with a legend explaining the colour scale.
An optional `--title "..."` is rendered at the top of the image.

## Running without ROS 2

None of `topo_stats.py`, `nav_stats_db.py` or `coverage_analysis.py`
import any ROS 2 packages, so the `ros2 run topological_navigation
topo_stats.py ...` prefix above can be replaced with a plain `python3
.../topo_stats.py ...` call in a venv or CI job that has no ROS 2
installation, exactly as described for `map_analyser.py` in
[MAP_ANALYSER.md](MAP_ANALYSER.md#running-without-ros-2).
