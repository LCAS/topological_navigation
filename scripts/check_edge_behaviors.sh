#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/check_edge_behaviors.sh 7 8 15 16
# Runs route execution in the running topo_nav_sim container and summarizes
# segment selection + cmd_vel behavior buckets (forward / slow / reverse).

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <waypoint1> <waypoint2> [waypoint3 ...]"
  exit 1
fi

CONTAINER_NAME="${CONTAINER_NAME:-topo_nav_sim}"
SAMPLE_SECONDS="${SAMPLE_SECONDS:-16}"
SLOW_THRESHOLD="${SLOW_THRESHOLD:-0.06}"
POS_THRESHOLD="${POS_THRESHOLD:-0.06}"
NEG_THRESHOLD="${NEG_THRESHOLD:--0.01}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running. Start it with docker compose up first."
  exit 1
fi

# Build YAML list: ['7', '8', '15', '16']
WAYPOINTS=""
for wp in "$@"; do
  if [[ -n "${WAYPOINTS}" ]]; then
    WAYPOINTS+=" ,"
  fi
  WAYPOINTS+="'${wp}'"
done

# Grab recent logs baseline so we can print only new segment lines.
BASE_LOGS=$(docker logs "${CONTAINER_NAME}" 2>&1 | wc -l)

TMP_CSV="/tmp/topo_cmd_vel_$$.csv"

# Capture cmd_vel stream while route executes.
docker exec "${CONTAINER_NAME}" bash -lc \
  "timeout ${SAMPLE_SECONDS}s ros2 topic echo /cmd_vel --csv" > "${TMP_CSV}" 2>/dev/null &
CAP_PID=$!

sleep 1

docker exec "${CONTAINER_NAME}" bash -lc \
  "ros2 action send_goal /execute_named_waypoints topological_navigation_msgs/action/ExecuteNamedWaypoints \"{waypoint_names: [${WAYPOINTS}], execute_navigation: true}\"" >/tmp/topo_action_$$.out 2>&1 || true

wait "${CAP_PID}" || true

NEW_SEG_LINES=$(docker logs "${CONTAINER_NAME}" 2>&1 | tail -n +$((BASE_LOGS + 1)) | grep -E "Executing segment|Slow edge traversal failed|Reverse edge traversal failed|Route planned and executed successfully" || true)

echo "=== Segment log lines ==="
if [[ -n "${NEW_SEG_LINES}" ]]; then
  echo "${NEW_SEG_LINES}"
else
  echo "(no new segment lines matched)"
fi

echo
echo "=== cmd_vel summary ==="
if [[ ! -s "${TMP_CSV}" ]]; then
  echo "No cmd_vel samples captured."
  rm -f "${TMP_CSV}" /tmp/topo_action_$$.out
  exit 0
fi

python3 - "$TMP_CSV" "$SLOW_THRESHOLD" "$POS_THRESHOLD" "$NEG_THRESHOLD" << 'PY'
import csv
import math
import sys

path = sys.argv[1]
slow_thr = float(sys.argv[2])
pos_thr = float(sys.argv[3])
neg_thr = float(sys.argv[4])

vals = []
with open(path, newline="", encoding="utf-8") as f:
    reader = csv.reader(f)
    for row in reader:
        if not row:
            continue
        # /cmd_vel --csv columns include linear.x at index 1 for Twist.
        try:
            x = float(row[1])
        except Exception:
            continue
        if not math.isfinite(x):
            continue
        vals.append(x)

if not vals:
    print("No numeric linear.x samples parsed.")
    raise SystemExit(0)

slow = sum(1 for v in vals if 0.0 < v <= slow_thr)
pos = sum(1 for v in vals if v > pos_thr)
neg = sum(1 for v in vals if v < neg_thr)
zeroish = sum(1 for v in vals if abs(v) <= 1e-3)

print(f"samples={len(vals)}")
print(f"max_linear_x={max(vals):.4f}")
print(f"min_linear_x={min(vals):.4f}")
print(f"slow_samples(0..{slow_thr:.3f}]={slow}")
print(f"positive_samples(>{pos_thr:.3f})={pos}")
print(f"reverse_samples(<{neg_thr:.3f})={neg}")
print(f"near_zero_samples={zeroish}")
PY

rm -f "${TMP_CSV}" /tmp/topo_action_$$.out
