<!--
DRAFT for review (not committed). Facts to confirm after a rehearsal from a fresh GitHub account:
- how long the first start takes, and whether a larger machine type is worth recommending
- whether the notification for port 8083 appears and the web app updates live through the Codespaces port proxy
- the address and any password for the noVNC desktop on port 5801
- the free Codespaces allowance wording (GitHub changes it; the guide deliberately gives no numbers)
-->

# Try Topological Navigation in Your Browser

**ROSCon UK 2026, workshop "ROS in the Field"**

This page lets you run the complete topological navigation stack, with a simulated robot on a real farm map, in a GitHub Codespace. You need a web browser and a free GitHub account. Nothing is installed on your own machine, and no robot is needed.

You will:

1. start a Codespace from the `roscon26` branch of this repository,
2. launch the stack with one command, and
3. click on the map in your browser to send the robot to a node.

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/LCAS/topological_navigation/tree/roscon26)

Direct link: <https://codespaces.new/LCAS/topological_navigation/tree/roscon26>

Repository and branch: <https://github.com/LCAS/topological_navigation/tree/roscon26>

---

## 1. Start the Codespace

1. Open the link above and sign in to GitHub if asked.
2. Check that the branch shown is **`roscon26`**, then choose **Create new codespace**.
   If you start from the repository page instead, use **Code**, then the **Codespaces** tab, then **...**, then **New with options**, and pick the `roscon26` branch.
3. Wait while the environment is built. **The first start takes several minutes**, because it prepares a ROS 2 Jazzy container with RViz and all dependencies and builds the workspace. Start it before you need it.
4. When VS Code appears in your browser, this page opens automatically. Open a terminal with **Terminal, New Terminal** (or press <kbd>Ctrl</kbd>+<kbd>`</kbd>).

The terminal starts in `/workspace/src` (this repository). ROS 2 and the pre-built workspace are already sourced in every new terminal.

**About cost and tidying up.** Codespaces use your own GitHub account's monthly free allowance; see GitHub's billing page for the current figures. A Codespace stops by itself after a period of inactivity. When you are finished, stop or delete it at <https://github.com/codespaces> so that it does not use up storage or hours.

## 2. Launch the stack

In the terminal:

```bash
ros2 launch topological_navigation topological_navigation.launch.py
```

RViz starts by default and is available on the noVNC desktop at port **5801**. If your Codespace has limited resources (for example, a 2-core machine), you can disable RViz by adding `rviz:=false` to the launch command.

This starts, in order:

| Component | What it does |
|---|---|
| Map manager | Loads the Riseholme farm map (92 nodes) and publishes it |
| Localisation | Works out the current and closest node from the robot pose |
| Fake Nav2 and virtual robot | Stand in for the real navigation stack and robot |
| Topological navigation server | Plans routes over the graph and runs the action on each edge |
| RViz | Visualises the map and robot on the noVNC desktop (port 5801) |
| Web app | Browser interface on port **8083** |

The virtual robot starts on the `Office` node. Leave this terminal running. Stop everything with <kbd>Ctrl</kbd>+<kbd>C</kbd>.

## 3. Open the web app

1. Open the **Ports** tab at the bottom of VS Code (or click **Open in Browser** in the notification that appears when the web app starts).
2. Find the row **Topological Navigation web app** (port 8083) and click the globe icon, or copy the forwarded address into a new browser tab.

Keep the port **private** (the default). The web app has no authentication, so anyone who could open it could move your robot.

## 4. Things to try

- **Explore the map.** Zoom with the mouse wheel, drag to pan, use **Fit map** and **Centre on robot**. Hover over a node to see its pose, its outgoing edges with their actions, and its `properties`. Green nodes have `roboflow.enabled: true`.
- **Send the robot.** Click the node `QC`, then confirm. The planned route turns orange, the edge being traversed turns red, and the dark disc with an arrow is the robot.
- **Go further, then cancel.** Send the robot to `t1-a1-cz` (the far end of a row), and press **Cancel goal** half way.
- **Use the command line.** Open a second terminal and send a goal. The browser follows along:
  ```bash
  ros2 action send_goal /topological_navigation \
      topological_navigation_msgs/action/GotoNode "{target: 'WayPoint10'}"
  ```
- **Open a second browser tab.** Every tab shows the same robot, goal and route, because all state lives in the backend.
- **See RViz.** Open the forwarded port **5801** (labelled `novnc`) to see the remote desktop with RViz.
- **Change the map.** Copy the example map, edit a node property or an edge action, and launch with your copy:
  ```bash
  cp topological_navigation/test/fixtures/riseholme.tmap3.yaml my_map.tmap3.yaml
  ros2 launch topological_navigation topological_navigation.launch.py \
      rviz:=false map_path:=/workspace/src/my_map.tmap3.yaml
  ```
  The edge actions (`row_traversal`, `navigate_through_poses`) and their behaviour trees are defined in `topological_navigation/config/topological_navigation_config.yaml`, which the launch file loads by default (argument `navigation_config_file`).

Useful launch arguments (all optional):

| Argument | Default | Meaning |
|---|---|---|
| `map_path` | Riseholme map [google maps](https://maps.app.goo.gl/Cha3CtPmiAZJW1iX7)| Absolute path to a `.tmap2.yaml` or `.tmap3.yaml` map |
| `rviz` | `true` | Start RViz (shown on the noVNC desktop, port 5801) |
| `web_app` | `true` | Start the web app |
| `web_app_port` | `8083` | Port of the web app |
| `robot_speed` | `1.5` | Simulated speed in m/s |
| `initial_x`, `initial_y`, `initial_yaw` | `25.75`, `2.8`, `0.0` | Where the virtual robot starts (the `Office` node) |

## 5. Edit code and rebuild

The repository is mounted at `/workspace/src`. After changing code, rebuild with the `build` alias and re-source the workspace:

```bash
build
source /workspace/install/setup.bash
```

## Troubleshooting

| Symptom | What to check |
|---|---|
| Nothing appears on port 8083 | Is the launch command still running in a terminal? Look for errors in its output, then reload the page |
| The map shows but clicking a node does nothing | Run `ros2 action list` in a second terminal. `/topological_navigation` should be listed. If not, restart the launch |
| The page looks stale | Reload the browser tab; the live connection reconnects by itself |
| The environment build failed | Choose **Codespaces: View Creation Log** from the command palette (<kbd>F1</kbd>) and send us the end of the log |
| "Port already in use" | An earlier launch is still running. Stop it with <kbd>Ctrl</kbd>+<kbd>C</kbd>, or use `web_app_port:=8090` |

## Prefer to run it locally?

With Docker and VS Code (Dev Containers extension) installed:

```bash
git clone -b roscon26 https://github.com/LCAS/topological_navigation.git
code topological_navigation
```

Then choose **Reopen in Container** and continue from step 2. The web app is forwarded to <http://localhost:8083/>.

## Learn more

- [README.md](README.md): overview, architecture, map format and all launch arguments
- [topological_navigation/doc/WEB_APP.md](topological_navigation/doc/WEB_APP.md): the web app, its HTTP API and parameters
- [topological_navigation/doc/NAVIGATION2.md](topological_navigation/doc/NAVIGATION2.md): Nav2 integration and edge actions
- Publication: <https://onlinelibrary.wiley.com/doi/10.1002/rob.22494>
- Questions and issues: <https://github.com/LCAS/topological_navigation/issues>
