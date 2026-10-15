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

### Alternative: run it locally in VS Code

If you prefer not to use Codespaces, you can run the same environment in VS Code on your own machine.

**You need:** [Docker](https://docs.docker.com/get-docker/) (running), [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers), and Git.

1. Clone the `roscon26` branch and open it in VS Code:
   ```bash
   git clone -b roscon26 https://github.com/LCAS/topological_navigation.git
   code topological_navigation
   ```
2. When VS Code offers **Reopen in Container**, accept. Otherwise press <kbd>F1</kbd> and run **Dev Containers: Reopen in Container**.
3. Wait for the container to build. **The first build takes several minutes**, and later starts are much faster.
4. Open a terminal in VS Code (**Terminal, New Terminal**). It runs inside the container, in `/workspace/src`, with ROS 2 sourced.
5. Continue with step 2 below. The ports are forwarded to your machine:
   - web app: <http://localhost:8083/>
   - RViz desktop (novnc): forwarded port **5801**; open it from the **Ports** tab if the address does not work directly.

If a forwarded port does not respond, remove and re-add it in the **Ports** tab as described under Troubleshooting.

Alternatively, to use your local VS Code with a Codespace that is already running, install the [GitHub Codespaces extension](https://marketplace.visualstudio.com/items?itemName=GitHub.codespaces), press <kbd>F1</kbd>, run **Codespaces: Connect to Codespace**, and pick it. The ports are then forwarded to `localhost` as well.

## 2. Launch the stack

In the terminal:

```bash
ros2 launch topological_navigation topological_navigation.launch.py
```

This starts, in order:

| Component | What it does |
|---|---|
| Map manager | Loads the Riseholme farm map (92 nodes) and publishes it |
| Localisation | Works out the current and closest node from the robot pose |
| Fake Nav2 and virtual robot | Stand in for the real navigation stack and robot |
| Topological navigation server | Plans routes over the graph and runs the action on each edge |
| Web app | Browser interface on port **8083** |
| RViz | Visualisation, shown on the remote desktop on port **5801** |

The virtual robot starts on the `Office` node. Leave this terminal running. Stop everything with <kbd>Ctrl</kbd>+<kbd>C</kbd>.

## 3. Open the web app and RViz

Open the **Ports** tab at the bottom of VS Code. Two ports are forwarded:

| Port | Label | What it shows |
|---|---|---|
| **8083** | Topological Navigation web app | The browser interface: the map, the robot, and the controls to send goals |
| **5801** | novnc | A remote desktop in your browser, showing RViz with the map, the robot and the planned route |

For each, click the globe icon (**Open in Browser**) in its row, or copy the forwarded address into a new browser tab. The notification that appears when the stack starts offers the same.

Keep both ports **private** (the default). The web app has no authentication, so anyone who could open it could move your robot.

## 4. Things to try

- **Explore the map.** Zoom with the mouse wheel, drag to pan, use **Fit map** and **Centre on robot**. Hover over a node to see its pose, its outgoing edges with their actions, and its `properties`. Compare the properties of different nodes and edges to see what metadata the map carries.
- **Send the robot.** Click the node `QC`, then confirm. The planned route turns orange, the edge being traversed turns red, and the dark disc with an arrow is the robot.
- **Go further, then cancel.** Send the robot to `t1-a1-cz` (the far end of a row), and press **Cancel goal** half way.
- **Use the command line.** Open a second terminal and send a goal. The browser follows along:
  ```bash
  ros2 action send_goal /topological_navigation \
      topological_navigation_msgs/action/GotoNode "{target: 'WayPoint10'}"
  ```
- **Open a second browser tab.** Every tab shows the same robot, goal and route, because all state lives in the backend.
- **Watch RViz alongside the web app.** Open the forwarded port **5801** (labelled `novnc`) in a second tab to see the remote desktop with RViz, and compare it with the web app while the robot moves.
- **Change the map.** Copy the example map, edit a node property or an edge action, and launch with your copy:
  ```bash
  cp topological_navigation/test/fixtures/riseholme.tmap3.yaml my_map.tmap3.yaml
  ros2 launch topological_navigation topological_navigation.launch.py \
      map_path:=/workspace/src/my_map.tmap3.yaml
  ```
  The edge actions (`row_traversal`, `navigate_through_poses`) and their behaviour trees are defined in `topological_navigation/config/topological_navigation_config.yaml`, which the launch file loads by default (argument `navigation_config_file`).

Useful launch arguments (all optional):

| Argument | Default | Meaning |
|---|---|---|
| `map_path` | Riseholme map [google maps](https://maps.app.goo.gl/Cha3CtPmiAZJW1iX7)| Absolute path to a `.tmap2.yaml` or `.tmap3.yaml` map |
| `rviz` | `true` | Start RViz (shown on the noVNC desktop, port 5801); use `rviz:=false` to switch it off |
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
| Port 8083 or 5801 shows **502 Bad Gateway** (or another error) | See "Forwarded ports show a 502 error" below |
| Nothing appears on port 8083 | Is the launch command still running in a terminal? Look for errors in its output, then reload the page |
| RViz does not appear on port 5801 | Check that the stack was launched without `rviz:=false`, then reload the noVNC tab |
| The map shows but clicking a node does nothing | Run `ros2 action list` in a second terminal. `/topological_navigation` should be listed. If not, restart the launch |
| The page looks stale | Reload the browser tab; the live connection reconnects by itself |
| The environment build failed | Choose **Codespaces: View Creation Log** from the command palette (<kbd>F1</kbd>) and send us the end of the log |
| "Port already in use" | An earlier launch is still running. Stop it with <kbd>Ctrl</kbd>+<kbd>C</kbd>, or use `web_app_port:=8090` |

### Forwarded ports show a 502 error

If the page for port **8083** (web app) or **5801** (novnc, the RViz desktop) shows **502 Bad Gateway**, the port forwarding has gone stale. Remove and re-add the ports:

1. Make sure the launch command from step 2 is running in a terminal.
2. Open the **Ports** tab, right-click each of the rows for 8083 and 5801, and choose **Stop Forwarding Port** (or click the bin icon).
3. Click **Forward a Port**, type `8083`, and press <kbd>Enter</kbd>. Repeat for `5801`.
4. Open each port again with the globe icon. Optionally set the labels back to `Topological Navigation web app` (8083) and `novnc` (5801) with right-click, **Set Port Label**.
5. Keep both ports **private**. If a 502 persists on 5801, right-click the port, choose **Change Port Protocol**, and try **HTTPS** instead of **HTTP** (or the other way round).

## Learn more

- [README.md](README.md): overview, architecture, map format and all launch arguments
- [topological_navigation/doc/WEB_APP.md](topological_navigation/doc/WEB_APP.md): the web app, its HTTP API and parameters
- [topological_navigation/doc/NAVIGATION2.md](topological_navigation/doc/NAVIGATION2.md): Nav2 integration and edge actions
- Publication: <https://onlinelibrary.wiley.com/doi/10.1002/rob.22494>
- Questions and issues: <https://github.com/LCAS/topological_navigation/issues>
