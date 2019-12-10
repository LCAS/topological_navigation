# Topological Navigation tests

The topological navigation tests are designed to optimise the parameter set of the DWA planner and to check the functionalitites of topological navigation in its execute policy mode. The test are run automatically on our Jenkins server but can also be run locally on your machine to test a new paremeterset. 

## Run test

Test for your whole worksapce can be run with `catkin_make test` which assumes that `strands_navigation` is present in your ros workspace. If you only want to run the test described here, use `catkin_make test --pkg topological_navigation`. This will only run the critical topological navigation tests. A set of supplementary tests can be run with:

```
roslaunch topological_navigation navigation_scenarios.test
rosrun topological_navigation topological_navigation_tester_supplementary.py
```

To run the testes from an installed version of topological_navigation do:

```
roslaunch topological_navigation navigation_scenarios.test
```

and 

```
rosrun topological_navigation topological_navigation_tester_critical.py
```

for the critical tests or

```
rosrun topological_navigation topological_navigation_tester_supplementary.py
```

for the supplementary tests to test parameter sets.

This will start all the tests and report the result to the terminal and a log file.

## Test Scenarios, and Infrastructure

### Infrastructure

In order to run the tests, the commands above will start the following nodes:

* `strands_morse` using the move_base_arena environment
* `mongodb_store` in test mode, creating a new and empty temporary datacentre
* `scitos_2d_navigation` with the correct slam map for the tests
* A static transform publisher with the transformation from `map` to `world`
* `topological_navigation`:
 * `monitored_navigation` with all recovery behaviours disabled
 * `map_manager.py`
 * `fremenserver`
 * `localisation.py`
 * `navigation.py`
 * `execute_policy_server.py`
 * `visualise_map.py`
 * `travel_time_estimator.py`
 * `topological_prediction.py`
 * `get_simple_policy.py`
* The test scneario server which is responsible for the test control
* The ros test file `topological_navigation_tester.py`

### Test Scenarios

All the test scnearios are based on a simple topological map that has to have a Node called `Start` and a node called `End`. The robot will always be teleported (in simulation) or pushed (on the real robot) to this node and then use policy execution to traverse the edges towards the node called `End`. If the node `End` cannot be reached, the navigation will fail and the scneario_server will trigger the graceful death behaviour, trying to navigate the robot back to the node `Start`.

The topological maps used for the tests presented here can be found in `strands_morse/mba/maps` and are loaded into the datacentre (if they haven't already been inserted) by the scanario_server. Currently run tests are:

**Static:**

* System critical:
 * `mb_test0`: Traversing a 2m wide l-shaped corridor.
* Supplementary:
 * `mb_test1`: The robot starting 10cm away from a wall facing it straight on (0 degrees)
 * `mb_test2`: The robot starting 10cm away from a wall facing it turned to the left (-45 degrees)
 * `mb_test3`: The robot starting 10cm away from a wall facing it turned to the right (+45 degrees)
 * `mb_test4`: Traversing a 1.55m wide straight corridor with .55m wide chairs on one side.
 * `mb_test5`: Traversing a 2.1m wide straight corridor with .55m wide chairs on both sides.
 * `mb_test6`: Cross a 80cm wide door.
 * `mb_test7`: Cross a 70cm wide door.
 * `mb_test8`: The robot is trapped in a corner and has to reach a goal behind it.
 * `mb_test9`: Traverse a 1m wide straight corridor.
 * `mb_test10`: Traverse a 1m wide l-shaped corridor.
 * `mb_test11`: Wheelchair blocking an intermediate node in open space.
 * `mb_test12`: Wheelchair blocking final node in open space. Graceful fail has to be tru to pass this test as the node itself can never be reached precisely.
 * `mb_test13`: Human blocking an intermediate node in open space.
 * `mb_test14`: Human blocking final node in open space. Graceful fail has to be tru to pass this test as the node itself can never be reached precisely.
 * `mb_test15`: Non-SLAM map chairs on one side of 2m wide l-shaped corridor.
 * `mb_test16`: Non-SLAM map wheelchairs on one side of 2m wide l-shaped corridor.
 * `mb_test17`: Non-SLAM map wheelchairs block 2m wide l-shaped corridor after intermediate waypoint. Graceful fail has to be tru to pass this test as the node itself can never be reached.
 * `mb_test18`: Non-SLAM map static humans block 2m wide l-shaped corridor after intermediate waypoint. Graceful fail has to be tru to pass this test as the node itself can never be reached.

**Dynamic**

Coming soon

### Scenario Server control

The scenario server can also be used without the unit test file for tests on the real robot. Running

```
roslaunch topological_navigation navigation_scenarios.test robot:=true map_dir:="path_to_topologicalmaps"
```

will start only the scneario server, the simple policy generation, and the joypad control. This assumes that everything on the robot, up to topological navigation, is running. The `map_dir` argument specifies a directory which holds the topological maps that you want to use for testing. If this is given, the maps will automatically inserted into the datacentre. If the maps have been inserted previously, the server will print a warning and skip the insertion. If the `map_dir` argument is omitted, no maps are inserted. The scneario server then offers 3 simple services:

* `/scneario_server/load <map_name>` expects a string which is the name of the topological map you want to test. Keep in mind that this has to have the node `Start` and `End`.
* `/scenario_server/reset` is an empty service and is called to reset the data recording and the robot position. The robot position, however, cannot as easily be changed in real life as it can be in simulation, hence the robot has to be pushed to the starting node and then confirmed via the `A` button on the joypad. The reasoning behind having to push the robot is that the starting position might not be easily reachable via `move_base`. The server will print `+++ Please push the robot to 'Start' +++` where `Start` is the name of the starting node, until the node is reached. Once the node is reached, the server will print `+++ Please confirm correct positioning with A button on joypad: distance 0.65m 180.06deg +++` where distance represents the distance of the current `/robot_pose` to the metric coordinates of the node. In simulation this will just teleport the robot to the correct node.
* `/scneario_server/start` starts the policy execution. Returns

 ```
bool nav_success
bool graceful_fail
bool human_success
float64 min_distance_to_human
float64 mean_speed
float64 distance_travelled
float64 travel_time
 ```

**Joypad control**

For convenience, a joypad control of the `reset` and `start` service is provided. The `load` service still has to be called manually to tell the server which map to use. The joypad control then offers an easy way to interact with the scenario server during the tests:

* `A`: Toggle between `start` and `reset`.
* `B`: Confirm current selection.

If in doubt, press `A` and look for the output on the terminal.

**Creating Topological maps for testing**

The topological map used for each test has to be a different one and the nodes have to conform to a specific naming scheme. By default, the start node has to be called `Start` and the goal has to be called `End`. This can be changed in `topologcial_navigation/tests/conf/scenario_server.yaml`. The easiest way to create these maps is:

1. Start topological navigation with `roslaunch topological_navigation topologocal_navigation_empty_map.launch map:=<map_name>` where `map_name` will be the name of your new map and cannot be the name of an existing one.
1. Drive the robot to the positions of the nodes you want to create and use the `add_rm_node` interactive marker in rviz to create a new node.
1. Use the interactive `edges` marker in rviz to delete unwanted edges.
1. Rename the start and end node to `Start` and `End` using `rosrun topological_utils rename_node <old_name> <new_name> <map_name>`
1. This map can now be loaded with `/scenario_server/load <map_name>`


**Creating scnearios with obstacles that are not in the SLAM map**

For this purpose, there are `Chair`s, `OfficeChair`s, `WheelChair`s, and `StaticHuman`s present in the current test environment that can be positioned via topological nodes. The exact obstacle types are defined in the `conf/scenario_server.yaml` under `obstacle_types`; The names used have to be the variable name of the obstacle in the morse environment. Obstacle types have to be lower case, node names can be camel case to enhance readability. Currently, in the test environment, each object has 10 instances so you can only use 10 of any single object. The Objects are positioned according to topological nodes following a naming scheme: `ObstacleStaticHuman1` for example positions a static human model on this node. `Obstacle` is the `obstacle_node_prefix`  defined in the `conf/scneario_server.yaml`, `StaticHuman` is the identifier of the obstacle type (if this omitted, an arbitrary object will be used), and `1` is just an arbitrary number to make the node name unique. When creating the topoligocal map, make sure that all edges from and to obstacle nodes are removed. Additionally, the nodes need to define a `localise_by_topic` json string so the robot will never localise itself based on these nodes. In the current simulation, we use the charging topic, because the robot will never charge and hence never localise itself at these nodes. We have to use an existing topic otherwise topological navigation fails. Example node:

```
- meta:
     map: mb_arena
     node: ObstacleStaticHuman1
     pointset: mb_test16
   node:
     edges: []
     localise_by_topic: '{"topic": "battery_state", "field": "charging", "val": true}'
     map: mb_arena
     name: ObstacleStaticHuman1
     pointset: mb_test16
     pose:
       orientation:
         w: 0.816770076752
         ...
       position:
        x: -4.58532047272
        ...
     verts:
     - x: 0.689999997616
       y: 0.287000000477
     ...
     xy_goal_tolerance: 0.3
     yaw_goal_tolerance: 0.1
```

Where the important bit is the `localise_by_topic: '{"topic": "battery_state", "field": "charging", "val": true}'` entry.

After the map has been loaded the obstacles will be spawned at (or better moved to) the respective nodes before the robot starts navigating. Before each test the arena is cleared to make sure that no obstacles linger.

