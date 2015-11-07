# Topological Navigation tests

The topological navigation tests are designed to optimise the parameter set of the DWA planner and to check the functionalitites of topological navigation in its execute policy mode. The test are run automatically on our Jenkins server but can also be run locally on your machine to test a new paremeterset. 

## Run test

Test for your whole worksapce can be run with `catkin_make test` which assumes that `strands_navigation` is present in your ros workspace. If you only want to run the test described here, use `catkin_make test --pkg topological_navigation`. To run the testes from an installed version of topological_navigation do:

```
rostest topological_navigation navigation_scenarios.test
```

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

* `mb_test0`: Traversing a 2m wide l-shaped corridor.
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

**Dynamic**

Coming soon

### Scenario Server control

The scenario server can also be used without the unit test file for tests on the real robot. Running

```
roslaunch topological_navigation navigation_scenarios.test robot:=true
```

will start only the scneario server and the simple policy generation. This assumes that everything on the robot, up to topological navigation, is running. It also assumes that the maps you want to use for testing are already inserted into the datacentre. The scneario server then offers 3 simple services:

* `/scneario_server/load` expects a string which is the name of the topological map you want to test. Keep in mind that this has to have the node `Start` and `End`.
* `/scenario_server/reset` is an empty service and is called to reset the data recording the the robot position. The robot position however cannot as easily be changed in real life as it can be in simulation, hence the robot has to be pushed to the starting node and then confirmed via a button press on the joypad. The reasoning behind having to push the robot is that the starting position might not be easily reachable via move_base. **Not implemented yet for the real robot**
* `\scneario_server/start` starts the poslicy execution. returns

 ```
bool nav_success
bool graceful_fail
bool human_success
float64 min_distance_to_human
float64 mean_speed
float64 distance_travelled
float64 travel_time
 ```

