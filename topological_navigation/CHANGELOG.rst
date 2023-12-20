^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.4 (2023-12-06)
------------------
* Merge pull request `#176 <https://github.com/LCAS/topological_navigation/issues/176>`_ from GPrathap/humble-dev
  adding server config for each action
* adding server config for each action
* Merge pull request `#172 <https://github.com/LCAS/topological_navigation/issues/172>`_ from GPrathap/humble-dev
  extending the functionality of bt trees to all action types
* Merge branch 'humble-dev' of github.com:LCAS/topological_navigation into humble-dev
* adding default config files
* extending the functionality of bt trees to all action types
* Contributors: GPrathap, James Heselden

3.0.3 (2023-11-24)
------------------

3.0.2 (2023-11-24)
------------------
* Merge pull request `#174 <https://github.com/LCAS/topological_navigation/issues/174>`_ from Iranaphor/humble-dev-dependencies
  removal of pip dependency
* removal of pip dependency
* Contributors: James R Heselden, Marc Hanheide

3.0.1 (2023-11-23)
------------------
* Merge pull request `#171 <https://github.com/LCAS/topological_navigation/issues/171>`_ from GPrathap/humble-dev
  adding different bt tree for row navigation
* adding row_traversal action type
* adding different bt tree for row navigation
* Merge pull request `#170 <https://github.com/LCAS/topological_navigation/issues/170>`_ from GPrathap/humble-dev
  adding traverse through poses and updating rviz plugin
* sending correct respond code
* adding traverse through poses
* Merge pull request `#169 <https://github.com/LCAS/topological_navigation/issues/169>`_ from GPrathap/humble-dev
  adding interactive marker server impl
* updating qos level
* adding interactive marker server impl
* Merge pull request `#167 <https://github.com/LCAS/topological_navigation/issues/167>`_ from Iranaphor/humble-dev-visuals
  Scaling added to legend
* scaling added to legend
* adding rviz vis tools init
* Merge pull request `#166 <https://github.com/LCAS/topological_navigation/issues/166>`_ from GPrathap/humble-dev
  adding simple policy server
* Merge branch 'humble-dev' of github.com:GPrathap/topological_navigation into humble-dev
* adding simple policy server
* Merge pull request `#165 <https://github.com/LCAS/topological_navigation/issues/165>`_ from GPrathap/humble-dev
  Adding the following core functionalities
* Forgot another launch_pytest to fix
* Corrected launch_pytest key
* adding test dependency
* adding testng dependency
* adding integration test script for testing navigation core components
* adding feedback messages
* removing synchronous service call
* code refactoring
* adding policy action server
* adding stopping goal action
* changing action and action types
* adding action server related changes
* fixing localisation node
* Merge pull request `#164 <https://github.com/LCAS/topological_navigation/issues/164>`_ from GPrathap/humble-dev
  fixing colcon build issue
* fixing colcon build issue
* Merge pull request `#162 <https://github.com/LCAS/topological_navigation/issues/162>`_ from Iranaphor/GoalMsg_Testing
  Development work toward a working ROS2 version
* inclusion of topomap_marker2.py with functional rescaling
* Merge pull request `#161 <https://github.com/LCAS/topological_navigation/issues/161>`_ from Iranaphor/humble-dev
  Topological map manager now working under ROS2
* incorrect tab removed
* updates to scalling for floats
* topovis scaling included for larger map visualsing from a distance
* topological transform publisher
* topological map visualiser and policy mode visualisers updated to ros2
* test script renamed to prevent crashing when using colcon test
* map_manager_2 up and running
* updates to fix imports when launching map_manager_2
* Merge pull request `#160 <https://github.com/LCAS/topological_navigation/issues/160>`_ from Iranaphor/humble-dev
  ROS 2 / Humble - porting
* python script building fix for topological_navigation
* Centralising topological_navigation stack messages to msgs package. (+version number sync across packages to 3.0.0)
* Updates to Maintainers and Authors
* full build working for topological_utils
* CMakeLists changes + restructuring for topological_utils
* Compilation of message files and restructuring heirarchy for ros2 compatibility
* initial work towards a working ros2 toponav
* Merge pull request `#156 <https://github.com/LCAS/topological_navigation/issues/156>`_ from LCAS/python3_marc_greg
  Some Python3 and multi-robot fixes
* another prefix bug relevant for multi-robot
* fixed wrong prefix for closest_edges intialisation
* Merge pull request `#154 <https://github.com/LCAS/topological_navigation/issues/154>`_ from adambinch/lcas
  Auto set goal when adding an edge to the topological map
* Goal mapping config can be set as a param
* Auto set goal when adding an edge to the topological map
  If a file e.g. thorvald_navigation_actions/config/inrownavGoal.yaml exists then the map manager will automatically set the goal of the action to the goal specified in that file, when the add edge function/service is called with arg action_type set to thorvald_navigation_actions/inrownavGoal. If that arg is not set or the file does not exist then a move base type goal is assumed.
* Merge pull request `#153 <https://github.com/LCAS/topological_navigation/issues/153>`_ from adambinch/lcas
  Add visualise_map2.py as install target
* Add visualise_map2.py as install target
* Merge pull request `#150 <https://github.com/LCAS/topological_navigation/issues/150>`_ from gpdas/lcas
  exec policy mode fix
* Merge pull request `#149 <https://github.com/LCAS/topological_navigation/issues/149>`_ from adambinch/lcas
  fix exec pol.
* start exec policy from closest or current nodes only
* fix exec pol
* Merge pull request `#147 <https://github.com/LCAS/topological_navigation/issues/147>`_ from adambinch/lcas
  Wait for closest edges before creating action servers.
* Wait for closest edges before creating action servers.
* Merge pull request `#146 <https://github.com/LCAS/topological_navigation/issues/146>`_ from adambinch/lcas
  Option to disable anchors and aliases in generated topological map ya…
* Option to disable anchors and aliases in generated topological map yaml file
* Merge pull request `#145 <https://github.com/LCAS/topological_navigation/issues/145>`_ from adambinch/lcas
  Initialise class variable
* Checks that closest edges are there in nav script. Also removed msg dependencies from cmakelists as all msgs are generated in topological_navigation_msgs
* Initialise class variable
* Merge pull request `#142 <https://github.com/LCAS/topological_navigation/issues/142>`_ from adambinch/exec_pol_nav_from_edge
  Navigating from the edge works with exec policy.
* Navigating from the edge works with exec policy.
  Also topic `topological_navigation/move_action_status` reports whether the goal
  of the current action is the final (toponav) goal (or not).
* Merge pull request `#141 <https://github.com/LCAS/topological_navigation/issues/141>`_ from adambinch/remove_movebase_dep
  Removed move base as dependency
* Removed move base as dependency
  tf is a run depend, not a test depend.
* Removed move base as dependency
  tf is a run depend, not a test depend.
* Merge pull request `#140 <https://github.com/LCAS/topological_navigation/issues/140>`_ from SAGARobotics/lcas
  Updating credentials
* Merge pull request `#9 <https://github.com/LCAS/topological_navigation/issues/9>`_ from adambinch/lcas
  updating credentials
* updating credentials
* Merge pull request `#139 <https://github.com/LCAS/topological_navigation/issues/139>`_ from SAGARobotics/lcas
  Update LCAS branch with SAGA branch
* Map manager unit test.
  One test which tests:
  1) If the topomap is received by a subscriber.
  2) That the `get_edges_between_nodes` service is advertised.
  3) Whether the service returns the 2 edges in the topomap.
* map manager improvements
* Additional edge reconfigure functionality.
  By default edge reconfigure resets the param back to its original value after the edge is traversed.
  This behaviour is now optional and can be disabled in the topological map edge config by setting `reset: false` for a given param.
  Updated map manager services accordingly.
* update
* marker lifetime set to half hour
* Fix no goto marker mode
* fix
* clear route markers for previous route
* adding route visualisation
* adding go to node markers and some visual improvements
* toponav launch files use new map visualiser
* Map Visualiser Based on toponav 2
* update
* update
* Removing mongodb as a dependency.
  Removed from `topological_navigation` and `topological_utils` packages.
* Merge pull request `#136 <https://github.com/LCAS/topological_navigation/issues/136>`_ from adambinch/melodic-devel
  New param `topological_navigation/move_base_goal` sets the goal type …
* update
* update
* update
* minor change
* New param `topological_navigation/move_base_goal` sets the goal type of the default action `move_base_name`.
  Defaults to standard move base goal type if not set.
  This will stop toponav breaking if the `move_base_name` action does not use move base type goals.
  Also an improvement to the add/remove edge param srvs.
* Merge pull request `#135 <https://github.com/LCAS/topological_navigation/issues/135>`_ from adambinch/melodic-devel
  Create action servers at end of initialisation.
* Create action servers at end of initialisation.
* Contributors: Adam Binch, GPrathap, Gautham P Das, Jaime Pulido Fentanes, James Heselden, James R Heselden, JamesH, Marc Hanheide, gpdas

2.4.0 (2022-01-25)
------------------
* Merge pull request `#134 <https://github.com/magnucha/topological_navigation/issues/134>`_ from adambinch/melodic-devel
  Map manager uses multiprocessing to load topomaps to decrease RAM usage.
* Minor changes
* minor change
* Using multiprocessing to yaml load topomap to decrease memory usage
* Merge pull request `#133 <https://github.com/magnucha/topological_navigation/issues/133>`_ from adambinch/melodic-devel
  Map manager improvements
* update
* update
* clear nodes srv
* update
* update
* update
* update
* Adding new msg and srv files
* update
* fail policy replanning does not skip the first edge of the new plan.
* update
* Merge branch 'master' of https://github.com/LCAS/topological_navigation into melodic-devel
* Merge pull request `#132 <https://github.com/magnucha/topological_navigation/issues/132>`_ from adambinch/load_with_json
  Load tmap2s with json to decrease RAM usage
* update
* update
* update
* update
* update
* new params namespaced
* Separate srv for setting influence zone.
  Caching the map and auto saving after service calls are both optional via ros params.
* Service /add_topological_node has option to add node vertices
* Merge pull request `#131 <https://github.com/magnucha/topological_navigation/issues/131>`_ from adambinch/melodic-devel
  Fix in map manager 2 when initialising with empty map before switchin…
* fix in map manager 2 when initialising with empty map before switching to another map
* Merge pull request `#130 <https://github.com/magnucha/topological_navigation/issues/130>`_ from adambinch/melodic-devel
  Edge reconfigure when target and origin nodes are the same
* Edge reconfigure when target and origin nodes are the same
* Merge pull request `#127 <https://github.com/magnucha/topological_navigation/issues/127>`_ from adambinch/melodic-devel
  Fix
* fixed
* testing...
* testing
* Merge pull request `#124 <https://github.com/magnucha/topological_navigation/issues/124>`_ from adambinch/melodic-devel
  Toponav does not attempt to execute fail policy actions when it is sh…
* Toponav does not attempt to execute fail policy actions when it is shutdown
* Merge pull request `#122 <https://github.com/magnucha/topological_navigation/issues/122>`_ from Jailander/quickfix
  Removing outdated dependency
* Removing outdated dependency
* Merge pull request `#121 <https://github.com/magnucha/topological_navigation/issues/121>`_ from adambinch/melodic-devel
  Fixes toponav breaking when move base is not being used
* minor change
* update
* Fixes toponav breaking when move base is not being used
* Merge pull request `#118 <https://github.com/magnucha/topological_navigation/issues/118>`_ from adambinch/fix_dependencies
  Fixing dependencies.
* restrictions manager moved from `src` to `scripts` and its install target added.
  navstats logger install target also added
* fix
* Fixing dependencies.
  `topological_map_edition.launch` moved to `topological_utils` package, which depends on `topological_rviz_tools` package.
  `topological_navigation` package depends on `topological_navigation_msgs` package.
* Merge pull request `#117 <https://github.com/magnucha/topological_navigation/issues/117>`_ from LCAS/noetic
  Basic navigation works in ros noetic but not all scripts are converted to python 3
* python 3 compatible for most parts of toponav but not all!
* Merge pull request `#116 <https://github.com/magnucha/topological_navigation/issues/116>`_ from adambinch/final_things
  A few final things.
* A few final things.
  Set `advertise_srvs` arg to False when initialising the map manager 2 allows other scripts/nodes to use its functions without advertising 20+ services.
  Descriptions of fail policy actions added to `UpdateFailPolicy.srv`.
  Tidying.
* Merge pull request `#115 <https://github.com/magnucha/topological_navigation/issues/115>`_ from francescodelduchetto/fail_policy_dirty
  working version of a "quick&dirty" implementation of the fail_policy
* Merge pull request `#6 <https://github.com/magnucha/topological_navigation/issues/6>`_ from adambinch/fail_policy_dirty
  Reset fail policy when new goal sent
* Reset fail policy when new goal sent
* replan now avoids current edge rather than current next node
* working version of a quick&dirty implementation of the fail_policy
* Merge pull request `#114 <https://github.com/magnucha/topological_navigation/issues/114>`_ from adambinch/melodic-devel
  Corrected inaccurate description of the `not_fluid` arg in `UpdateEdge.srv`
* minor changes
* Minor changes
* Merge pull request `#111 <https://github.com/magnucha/topological_navigation/issues/111>`_ from adambinch/remove_strands_dependencies
  Removing strands navigation dependencies from topological navigation.
* Last minor changes
* Merge branch 'remove_strands_dependencies' of github.com:adambinch/topological_navigation into remove_strands_dependencies
* minor changes
* Merge pull request `#4 <https://github.com/magnucha/topological_navigation/issues/4>`_ from francescodelduchetto/adambinch-remove_strands_dependencies
  fix old imports
* fix old imports
* minor change
* minor changes
* toponav dies (more) gracefully
* Merge branch 'master' of github.com:LCAS/topological_navigation into remove_strands_dependencies
* Merge pull request `#113 <https://github.com/magnucha/topological_navigation/issues/113>`_ from gpdas/fixes
  Mostly additive.
  Minor fixes
* initialise quarternions in markers
* update
* strands dependencies removed from topological_rviz_tools
* strands dependencies removed from `topological_navigation/topological_navigation`
* Merge branch 'master' of github.com:LCAS/topological_navigation into remove_strands_dependencies
  # Conflicts:
  #	topological_navigation/src/topological_navigation/manager2.py
  #	topological_navigation_msgs/CMakeLists.txt
* Merge pull request `#112 <https://github.com/magnucha/topological_navigation/issues/112>`_ from adambinch/fail_policy_srvs
  Services for updating the fail policy.
* Services for updating the fail policy.
  Service `/topological_map_manager2/update_edge` now has field for updating the edge's fail policy.
  New service `/topological_map_manager2/update_fail_policy` for updating the fail policy of every edge in the map.
* Removing strands dependencies from topological navigation
* Removing strands dependencies from topological navigation
* Removing strands nav dependencies from navigation
* Removing strands nav dependencies from navigation
* improvements to prints and logs
* Minor change
* Publishers all started with `queue_size` arg.
  Improved prints and logs.
* Removed strands nav msgs from localisation
* Merge branch 'master' of github.com:LCAS/topological_navigation into remove_strands_dependencies
* Merge pull request `#106 <https://github.com/magnucha/topological_navigation/issues/106>`_ from francescodelduchetto/toponav2-restrictions-params
  removing publishing restricted map in service callback
* Legacy map manager no longer dependent on strands nav msgs
* Moving topomap msgs from strands nav to toponav repo. Map manager 2 is now strands independent.
* Removing strands navigation dependencies from the Toponav repo.
* Removing strands navigation dependencies from TopoNav.
  Copying srv definitions used by toponav from strands nav to the toponav repo.
* Merge pull request `#110 <https://github.com/magnucha/topological_navigation/issues/110>`_ from adambinch/melodic-devel
  Navigation defaults to using edge reconfigure.
* minor change
* minor changes
* toponav launch update
* toponav launch runs restrictions manager.
* Extension for new map is `.tmap2`
* Option to use restricted map in main toponav launch file.
* Navigation defaults to using edge reconfigure.
* Navigation defaults to using edge reconfigure.
* Merge pull request `#107 <https://github.com/magnucha/topological_navigation/issues/107>`_ from adambinch/node_names
  Resolves Issue `#90 <https://github.com/magnucha/topological_navigation/issues/90>`_, Adds datum to the tmap meta, and other things.
* Merge pull request `#3 <https://github.com/magnucha/topological_navigation/issues/3>`_ from francescodelduchetto/adambinch-node_names
  removing splitting underscore edges for retrieving nodes, using the n…
* removing splitting underscore edges for retrieving nodes, using the new function
* Merge branch 'master' of github.com:LCAS/topological_navigation into node_names
* Merge pull request `#109 <https://github.com/magnucha/topological_navigation/issues/109>`_ from adambinch/faster_route_search
  Faster route planner.
* possibly faster route search
* Added launch file for running the restrictions manager for a multi robot scenario.
  Some improvements to prints/logs and tidying.
* Some optimisation of the navigation script.
* Can now pass properties of the edge's origin node to its goal args in the topological map using `+`
  (similar to passing properties of the edge's destination node using `$`)
* Service `/topological_map_manager2/update_edge` replaces `/topological_map_manager2/update_edge_action`
  setting the same args with an additional boolean arg for setting whether navigation is fluid or not.
  Uses srv type `topological_navigation_msgs.srv.UpdateEdge`
* Map manager service for adding GNSS latitude/longitude to the topological map's top-level meta info
* removing publishing restricted map in service callback
* Merge branch 'master' of github.com:LCAS/topological_navigation into node_names
* Merge pull request `#104 <https://github.com/magnucha/topological_navigation/issues/104>`_ from francescodelduchetto/toponav2-restrictions-params
  restriction manager gets the out_topic for the restricted map and the config file as parameters
* restriction manager gets the out_topic for the restricted map and the config file as parameters
* Merge pull request `#78 <https://github.com/magnucha/topological_navigation/issues/78>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions
* removing obsolete test script for restrictions
* fix, from pull-request `#5 <https://github.com/magnucha/topological_navigation/issues/5>`_
* Navigation can handle node names containing underscores
* Map managers can handle node names containing underscores
* optimise a bit more obstacleFree
* making task and robot type restrictions faster
* adding checks for the coordination config file to the restrictions
* Merge pull request `#4 <https://github.com/magnucha/topological_navigation/issues/4>`_ from adambinch/toponav2-restrictions
  Nav script checks for availability of restriction services before att…
* turn prints to rospy logs
* Nav script checks for availability of restriction services before attempting to use them.
  Therefore toponav can run independently of the restrictions manager.
* adding lost files after the merge; fix few changes
* Merge branch 'master' of github.com:LCAS/topological_navigation into francescodelduchetto-toponav2-restrictions
* remove satisfy_runtime_restrictions code and comment some prints
* refine implementation of obstacleFree restriction with closest_node topic of robots; navigation script checks the runtime restrictions on nodes/edges before executing an action
* Merge pull request `#102 <https://github.com/magnucha/topological_navigation/issues/102>`_ from adambinch/default_edge_reconf
  Edge reconfigure defaults to new method.
* Edge reconfigure defaults to new method.
* Merge pull request `#101 <https://github.com/magnucha/topological_navigation/issues/101>`_ from adambinch/new_topics
  New topics
* small fix, parentesys missing
* remove startOnNode restriction
* adding fluid navigation flag in manager2
* do not call runtime_restriction but rather use an ad-hoc flag for 'fluid_navigation' in the tmap
* minor changes
* minor change
* Better prints and logs from nav script.
  Both action servers report terminal state aborted if the move action is aborted.
  Better default move base actions list in toponav launch file.
* minor change
* minor improvements.
* Better prints/logs for go to node.
* round published dist to closest node to 3dp
* minor change
* move action status topic now has std msg type String
* Status of move action moved from go to node action definition to its own topic /topological_navigation/move_action_status.
  Msg definition is topological_navigation_msgs/MoveActionStatus
* improvements
* Status field of feedback converted to json string.
* When move action is aborted the toponav feedback reports the route as the current node.
* improvements
* Status of current edge action reported as a string.
* minor change
* minor change
* Status of the current action is reported in the feedback of the go-to-node action definition.
* Distance to closest node published to topic `/closest_node_distance`.
  This is always the distance to the physically closest node.
* Merge pull request `#100 <https://github.com/magnucha/topological_navigation/issues/100>`_ from adambinch/toponav2_launch
  Launch files toponav 2 ready
* minor change
* Making launch files toponav 2 ready.
* minor change
* Minor change
* improved description of arg
* minor improvement.
* Making launch files toponav 2 ready.
* Making launch files toponav 2 ready.
* Making launch files toponav2 ready.
* Making launch files toponav 2 ready.
* minor improvement
* minor change
* Making launch files toponav 2 ready.
* Making launch files toponav 2 ready.
* Making launch files toponav 2 ready.
* minor change
* minor improvement
* Making launch files toponav 2 ready.
  Improvement to get_edge_vectors function in localisation.
* navstats_loger.py changed to navstats_logger.py
* Making launch files toponav 2 ready.
* Making launch files toponav 2 ready.
* Making launch files toponav 2 ready.
  Updated rviz config.
  Tidying of nav script.
* Merge branch 'master' of github.com:LCAS/topological_navigation into toponav2_launch
* Merge pull request `#99 <https://github.com/magnucha/topological_navigation/issues/99>`_ from adambinch/melodic-devel
  Fix
* exec policy sets the correct target
* Improvement to exec policy prints
* minor change
* minor change
* improvements to prints
* minor change
* Fix
* Making launch files toponav 2 ready
* Merge branch 'master' of github.com:LCAS/topological_navigation into toponav2_launch
* Merge pull request `#98 <https://github.com/magnucha/topological_navigation/issues/98>`_ from adambinch/faster_route_search2
  Faster route distance function
* Faster route distance function
* Merge pull request `#96 <https://github.com/magnucha/topological_navigation/issues/96>`_ from adambinch/faster_route_search2
  Faster Route Planner for Toponav 2
* Navigation now takes advantage of the faster route planner
* adding possibility of satisfying runtime restrictions, not tested yet
* Merge branch 'master' of github.com:LCAS/topological_navigation into faster_route_search2
* tidying
* Faster Route Search 2
* Reverted change to navigation script as those will be done in a separate PR.
* adding services to evaluate single nodes/edges and exactPose restriction
* Modifying launch files for toponav 2 usage.
  Bit of tidying of navigation script.
* up
* WIP adding runtime restriction for obstacles in path, based on the other robots poses
* allow topics namespaced
* correctly publishing topomap2
* providing restricted tmaps for each robot£
* restrictions manager auto infer robot state from namespaced topic if state not provided
* Merge branch 'toponav2-devel-restrictions' of github.com:francescodelduchetto/topological_navigation into toponav2-devel
* Merge branch 'toponav2-restrictions' of github.com:francescodelduchetto/topological_navigation into toponav2-devel
* WIP restrictions to ground to specific robot automatically using namespace
* 'restrictions_manager' to 'topological_restrictions_manager'
* adding requirement of sympy>=1.5.1
* restriction manager works with runtime and planning restrictions; test script for testing
* Merge branch 'melodic-devel' of https://github.com/adambinch/topological_navigation into adam_melodic-devel
* Merge branch 'master' of https://github.com/adambinch/topological_navigation into adam-master
* WIP kinda of works
* WIP restrictions manager
* Contributors: Adam Binch, Gautham P Das, Jaime Pulido Fentanes, MikHut, adambinch, francescodelduchetto, gpdas

2.3.0 (2021-07-15)
------------------
* Merge pull request `#95 <https://github.com/LCAS/topological_navigation/issues/95>`_ from adambinch/melodic-devel
  Navigating from the closest edge is now optional.
* simplification
* Navigating from the closest edge is now optional.
  Set with param `max_dist_to_closest_edge` (default = 1 meter)
  Robot will NOT attempt to navigate from the closest edge if ANY of the following are true:
  a) `max_dist_to_closest_edge` = 0
  b) the distance to closest edge > `max_dist_to_closest_edge`
  c) current node != "none"
  Else the robot will navigate from the closest node in exactly the same way as it has always done.
* Merge pull request `#94 <https://github.com/LCAS/topological_navigation/issues/94>`_ from adambinch/tmap_to_tmap2
  Script for converting all tmaps found in repo to tmap2 format and script for populating tmap2s with params from edge reconfigure config files.
* Script for populating tmap2s with edge reconfigure params from edge reconfigure groups config files.
* Finished script for converting tmaps.
  Map manager services registered in the class __init_\_ function and the tmap
  is loaded in separate init_map function.
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
  # Conflicts:
  #	topological_navigation/src/topological_navigation/manager.py
* Merge pull request `#93 <https://github.com/LCAS/topological_navigation/issues/93>`_ from adambinch/melodic-devel
  fix
* fix
* Merge pull request `#91 <https://github.com/LCAS/topological_navigation/issues/91>`_ from adambinch/melodic-devel
  Nav from closest edge fix
* fix
* If the closest edges are of equal distance (usually a bidirectional edge) then use the destination node that results in a shorter route to the goal.
* Localisation map callback only sets map received when all computation inside the callback has completed.
  Comments and tidying.
* another fix
* fix
* distance from edge taken into account when deciding to navigate from closest edge
* toponav generates route between the destination node of the closest edge and the goal node
* Merge pull request `#89 <https://github.com/LCAS/topological_navigation/issues/89>`_ from adambinch/melodic-devel
  Efficient computing of closest edges in localisation.
* minor change
* New `get_edge_distances_to_pose` functions catches exeception.
  Code more efficient and tidying.
  readme.md updated to warn reader that current instructions apply to legacy branch.
* more efficient
* more efficient
* tidying
* vectorised the toponav version of point2line, such that you can pass it a numpy array of edges (an array of vectors) and get it to return you the distances to every edge in the map at once.
* Improvement to reporting of errors by `get_edge_distances_to_pose` function in localisation.
* Better name for new param
* Efficient computing of closest edges in localisation.
  Option to get the closest edges only from the N closest nodes to the robot.
  Useful for very large and dense maps (such as clockhouse vanity transportation map) where you do not want to be
  computing the distance from every edge in the map to the robot.
  N set by rosparam `/topological_localisation/NumClosestNodes`.
  Default is 0, such that that the distance is computed for every edge in the map.
* Merge pull request `#86 <https://github.com/LCAS/topological_navigation/issues/86>`_ from adambinch/melodic-devel
  Switch map srv looks for maps in current working directory and also converts from tmap1 to tmap2 and vice-versa.
* simplification
* tidying
* minor change
* Removed unused `n_tries` param and imports.
  Tidying.
* params `/topological_map_filename` and `/topological_map2_filename`
* fix
* minor change
* tidying
* Merge branch 'master' of github.com:LCAS/topological_navigation into melodic-devel
* Merge pull request `#88 <https://github.com/LCAS/topological_navigation/issues/88>`_ from adambinch/fix
  Fix for localisation breaking when edge in map has destination equal to origin
* only prints error once
* toponav checks if an edge in the map has a destination equal to its origin
* testing build
* testing build
* removed redundant service
* tidying
* minor improvements
* Starting script for converting all legacy tmaps in a repo to new format
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* corresponding changes to manager 2 switch maps srv
* Fix for old map manager switch map srv returning a service response error when converting the switched map to new format.
* switch map srv assumes you are switching maps within the same dir when loading map from file
* old map manager switch map srv converts updated map to new format
* Merge pull request `#85 <https://github.com/LCAS/topological_navigation/issues/85>`_ from adambinch/melodic-devel
  Map manager services for updating edge action, type and goal.
* Retained ability to do edge reconfigure in the old way (currently default). Example config provided.
* map manager service for setting the action, action type and goal for an edge
  map manager service for setting the action type and goal for any edge with a given action
* Merge pull request `#57 <https://github.com/LCAS/topological_navigation/issues/57>`_ from LCAS/toponav2-devel
  Topological Navigation version 2 Master Branch
* Merge pull request `#82 <https://github.com/LCAS/topological_navigation/issues/82>`_ from adambinch/fix_conflicts
  Fix conflicts
* Merge branch 'master' of github.com:LCAS/topological_navigation into fix_conflicts
  # Conflicts:
  #	topological_navigation/scripts/execute_policy_server.py
  #	topological_navigation/scripts/navigation.py
* Merge pull request `#77 <https://github.com/LCAS/topological_navigation/issues/77>`_ from adambinch/melodic-devel
  Fixes
* fix for a couple of the utils
* tidying
* tidying
* minor change
* Route checker checks for empty strings and other improvements
* fix for route checker not catching an empty exec policy route
* fixing race conditions when multiple goals arrive at the same time
* old manager allows switching of topomap when loading from a file
* fix for go to node action not ending in correct terminal state when preempted by exec policy and vice-versa
* minor imporvements
* Merge pull request `#62 <https://github.com/LCAS/topological_navigation/issues/62>`_ from francescodelduchetto/master
  New features in bayesian_topological_localisation node
* mnior changes
* Fix for exec policy action breaking toponav when the goal route is invalid
* minor changes and tidying
* improved route checking function
* Function for checking if an exec policy route is valid
* Fix for go to node action breaking when the goal exists but there is no route to it.
* tidying
* Map manager fixes
* fix
* minor change
* minor changes
* Improvements to edge action manager
* Merge pull request `#76 <https://github.com/LCAS/topological_navigation/issues/76>`_ from adambinch/any_edge_action
  Improvement to edge action manager
* minor change
* fixes
* minor change
* minor changes
* Fix for goal preempting breaking nav
* Merge pull request `#75 <https://github.com/LCAS/topological_navigation/issues/75>`_ from adambinch/any_edge_action
  Topological navigation can handle any type of goal.
* minor change
* possible fix
* map manger 2 sets default action type as `move_base_msgs/MoveBaseGoal`
* fix
* Functions of edge reconf manager called only when there are param to reconfigure.
* Minor changes
* Removed monitored navigation
* Integration of edge action manager into navigation script.
  Toponav can now use any type of goals.
* get_node_from_tmap2 utility modified so it returns all of the node inc its meta.
  consequent changes to other files.
* New manager 2 srv for updating the action type of each edge in the tmap according to the action name
* Edge action manager finished hopefully
* Improvements to the edge action manager
* Edge action manager: can construct goals and map them to ROS messages flexibly.
  Updated map manager with new default fields for the goal specified in the topological edge.
* Working on edge action manager.
  map manager 2 now sets rosparam `topological_map_name`
* Merge pull request `#73 <https://github.com/LCAS/topological_navigation/issues/73>`_ from adambinch/switch_topomap
  switch topological maps srv works when loading tmaps from files
* switch topological maps srv works when loading tmaps from files
* minor changes
* Making new class for handling (any) edge actions
* Merge branch 'toponav2-devel' of github.com:LCAS/topological_navigation into any_edge_action
* Merge pull request `#72 <https://github.com/LCAS/topological_navigation/issues/72>`_ from adambinch/toponav2-devel
  minor change
* minor change
* Merge pull request `#71 <https://github.com/LCAS/topological_navigation/issues/71>`_ from adambinch/toponav2-devel
  Edge reconf manager improvement to exception handling
* Edge reconf manager improvement to exception handling
* edge reconf manager improvement to exception handling
* Merge branch 'toponav2-devel' of github.com:LCAS/topological_navigation into any_edge_action
* Merge pull request `#70 <https://github.com/LCAS/topological_navigation/issues/70>`_ from adambinch/toponav2-devel
  Edge Reconfigure Improvements
* more efficient
* edge reconfigure manager only resets params that have been reconfigured
* tidying
* Tidying
* Merge pull request `#69 <https://github.com/LCAS/topological_navigation/issues/69>`_ from adambinch/pub_closest_edges
  Planning considering edges when robot current_node = none and topological localisation publishes closest edges to the robot.
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/adambinch-pub_closest_edges
  Planning considering edges when robot current_node = none
* warn to info
* planning ensures that the robot does not goes back to closest node before navigating and that it always navigate from the closest edge when far from any node
* Function for getting distance to edges is much more efficient
* Merge branch 'pub_closest_edges' of https://github.com/adambinch/topological_navigation into adambinch-pub_closest_edges
* tidying
* tidying
* tidying
* tidying
* Topological Localisation publishes closest edges to the robot.
  Publishes the two closest edges to the robot on the topic `/closest_edges`
  with message type `topological_navigation_msgs.msg.ClosestEdges`
  This message has fields for the edge ids and the distances (to the robot) e.g.
  ---
  edge_ids: [WayPoint56_WayPoint66, WayPoint66_WayPoint56]
  distances: [0.3709999918937683, 0.3709999918937683]
  ---
  Often the two edges reported on this topic will form a bi-directional edge.
* Merge pull request `#63 <https://github.com/LCAS/topological_navigation/issues/63>`_ from ayu135/combine_exec_nav
  Combine execute policy and nav actions in a single script
* Added none check for set ended
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/ayu135-combine_exec_nav
  Ayu135 combine exec nav
* correctly cancelling previous goal and waiting before starting the new one
* remove some superfluous lines in preempting nav goals
* Merge branch 'combine_exec_nav' of https://github.com/ayu135/topological_navigation into toponav2-devel
* Merge pull request `#67 <https://github.com/LCAS/topological_navigation/issues/67>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions implementation
* Merge branch 'combine_exec_nav' of https://github.com/ayu135/topological_navigation into ayu135-combine_exec_nav
* 'restrictions_manager' to 'topological_restrictions_manager'
* adding requirement of sympy>=1.5.1
* restriction manager works with runtime and planning restrictions; test script for testing
* WIP restrictions manager
* Merge pull request `#66 <https://github.com/LCAS/topological_navigation/issues/66>`_ from adambinch/melodic-devel
  Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
* if updating node restrictions then apply planning restrictions to edges involving the node.
  Set this behaviour with new boolean arg `update_edges` in srv for updating a node's restrictions
* Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
  Both are boolean sentences (default="True")
  Update restrictions services modified to account for this.
* Better integrate nav and exec policy actions
* Combined execute policy and nav actions in a single script navigation.py
* Removed tmap1 related functions fron nav.py
* Merge pull request `#64 <https://github.com/LCAS/topological_navigation/issues/64>`_ from adambinch/melodic-devel
  Map manager services for updating restrictions
* Map manager services for updating restrictions
  Restrictions field for a node or an edge is now a string which is a boolean sentence (default="True").
  New services `/topological_map_manager2/update_node_restrictions` and `/topological_map_manager2/update_edge_restrictions` added in the map manager 2.
* Merge pull request `#3 <https://github.com/LCAS/topological_navigation/issues/3>`_ from francescodelduchetto/particles-states
  Particles states
* remove modifications to route_search
* Merge pull request `#60 <https://github.com/LCAS/topological_navigation/issues/60>`_ from adambinch/melodic-devel
  Base frame used by localisation is no longer hard coded (in toponav 2).
* file renamed in install targets
* Merge branch 'melodic-devel' of github.com:adambinch/topological_navigation into melodic-devel
* pose pub is replaced with a tf broadcaster. renamed file
* added install target for the new node.
* New node for publishing the map to topomap transform on the topic `/topological_transform` with msg type `geometry_msgs/TransformStamped`
* Base frame used by localisation is no longer hard coded.
  It is set by a rosparam `topological_localisation/base_frame` (default=`base_link`).
  topo_map frame is retrieved from the topological map.
  removed unused imports from localisation.
* Merge pull request `#58 <https://github.com/LCAS/topological_navigation/issues/58>`_ from adambinch/melodic-devel
  removed `use_tmap2` arg from localisation - localisation uses the new…
* removed `use_tmap2` arg from localisation - localisation uses the new format map only.
* Merge pull request `#54 <https://github.com/LCAS/topological_navigation/issues/54>`_ from adambinch/edge_reconf
  Edge reconfigure integration for the new map type
* minor improvement to the edge reconfigure manager
* The edge reconfigure manager is in its own file.
* Cant add duplicate params when using srv `add_param_to_edge_config`
* Fixes and improvements to the edge reconfigure manager.
* Lots of fixes
* EdgeReconfigureManager class done. Needs testing.
* Service `update_edge_config` renamed to `add_param_to_edge_config` to better reflect what it does.
  That service and `rm_param_from_edge_config` modified to account for the changes in the previous commit.
  Constructing new class `EdgeReconfigureManager` in `navigation.py` to handle everything edge reconfigure related.
* topo path planning considers blacklisted nodes
* Edges config is now a list where each item is a dict with the params namespace, name and value.
  The default config is empty and the tmap to tmap2 conversion sets an empty config.
* Service for removing params from an edge's config and a fix.
* service `update_edge_reconf` renamed to `update_edge_config`
* New service for adding/updating edge reconfigure parameters.
* fix
* Function that does the new to old conversion catches exceptions
* `convert_to_legacy` rosparam sets whther the new to old format map conversion happens or not
* map manager 2 coverts new format maps (broadcast on the topic `/topological_map_2`) to the old format (broadcast on the topic `/topological_map`).
  This allows nodes/actions that rely on the old map format to function whilst using/testing features from the new map.
* The arg `use_tmap2` (used by localisation and navigation) is now a rosparam
* Merge pull request `#47 <https://github.com/LCAS/topological_navigation/issues/47>`_ from heuristicus/eband-planner
  Allow use of EBandPlannerROS as local planner
* Some fixes:
  The monitored navigation function in `navigation.py` expects a geom msgs Pose object rather than a monitored nav goal object (stops nav breaking when using the old map format).
  Navigation now reconfigures move base tolerances according to the values specified in the tmap.
* Merge pull request `#45 <https://github.com/LCAS/topological_navigation/issues/45>`_ from ayu135/toponav2-devel
  Added tmap2 support for navigation.py and execute_policy
* added route_search2.py for tmap2 and corresponding changes and fixes
* Added separate navigate and follow route funtions for tmap2
* Added command line option for topomap2
* Some fixes after testing
* Updated map callback for execute policy
* adding support for tmap2 and combining execute policy
* Merge pull request `#44 <https://github.com/LCAS/topological_navigation/issues/44>`_ from adambinch/manager2_srvs
  All manager services available and working on new map type
* Improvement to the function that loads the map
* Correction to srv `/topological_map_manager2/update_edge`
* add max_vel_lin for eband in dynparam mapping
* add eband to dynparam mappings
* When loading a map using the map manager 2, it is cached in `$HOME/.ros/topological_maps`.
  General improvements.
* correction
* reverting accidental change
* minor improvement
* Added srvs `/topological_map_manager2/rm_tag_from_node` and `/topological_map_manager2/update_edge`
* Added srvs `/topological_map_manager2/modify_node_tags` and `/topological_map_manager2/add_tag_to_node`
* Added srvs `/topological_map_manager2/update_node_pose` and `/topological_map_manager2/update_node_tolerance`.
  General improvements.
* Added service `/topological_map_manager2/update_node_name`
* Added service `/topological_map_manager2/add_content_to_node`
* Added services `/topological_map_manager2/remove_topological_node` and `/topological_map_manager2/remove_edge`.
  General improvements.
* Made map manager 2 node more user friendly
  Corrected error when generating influence zone vertices
  removed unnecessary msg definition
  General improvements
* Added services `/topological_map_manager2/add_topological_node` and `/topological_map_manager2/add_edges_between_nodes`
* Made node(`map_manager2.py`) for loading in new format maps using the manager 2 class.
  Added service `/topological_map_manager2/write_topological_map` for writing new format topological maps to yaml files. If you dont specify the path/name of the map then it will just write to the one given to the manager 2 class.
  When loading a tmap (`tmap.tmap`) from a file using the original map manager, the converted tmap can now be written to a file (`tmap.yaml`) using the `write_topological_map` service.
  Added map sanity checking function to the manager 2 class.
* minor changes
* Merge branch 'master' of https://github.com/LCAS/topological_navigation into manager2_srvs
* Created `topological_navigation_msgs` package that will contain the new msg and srv types for the new format topomap.
  Added services `/topological_map_manager2/switch_topological_map` and `/topological_map_manager2/get_edges_between_nodes`.
  Added function in map manager 2 that warns if you are trying to use it to load an old-format topomap.
  Some minor improvements.
* Edge id field included in new map. Default is `origin_destination`
* minor change
* Added manager 2 services:
  - `get_topological_map `
  - `get_tagged_nodes`
  - `get_tags`
  - `get_node_tags`
* correction
* map manager 2 class now publishes the new format topomap, rather than the origin map manager.
  map manager 2 can now load a new format topomap from a given file path.
* Merge pull request `#31 <https://github.com/LCAS/topological_navigation/issues/31>`_ from adambinch/loc2
  All functions in localisation now work with the new map type.
* correction
* corrections
* All functions in localisation can now work with the new map type. This includes its services.
* rearranging
* Argument added to switch between using map types in topological localisation.
  Get nodes with tag service in localisation now works on new map type.
  Map manager 2 now has service for getting nodes with a tag.
  Map manager now adds a nodes tags during map conversion.
* Localisation uses the topo_map to base_link tf transform, rather than the robot pose.
* prettyfying
* Map manager broadcasts map->topo_map tf transform.
* Merge pull request `#29 <https://github.com/LCAS/topological_navigation/issues/29>`_ from adambinch/topomap2
  Function for converting topological maps into the new format in the
* New map type is regenerated and republished when current map is updated.
* bit of tidying
* Map manager keeps its class attribute copy of the new map as a dictionary, but publishes it as a string.
* updated package xml
* minor change
* Function for converting topological maps into the new format in the map manager.
  Includes a map manager 2 class for handling topological maps in the new format.
* Contributors: Adam Binch, Ayush Sharma, Jaime Pulido Fentanes, Michal Staniaszek, adambinch, francescodelduchetto, gpdas

* Merge pull request `#95 <https://github.com/LCAS/topological_navigation/issues/95>`_ from adambinch/melodic-devel
  Navigating from the closest edge is now optional.
* simplification
* Navigating from the closest edge is now optional.
  Set with param `max_dist_to_closest_edge` (default = 1 meter)
  Robot will NOT attempt to navigate from the closest edge if ANY of the following are true:
  a) `max_dist_to_closest_edge` = 0
  b) the distance to closest edge > `max_dist_to_closest_edge`
  c) current node != "none"
  Else the robot will navigate from the closest node in exactly the same way as it has always done.
* Merge pull request `#94 <https://github.com/LCAS/topological_navigation/issues/94>`_ from adambinch/tmap_to_tmap2
  Script for converting all tmaps found in repo to tmap2 format and script for populating tmap2s with params from edge reconfigure config files.
* Script for populating tmap2s with edge reconfigure params from edge reconfigure groups config files.
* Finished script for converting tmaps.
  Map manager services registered in the class __init_\_ function and the tmap
  is loaded in separate init_map function.
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
  # Conflicts:
  #	topological_navigation/src/topological_navigation/manager.py
* Merge pull request `#93 <https://github.com/LCAS/topological_navigation/issues/93>`_ from adambinch/melodic-devel
* Merge pull request `#91 <https://github.com/LCAS/topological_navigation/issues/91>`_ from adambinch/melodic-devel
  Nav from closest edge fix
* If the closest edges are of equal distance (usually a bidirectional edge) then use the destination node that results in a shorter route to the goal.
* Localisation map callback only sets map received when all computation inside the callback has completed.
  Comments and tidying.
* another fix
* distance from edge taken into account when deciding to navigate from closest edge
* toponav generates route between the destination node of the closest edge and the goal node
* Merge pull request `#89 <https://github.com/LCAS/topological_navigation/issues/89>`_ from adambinch/melodic-devel
  Efficient computing of closest edges in localisation.
* minor change
* New `get_edge_distances_to_pose` functions catches exeception.
  Code more efficient and tidying.
  readme.md updated to warn reader that current instructions apply to legacy branch.
* more efficient
* more efficient
* tidying
* vectorised the toponav version of point2line, such that you can pass it a numpy array of edges (an array of vectors) and get it to return you the distances to every edge in the map at once.
* Improvement to reporting of errors by `get_edge_distances_to_pose` function in localisation.
* Better name for new param
* Efficient computing of closest edges in localisation.
  Option to get the closest edges only from the N closest nodes to the robot.
  Useful for very large and dense maps (such as clockhouse vanity transportation map) where you do not want to be
  computing the distance from every edge in the map to the robot.
  N set by rosparam `/topological_localisation/NumClosestNodes`.
  Default is 0, such that that the distance is computed for every edge in the map.
* Merge pull request `#86 <https://github.com/LCAS/topological_navigation/issues/86>`_ from adambinch/melodic-devel
  Switch map srv looks for maps in current working directory and also converts from tmap1 to tmap2 and vice-versa.
* Merge pull request `#88 <https://github.com/LCAS/topological_navigation/issues/88>`_ from adambinch/fix
  Fix for localisation breaking when edge in map has destination equal to origin
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* corresponding changes to manager 2 switch maps srv
* Fix for old map manager switch map srv returning a service response error when converting the switched map to new format.
* switch map srv assumes you are switching maps within the same dir when loading map from file
* old map manager switch map srv converts updated map to new format
* Merge pull request `#85 <https://github.com/LCAS/topological_navigation/issues/85>`_ from adambinch/melodic-devel
  Map manager services for updating edge action, type and goal.
* Retained ability to do edge reconfigure in the old way (currently default). Example config provided.
* map manager service for setting the action, action type and goal for an edge
  map manager service for setting the action type and goal for any edge with a given action
* Merge pull request `#57 <https://github.com/LCAS/topological_navigation/issues/57>`_ from LCAS/toponav2-devel
  Topological Navigation version 2 Master Branch
* Merge pull request `#82 <https://github.com/LCAS/topological_navigation/issues/82>`_ from adambinch/fix_conflicts
  Fix conflicts
* Merge branch 'master' of github.com:LCAS/topological_navigation into fix_conflicts
  # Conflicts:
  #	topological_navigation/scripts/execute_policy_server.py
  #	topological_navigation/scripts/navigation.py
* Merge pull request `#77 <https://github.com/LCAS/topological_navigation/issues/77>`_ from adambinch/melodic-devel
  Fixes
* fix for a couple of the utils
* tidying
* tidying
* minor change
* Route checker checks for empty strings and other improvements
* fix for route checker not catching an empty exec policy route
* fixing race conditions when multiple goals arrive at the same time
* old manager allows switching of topomap when loading from a file
* fix for go to node action not ending in correct terminal state when preempted by exec policy and vice-versa
* minor imporvements
* Merge pull request `#62 <https://github.com/LCAS/topological_navigation/issues/62>`_ from francescodelduchetto/master
  New features in bayesian_topological_localisation node
* mnior changes
* Fix for exec policy action breaking toponav when the goal route is invalid
* minor changes and tidying
* improved route checking function
* Function for checking if an exec policy route is valid
* Fix for go to node action breaking when the goal exists but there is no route to it.
* tidying
* Map manager fixes
* fix
* minor change
* minor changes
* Improvements to edge action manager
* Merge pull request `#76 <https://github.com/LCAS/topological_navigation/issues/76>`_ from adambinch/any_edge_action
  Improvement to edge action manager
* minor change
* fixes
* minor change
* minor changes
* Fix for goal preempting breaking nav
* Merge pull request `#75 <https://github.com/LCAS/topological_navigation/issues/75>`_ from adambinch/any_edge_action
  Topological navigation can handle any type of goal.
* minor change
* possible fix
* map manger 2 sets default action type as `move_base_msgs/MoveBaseGoal`
* fix
* Functions of edge reconf manager called only when there are param to reconfigure.
* Minor changes
* Removed monitored navigation
* Integration of edge action manager into navigation script.
  Toponav can now use any type of goals.
* get_node_from_tmap2 utility modified so it returns all of the node inc its meta.
  consequent changes to other files.
* New manager 2 srv for updating the action type of each edge in the tmap according to the action name
* Edge action manager finished hopefully
* Improvements to the edge action manager
* Edge action manager: can construct goals and map them to ROS messages flexibly.
  Updated map manager with new default fields for the goal specified in the topological edge.
* Working on edge action manager.
  map manager 2 now sets rosparam `topological_map_name`
* Merge pull request `#73 <https://github.com/LCAS/topological_navigation/issues/73>`_ from adambinch/switch_topomap
  switch topological maps srv works when loading tmaps from files
* switch topological maps srv works when loading tmaps from files
* minor changes
* Making new class for handling (any) edge actions
* Merge branch 'toponav2-devel' of github.com:LCAS/topological_navigation into any_edge_action
* Merge pull request `#72 <https://github.com/LCAS/topological_navigation/issues/72>`_ from adambinch/toponav2-devel
  minor change
* minor change
* Merge pull request `#71 <https://github.com/LCAS/topological_navigation/issues/71>`_ from adambinch/toponav2-devel
  Edge reconf manager improvement to exception handling
* Edge reconf manager improvement to exception handling
* edge reconf manager improvement to exception handling
* Merge branch 'toponav2-devel' of github.com:LCAS/topological_navigation into any_edge_action
* Merge pull request `#70 <https://github.com/LCAS/topological_navigation/issues/70>`_ from adambinch/toponav2-devel
  Edge Reconfigure Improvements
* more efficient
* edge reconfigure manager only resets params that have been reconfigured
* tidying
* Tidying
* Merge pull request `#69 <https://github.com/LCAS/topological_navigation/issues/69>`_ from adambinch/pub_closest_edges
  Planning considering edges when robot current_node = none and topological localisation publishes closest edges to the robot.
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/adambinch-pub_closest_edges
  Planning considering edges when robot current_node = none
* warn to info
* planning ensures that the robot does not goes back to closest node before navigating and that it always navigate from the closest edge when far from any node
* Function for getting distance to edges is much more efficient
* Merge branch 'pub_closest_edges' of https://github.com/adambinch/topological_navigation into adambinch-pub_closest_edges
* tidying
* tidying
* tidying
* tidying
* Topological Localisation publishes closest edges to the robot.
  Publishes the two closest edges to the robot on the topic `/closest_edges`
  with message type `topological_navigation_msgs.msg.ClosestEdges`
  This message has fields for the edge ids and the distances (to the robot) e.g.
  ---
  edge_ids: [WayPoint56_WayPoint66, WayPoint66_WayPoint56]
  distances: [0.3709999918937683, 0.3709999918937683]
  ---
  Often the two edges reported on this topic will form a bi-directional edge.
* Merge pull request `#63 <https://github.com/LCAS/topological_navigation/issues/63>`_ from ayu135/combine_exec_nav
  Combine execute policy and nav actions in a single script
* Added none check for set ended
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/ayu135-combine_exec_nav
  Ayu135 combine exec nav
* correctly cancelling previous goal and waiting before starting the new one
* remove some superfluous lines in preempting nav goals
* Merge branch 'combine_exec_nav' of https://github.com/ayu135/topological_navigation into toponav2-devel
* Merge pull request `#67 <https://github.com/LCAS/topological_navigation/issues/67>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions implementation
* Merge branch 'combine_exec_nav' of https://github.com/ayu135/topological_navigation into ayu135-combine_exec_nav
* 'restrictions_manager' to 'topological_restrictions_manager'
* adding requirement of sympy>=1.5.1
* restriction manager works with runtime and planning restrictions; test script for testing
* WIP restrictions manager
* Merge pull request `#66 <https://github.com/LCAS/topological_navigation/issues/66>`_ from adambinch/melodic-devel
  Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
* if updating node restrictions then apply planning restrictions to edges involving the node.
  Set this behaviour with new boolean arg `update_edges` in srv for updating a node's restrictions
* Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
  Both are boolean sentences (default="True")
  Update restrictions services modified to account for this.
* Better integrate nav and exec policy actions
* Combined execute policy and nav actions in a single script navigation.py
* Removed tmap1 related functions fron nav.py
* Merge pull request `#64 <https://github.com/LCAS/topological_navigation/issues/64>`_ from adambinch/melodic-devel
  Map manager services for updating restrictions
* Map manager services for updating restrictions
  Restrictions field for a node or an edge is now a string which is a boolean sentence (default="True").
  New services `/topological_map_manager2/update_node_restrictions` and `/topological_map_manager2/update_edge_restrictions` added in the map manager 2.
* Merge pull request `#3 <https://github.com/LCAS/topological_navigation/issues/3>`_ from francescodelduchetto/particles-states
  Particles states
* remove modifications to route_search
* Merge pull request `#60 <https://github.com/LCAS/topological_navigation/issues/60>`_ from adambinch/melodic-devel
  Base frame used by localisation is no longer hard coded (in toponav 2).
* file renamed in install targets
* Merge branch 'melodic-devel' of github.com:adambinch/topological_navigation into melodic-devel
* pose pub is replaced with a tf broadcaster. renamed file
* added install target for the new node.
* New node for publishing the map to topomap transform on the topic `/topological_transform` with msg type `geometry_msgs/TransformStamped`
* Base frame used by localisation is no longer hard coded.
  It is set by a rosparam `topological_localisation/base_frame` (default=`base_link`).
  topo_map frame is retrieved from the topological map.
  removed unused imports from localisation.
* Merge pull request `#58 <https://github.com/LCAS/topological_navigation/issues/58>`_ from adambinch/melodic-devel
  removed `use_tmap2` arg from localisation - localisation uses the new…
* removed `use_tmap2` arg from localisation - localisation uses the new format map only.
* Merge pull request `#54 <https://github.com/LCAS/topological_navigation/issues/54>`_ from adambinch/edge_reconf
  Edge reconfigure integration for the new map type
* minor improvement to the edge reconfigure manager
* The edge reconfigure manager is in its own file.
* Cant add duplicate params when using srv `add_param_to_edge_config`
* Fixes and improvements to the edge reconfigure manager.
* Lots of fixes
* EdgeReconfigureManager class done. Needs testing.
* Service `update_edge_config` renamed to `add_param_to_edge_config` to better reflect what it does.
  That service and `rm_param_from_edge_config` modified to account for the changes in the previous commit.
  Constructing new class `EdgeReconfigureManager` in `navigation.py` to handle everything edge reconfigure related.
* topo path planning considers blacklisted nodes
* Edges config is now a list where each item is a dict with the params namespace, name and value.
  The default config is empty and the tmap to tmap2 conversion sets an empty config.
* Service for removing params from an edge's config and a fix.
* service `update_edge_reconf` renamed to `update_edge_config`
* New service for adding/updating edge reconfigure parameters.
* fix
* Function that does the new to old conversion catches exceptions
* `convert_to_legacy` rosparam sets whther the new to old format map conversion happens or not
* map manager 2 coverts new format maps (broadcast on the topic `/topological_map_2`) to the old format (broadcast on the topic `/topological_map`).
  This allows nodes/actions that rely on the old map format to function whilst using/testing features from the new map.
* The arg `use_tmap2` (used by localisation and navigation) is now a rosparam
* Merge pull request `#47 <https://github.com/LCAS/topological_navigation/issues/47>`_ from heuristicus/eband-planner
  Allow use of EBandPlannerROS as local planner
* Some fixes:
  The monitored navigation function in `navigation.py` expects a geom msgs Pose object rather than a monitored nav goal object (stops nav breaking when using the old map format).
  Navigation now reconfigures move base tolerances according to the values specified in the tmap.
* Merge pull request `#45 <https://github.com/LCAS/topological_navigation/issues/45>`_ from ayu135/toponav2-devel
  Added tmap2 support for navigation.py and execute_policy
* added route_search2.py for tmap2 and corresponding changes and fixes
* Added separate navigate and follow route funtions for tmap2
* Added command line option for topomap2
* Some fixes after testing
* Updated map callback for execute policy
* adding support for tmap2 and combining execute policy
* Merge pull request `#44 <https://github.com/LCAS/topological_navigation/issues/44>`_ from adambinch/manager2_srvs
  All manager services available and working on new map type
* Improvement to the function that loads the map
* Correction to srv `/topological_map_manager2/update_edge`
* add max_vel_lin for eband in dynparam mapping
* add eband to dynparam mappings
* When loading a map using the map manager 2, it is cached in `$HOME/.ros/topological_maps`.
  General improvements.
* correction
* reverting accidental change
* minor improvement
* Added srvs `/topological_map_manager2/rm_tag_from_node` and `/topological_map_manager2/update_edge`
* Added srvs `/topological_map_manager2/modify_node_tags` and `/topological_map_manager2/add_tag_to_node`
* Added srvs `/topological_map_manager2/update_node_pose` and `/topological_map_manager2/update_node_tolerance`.
  General improvements.
* Added service `/topological_map_manager2/update_node_name`
* Added service `/topological_map_manager2/add_content_to_node`
* Added services `/topological_map_manager2/remove_topological_node` and `/topological_map_manager2/remove_edge`.
  General improvements.
* Made map manager 2 node more user friendly
  Corrected error when generating influence zone vertices
  removed unnecessary msg definition
  General improvements
* Added services `/topological_map_manager2/add_topological_node` and `/topological_map_manager2/add_edges_between_nodes`
* Made node(`map_manager2.py`) for loading in new format maps using the manager 2 class.
  Added service `/topological_map_manager2/write_topological_map` for writing new format topological maps to yaml files. If you dont specify the path/name of the map then it will just write to the one given to the manager 2 class.
  When loading a tmap (`tmap.tmap`) from a file using the original map manager, the converted tmap can now be written to a file (`tmap.yaml`) using the `write_topological_map` service.
  Added map sanity checking function to the manager 2 class.
* minor changes
* Merge branch 'master' of https://github.com/LCAS/topological_navigation into manager2_srvs
* Created `topological_navigation_msgs` package that will contain the new msg and srv types for the new format topomap.
  Added services `/topological_map_manager2/switch_topological_map` and `/topological_map_manager2/get_edges_between_nodes`.
  Added function in map manager 2 that warns if you are trying to use it to load an old-format topomap.
  Some minor improvements.
* Edge id field included in new map. Default is `origin_destination`
* minor change
* Added manager 2 services:
  - `get_topological_map `
  - `get_tagged_nodes`
  - `get_tags`
  - `get_node_tags`
* correction
* map manager 2 class now publishes the new format topomap, rather than the origin map manager.
  map manager 2 can now load a new format topomap from a given file path.
* Merge pull request `#31 <https://github.com/LCAS/topological_navigation/issues/31>`_ from adambinch/loc2
  All functions in localisation now work with the new map type.
* correction
* corrections
* All functions in localisation can now work with the new map type. This includes its services.
* rearranging
* Argument added to switch between using map types in topological localisation.
  Get nodes with tag service in localisation now works on new map type.
  Map manager 2 now has service for getting nodes with a tag.
  Map manager now adds a nodes tags during map conversion.
* Localisation uses the topo_map to base_link tf transform, rather than the robot pose.
* prettyfying
* Map manager broadcasts map->topo_map tf transform.
* Merge pull request `#29 <https://github.com/LCAS/topological_navigation/issues/29>`_ from adambinch/topomap2
  Function for converting topological maps into the new format in the
* New map type is regenerated and republished when current map is updated.
* bit of tidying
* Map manager keeps its class attribute copy of the new map as a dictionary, but publishes it as a string.
* updated package xml
* minor change
* Function for converting topological maps into the new format in the map manager.
  Includes a map manager 2 class for handling topological maps in the new format.
* Contributors: Adam Binch, Ayush Sharma, Jaime Pulido Fentanes, Michal Staniaszek, adambinch, francescodelduchetto, gpdas

2.1.0 (2020-04-20)
------------------
* Merge pull request `#7 <https://github.com/LCAS/topological_navigation/issues/7>`_ from heuristicus/improve-manager
  Minor quality of life improvements for map_manager
* influence vertices generated by function rather than hardcoded
* goal tolerances are object attribute, close_nodes dist is a parameter
* add message_generation to cmakelists
* Contributors: Jaime Pulido Fentanes, Michal Staniaszek

2.0.0 (2020-04-08)
------------------

1.1.1 (2020-04-08)
------------------
* Merge pull request `#6 <https://github.com/LCAS/topological_navigation/issues/6>`_ from Jailander/master
  Choosing move base action to approach node position following actions…
* Choosing move base action to approach node position following actions order defined in move_base_actions parameter.
  This is very useful to establish priorities across the map
* Merge pull request `#5 <https://github.com/LCAS/topological_navigation/issues/5>`_ from adambinch/fix
  Added reconf at edges server as an install target.
* Added reconf at edges server as an install target.
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from Jailander/master
  Importing original version of topological navigation
* Merge branch 'temp_toponav_only' of ../strands_navigation
* moving all files into correct folder
* Contributors: Jaime Pulido Fentanes, Marc Hanheide, adambinch, jailander

1.1.0 (2019-11-27)
------------------
* Merge pull request `#377 <https://github.com/strands-project/strands_navigation/issues/377>`_ from gpdas/fix_route_search
  Fix route search
* variable name fix
* Merge branch 'indigo-devel' into fix_route_search
* Merge pull request `#376 <https://github.com/strands-project/strands_navigation/issues/376>`_ from gpdas/exec_policy_reconf_edge
  enable edge_reconfig for execute_policy_mode server
* fix TopologicalRouteSearch
  1. As of now, an expanded node (in expanded or to_expand) are not updated when a shorter path to it is found. This is fixed.
  2. Some performance improvements by limiting loop iterations searching for expanded_node
* TopologicalRouteSearch checks origin and target are the same
* enable edge_reconfig for execute_policy_mode server
  1. edge reconfig ported from topological_navigation/navigation.py
  2. minor fixes in
  - topological_navigation/navigation.py
  - topological_navigation/route_search.py
* Contributors: Jaime Pulido Fentanes, gpdas

1.0.8 (2019-06-04)
------------------
* Merge pull request `#374 <https://github.com/strands-project/strands_navigation/issues/374>`_ from Jailander/edge-reconf
  Move base parameters being reconfigured at edges
* Merge pull request `#373 <https://github.com/strands-project/strands_navigation/issues/373>`_ from bfalacerda/indigo-devel
  add local planner arg to single robot topo nav launch
* Merge pull request `#1 <https://github.com/strands-project/strands_navigation/issues/1>`_ from gpdas/edge-reconf
  reconfig_at_edges services added
* update current_edge_group only if reconfig successful
  reconf_at_edges service node now subscribes to param /edge_nav_reconfig_groups (removed relative ns)
* reconfig_at_edges services added
  1. edges_groups param is modified to have the parameter names and values for reconfiguration
  2. added a node in topological_navigation for running the reconf_at_edges service - @adambinch
  3. topological_navigation/navigation.py updated to use the modified param
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into indigo-devel
* add local planner arg to single robot topo nav launch
* Moving reconf server to strands
* reconfiguring when no group (so default option can be used)
* Reverting test
* testing
* Now resetting to the right set of params
* bug fix
* Re-configuring tolerance from latest set of parameters not original set
* adding edge reconfigure manager
* Corrected battery namespaces for localise by topic
* Merge pull request `#369 <https://github.com/strands-project/strands_navigation/issues/369>`_ from strands-project/ori-indigo-devel
  Support for multi-robot and different global planners
* minor changes to work with move_base_flex. defaults should produce backward compatible behaviour still
* Fixed typo and maintaining backward compatibility for policy visualisation
* Corrected indentation
* Merge remote-tracking branch 'ori/indigo-devel' into indigo-devel
  Bringing in changes from ORI for multi-robot and different base planners.
* respawn travel estimator when it dies
* Using correct exception type for dynparam call
* add different color to policy arrows
* Updated top nav execution to handled different types of local planner for move_base.
  Tested under navigation and policy execution, but not extensively.
* Minimal topological navigation config with no extra dependencies and no monitored nav recoveries
* top nav supports other planners for dynparam. still need to update policy exec
* making topo nav feedback more robutst to possible lag in localisation - fetch issues
* making sure number of fails gets reset after the fail threshold is reached
* make code less contrived
* correct feedback publishing from topo nav
* multi-robot setup
* update of absolute/relative topic names for multi-robot setup
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Marc Hanheide, Nick Hawes, gpdas

1.0.7 (2018-10-26)
------------------
* Temporarily disabling Morse-based tests (`#360 <https://github.com/strands-project/strands_navigation/issues/360>`_)
* Contributors: Jaime Pulido Fentanes

1.0.6 (2018-07-17)
------------------
* Merge pull request `#358 <https://github.com/strands-project/strands_navigation/issues/358>`_ from Jailander/rasberry-devel
  re-adding ability to work with other planners
* Fixes bug on service call for adding node
* re-adding ability to work with other planners
* Revert "Revert "Revert "Adding the ability to work with local planners other than DWA"""
  This reverts commit b0ea99543615e6dfc8dbb2cb9969ce1da6ae546c.
* Revert "Fixing bug on add node service marker"
  This reverts commit 0a364cbfda27ea5971eeb871e286cfd186ceca1c.
* Revert "Revert "Adding the ability to work with local planners other than DWA""
  This reverts commit e11a93bf79b01e17889eb3e00750b8f588385f93.
* Revert "Adding the ability to work with local planners other than DWA"
  This reverts commit b86ca393944362eb9c0cf21884810f5c0f8862e2.
* Fixing bug on add node service marker
* Adding the ability to work with local planners other than DWA
* Contributors: Jaime Pulido Fentanes

1.0.5 (2018-04-17)
------------------
* add speed based prediction to install scripts
* Merge pull request `#342 <https://github.com/strands-project/strands_navigation/issues/342>`_ from bfalacerda/predictions
  optimistic nav predictions until 10 samples
* Merge pull request `#351 <https://github.com/strands-project/strands_navigation/issues/351>`_ from heuristicus/indigo-devel
  Can now place nodes with RMB to stop automatic edge creation
* Merge pull request `#352 <https://github.com/strands-project/strands_navigation/issues/352>`_ from heuristicus/patch-2
  Ensure that meta out is defined to prevent crashes
* Ensure that meta out is defined to prevent crashes
* Can now place nodes with RMB to stop automatic edge creation
  Fix deletion dialogue, edges and tags were swapped
* Merge pull request `#350 <https://github.com/strands-project/strands_navigation/issues/350>`_ from heuristicus/patch-1
  Fix crash on attempting to change node name
* Fix crash on attempting to change node name
* Merge pull request `#349 <https://github.com/strands-project/strands_navigation/issues/349>`_ from mudrole1/indigo-devel
  Adding waiting for the add_node service
* Removed loadMap() in the delete method
* optimistic predictions until 10 samples
* Merge branch 'prediction-hacking' of https://github.com/Jailander/strands_navigation into predictions
* creating optimistic predictions
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Lenka Mudrova, Michal Staniaszek, Nick Hawes

1.0.4 (2017-06-23)
------------------
* Modifications to topological map tools to accommodate topological map editor (`#345 <https://github.com/strands-project/strands_navigation/issues/345>`_)
  * fix weird space-colon
  * Easier translational movement of waypoints, generic node field updater
  Moving the waypoints that are displayed in the topological map in rviz is now
  easier - just uses 2D planar motion as opposed to multiple handles for the x and
  y dimensions.
  Added a function which calls into the database to update any property of a node.
  * Fixed not loading map after update, correctly updates edges on node rename
  This should really not be the file being used - it seems like the one in util is
  used to change things and as such is more up to date.
  * remove unnecessary if
  * update function for edge action and top_vel
  * add deprecation warnings to topological_map.py - should use manager.py instead
  * start on work to make manager services more useful for modifying map
  * add callback for getting tags for a specific node
  * partial switch to the using manager, updating and adding tags
  * fix message fields and add messages to generation
  * small script to insert empty map into a database
  * add edge removal service
  * change callbacks so that functions can be called without service
* Update README.md
* Contributors: Jaime Pulido Fentanes, Michal Staniaszek

1.0.3 (2017-01-11)
------------------
* now the actions in the edges of the topological map have different colours, the markers have namespaces and there is a legend with the colours and the action names
* Implementing formula for keeping probabilities of under explored edge… (`#336 <https://github.com/strands-project/strands_navigation/issues/336>`_)
  * Implementing formula for keeping probabilities of under explored edges higher
  * Stats on same topic and not builiding fremen models when first topological map arrives
  * fixing bug in previous PR
* fixing bug in previous PR
* Stats on same topic and not builiding fremen models when first topological map arrives
* Implementing formula for keeping probabilities of under explored edges higher
* Topological prediction now works properly with map switching and using nav_stats only when models have been created
* moving localisation subscriber to map callback
* adding new action to move_base type actions and making it a param
* removing prints
* now models will be updated as robot navigates (model building is still necessary one in a while)
* Contributors: Jaime Pulido Fentanes, jailander

1.0.2 (2016-10-31)
------------------
* bug fix
* fixes localise by topic and conflicts
* Revert "2lbtfix"
* forcing check of localise byt topic
* makes sense
* now it will draw topological map despite of missing nodes for edges
* changing default values for model building params and setting params
* fixing nav stats
* changing default values
* now the parameters `/topological_prediction/success_values` and `/topological_prediction/fail_values` and be used to set the values considered for failures and successes
* Contributors: Jaime Pulido Fentanes

1.0.1 (2016-06-21)
------------------
* Removing Prints from topological prediction
* Printing debug info and attempt to fix eternal retry problem
* adding move base as a run dependency on topological_navigation
* Contributors: Jaime Pulido Fentanes

1.0.0 (2016-06-09)
------------------
* adding move base as a run dependency on topological_navigation (`#315 <https://github.com/strands-project/strands_navigation/issues/315>`_)
* Contributors: Jaime Pulido Fentanes

0.0.45 (2016-06-06)
-------------------
* removed race condition, but this really needs a better fix
* Contributors: Nick Hawes

0.0.44 (2016-05-30)
-------------------
* Added install for new script.
* Cleaned up a bit.
* Added simple node to report manually provided edge predictions from a yaml file.
* Adding Fremenserver monitors to topological prediction
* Contributors: Jaime Pulido Fentanes, Nick Hawes

0.0.43 (2016-05-25)
-------------------
* Merge pull request `#300 <https://github.com/strands-project/strands_navigation/issues/300>`_ from bfalacerda/indigo-devel
  allowing setting of max bumper recoveries param at startup
* Improving sampling for topological prediction
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into indigo-devel
* Changing a priori entropy
* bug fix (introduced by copy paste)
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into indigo-devel
  Conflicts:
  topological_navigation/scripts/localisation.py
* 0.0.42
* updated changelogs
* Removing lambda function
* calling the instance does not return anything. appending to list first and the calling.
* Making localise by topic wait for the topic to be published
* 0.0.41
* updated changelogs
* Adding localise_pose service which returns the node and closest node for a pose.
  And fixing conflicts
* fixing a priory entropies and probabilities and tidy up code
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into tsc-deployment
* making a priory probabilities 1 and considering non fatal as successful.
* Making navigation nodes respawnable
* Implementing service lock for topological prediction
* Added ability to load dummy maps from yaml
* Monkey patching localisation by topic to wait longer between polls
* Using more standard waypoint names to fit with other systems
* printing messages for debugging
* allowing setting of max bumper recoveries param at startup
* Contributors: Bruno Lacerda, Christian Dondrup, Jaime Pulido Fentanes, Jenkins, Nick Hawes, jailander

0.0.42 (2016-03-21)
-------------------
* Removing lambda function
* calling the instance does not return anything. appending to list first and the calling.
* Making localise by topic wait for the topic to be published
* Contributors: Christian Dondrup

0.0.41 (2016-03-03)
-------------------

0.0.40 (2016-02-07)
-------------------
* prediction of traversal duration using speeds that are properly fremenised
* adding policy visualisation
* prediction changes
* policies visualisation
* Contributors: Jaime Pulido Fentanes

0.0.39 (2016-01-28)
-------------------
* removing annoying print
* print warning when no route to node
* Impossible tests now require the navigation to fail on its own accord
  Currently, the impossible tests, i.e., blocking the way or the final node, require that the graceful death attempt is successful, meaning that the robot is able to navigate back to start after the navigation to end failed. With this PR, a new field for the service is added, giving feedback if the navigation timed out or if it failed on its own accord. Impossible tests are therefore only passed, if the navigation failed without timing out and if graceful death was successful.
* now execute policy server when it can't reach the position of the final node
* If the path or final waypoint is completely blocked the test will succeed if the robot is able to fail gracefully.
* Removing support for dynamic human tests. These have been postponed in simulation.
* Adding more tests with humans blocking waypoints.
* making sure topological navigation fails when it should
* Adding description of new tests and how to create a topo map that uses the passive morse objects added to readme.
* Change in test files assuming that maps always are prefixed with `mb_test` and just append a number for the correct one.
* * Adding obstacle nodes
  * Making sure that position injection worked
  * Adding untested support for dynamic human tests by playing a bag file and positioning the human correctly.
  * Other minor improvements
* Using new mba_test builder script for simulation to also include passive objects as obstacles.
* Update README.md
* Contributors: Christian Dondrup, Jaime Pulido Fentanes, Marc Hanheide

0.0.38 (2015-11-17)
-------------------
* Updating readme
* Correcting output
* Changing to degrees and unregeistering robot_pose callback when not needed.
* Bugfix and adding output to screen for new control
* Adding joypad control
* Adding displaying of the distance in meters and radians to the actual position in the tha map after reaching the node.
* fixing copy and paste error
* Calli8ng services to enable freerun and reenable motors in case of bumper hit or barrier stop.
* Fixing faulty wait for message for button press.
* Adding missing return and using if and unless in map_dir arg due to roslaunch bugs/features
* Inserting maps if map_dir is given
* Making map directory for topological maps a parameter.
* Adding robot specific reset function.
* Dividing tests into critical and supplementary. Only critical tests are run on jenkins and supplementary tests can be run to test navigation parameters. See README.
* Adding install targets for test and get_simple_policy script.
  Adding correct description of how to run tests in README
* Undoing installing tests directory. This needs a little more thought to make it work.
* Adding a readme for the navigation tests
* Installing test directory
* Adding argument robot to test launch file to be able to run only the essentials on the robot.
* Only try to load maps from strands_morse if run in simulation. strands_morse might not be installed on the robot.
* Giving tests speaking names
* Exposing retries parameter for topological navigation via launch files.
* Exposing execute_policy_retries via launch files
* Removing unnecessary dependencies and adding some prints.
* Adds the first version of the simulation only unit-test for topological_navigation/move_base.
* Extending the load yaml map functionality. Now based on a class in topological navigation to prevent circular test dependencies.
* Removing annoying print statement
* Revert "Adding first version of topological test scenarios"
* Adding install targets for test and get_simple_policy script.
  Adding correct description of how to run tests in README
* Undoing installing tests directory. This needs a little more thought to make it work.
* Adding a readme for the navigation tests
* Installing test directory
* Adding argument robot to test launch file to be able to run only the essentials on the robot.
* Only try to load maps from strands_morse if run in simulation. strands_morse might not be installed on the robot.
* Giving tests speaking names
* Exposing retries parameter for topological navigation via launch files.
* Exposing execute_policy_retries via launch files
* Removing unnecessary dependencies and adding some prints.
* Adds the first version of the simulation only unit-test for topological_navigation/move_base.
* Extending the load yaml map functionality. Now based on a class in topological navigation to prevent circular test dependencies.
* Removing annoying print statement
* this should fix the race condition permanently
* waiting for reconfigure services for 50 seconds before continuing. should avoid race condition
* making number of tries a parameter
* how embarrassing ...
* avoiding race condition in execute policy server by waiting for topological localisation before publitising the action server
* solving silly race condition
* adding simple policy generation based on A*
* now you can launch topological navigation with an empty map (meaning no nodes)
* safety commit
* adding services for adding and deleting nodes
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into move-base-testing
* creating move base testing branch
* Various fixes and code cleaning in topological map visualiser
* now the topological map name param is set by the map manager and not by navigation
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into map-edition-fixes
* minor fixes
* Contributors: Christian Dondrup, Jaime Pulido Fentanes, Nick Hawes

0.0.37 (2015-08-26)
-------------------
* Fixed bug in dummy map where origin and ChargingPoint names were mixed up.
* getting rid of nasty error
* Fixing Visualisation of policies
* creating edge_entropy service
* Added window range to action message. If this is left blank in the goal the behaviour is as before
* Does duration prediction based on mean of data.
* Speed-based duration predictor for single edges
* adding the possibility of limiting the stats used for the predictions by time range
* output to screen
* map drawing utilities
* making sure the number of messages needed for persist is consecutive
* Update README.md
* including persistency check on localise by topic, and localise_anywhere is
  now configurable on the localise by topic string
* Contributors: Jailander, Jaime Pulido Fentanes, Nick Hawes

0.0.36 (2015-05-17)
-------------------
* Added the wait_reset_bumper_duration to top_nav.launch
* if localised by topic assume as current node no matter pose
* removing speed reconfiguration in topological navigation, this is messing with the walking group speeds, there should be something smarter like in policy execution
* Contributors: Jaime Pulido Fentanes, Nils Bore

0.0.35 (2015-05-10)
-------------------
* forcing the creation of move_base reconfigure client even when there are no move_base edges on the topological map
* sorting nodes by name when calling `/topological_map_publisher/get_topological_map` service
* Creating Reconfigure Client only for needed actions and handling not available reconfigure clients
* fix for localise by topic where localisation by topic is only verified once the robot has moved more than 10 cm away from the pose it first detected the topic on
* reconfigure using move base on non-move_base type action
* Adding reconfigure Client depending on edge action
* reconfiguring speed and removing move_base to closest node
* Contributors: Jaime Pulido Fentanes

0.0.34 (2015-05-05)
-------------------
* Adding boolean to tell topological navigation not to care for orientation in the final node
* fixing bug with repeated edges in prediction, and adding test for this case in test top prediction
* reconfiguring move_base yaw tolerance depending on next action if its move_base type to 2*PI if its none to the default node tolerance and if it is a non move_base type to 30 degrees
* Contributors: Jaime Pulido Fentanes

0.0.32 (2015-04-12)
-------------------
* emergency behaviours launch file
* updating service list when most services will be needed
* Adding Emergency Behaviours
* fixing action server bug
* Contributors: Jaime Pulido Fentanes

0.0.31 (2015-04-10)
-------------------
* fixing issues tested
* typo
* changing prints to rospy.loggerr
* Improving error handling
* adding service to get tagged nodes ordered by distance and minor bug fix on topological navigation
* Policy execution doesn't do move_base to the waypoint when the waypoint is localised by topic
* localisation by topic only works if the robot is in the influence zone of the node, migrate script now adds JSON string for localisation on ChargingPoint
* Implementing Localise By topic and No go nodes exceptions
* Topological prediction now uses forecast service
* Improving time estimation
* returning only edge_id in topological prediction
* Fixing issues with topological Prediction
* second part of previous commit
* checking sanity on migrate scripts
* Topological navigation doesn't use nasty old Classes anymore
* adding search route script
* Contributors: Jaime Pulido Fentanes

0.0.29 (2015-03-23)
-------------------

0.0.28 (2015-03-20)
-------------------

0.0.27 (2015-03-19)
-------------------
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_navigation into indigo-devel
  Conflicts:
  topological_navigation/CMakeLists.txt
* Adding topological map editor launch file,
  replacing map_publisher with map manager,
  adding add_node service
* adding edit mode to visualise
* fixing typo
* sending the robot to waypoint when in the influence area of the target node
* making sure robot executes action when reaching node in policy execution
* Navigation and policy_executor working with new defs
* bug fixes
* adding Get Topological Map service
* new branch created
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes

0.0.26 (2015-03-18)
-------------------
* Forgot the install targets
* Contributors: Nick Hawes

0.0.25 (2015-03-18)
-------------------
* Renamed to .py to be consistent.
* Contributors: Nick Hawes

0.0.24 (2015-03-17)
-------------------

0.0.23 (2014-12-17)
-------------------

0.0.22 (2014-11-26)
-------------------
* Got the speed more correct.
* Fixing typo, also now the top loc will check for the influence area of the two closest nodes instead of just the closest
* removing docking from action that are allowed so the robot navigates to closest node that now is never ChargingStation
* adding ChargingPoint exception to localisation
* Triying Docking when Charging station is the closest node
* Fixing indentation
* Bug Fix with inc variable not being set on special cases
* Contributors: Nick Hawes, STRANDS user on Pablo-PC

0.0.21 (2014-11-23)
-------------------
* Merge branch 'hydro-devel' of https://github.com/Jailander/strands_navigation into hydro-devel
* error handling when no route is possible
* adding sleep to reduce cpu consumption
* Contributors: Jaime Pulido Fentanes

0.0.20 (2014-11-21)
-------------------
* replcaing result for nav_ok
* Contributors: Jaime Pulido Fentanes

0.0.19 (2014-11-21)
-------------------
* typo
* Contributors: Jaime Pulido Fentanes

0.0.18 (2014-11-21)
-------------------
* bug fix
* Now checking if there is a move_base action in the edges of the first node
  in route if not it's dangerous to move or inconvenient
  like in the charging station
* Contributors: Jaime Pulido Fentanes

0.0.17 (2014-11-21)
-------------------
* catching reconfigur move_base exception
* only increase the fail counter of monitored navigation if result.recovered is True and result.human_interaction is False as suggested by @BFALacerda
* fixing bug with an even longer if
* Contributors: Jaime Pulido Fentanes

0.0.16 (2014-11-21)
-------------------
* removinf scitos_msgs from CmakeLists
* making robot navigate to Way Point always when the first action is not move_base type
* Added locking to service call.
* removing old dependency on scitos_msgs from top nav
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Nick Hawes

0.0.15 (2014-11-19)
-------------------

0.0.14 (2014-11-19)
-------------------
* Update README.md
* Contributors: Jaime Pulido Fentanes

0.0.12 (2014-11-17)
-------------------

0.0.11 (2014-11-14)
-------------------
* bug fix
* Contributors: Jaime Pulido Fentanes

0.0.10 (2014-11-14)
-------------------
* replanning when failing
* Adding retries to topological navigation and current edge publisher
* Update README.md
* Contributors: Jaime Pulido Fentanes

0.0.9 (2014-11-12)
------------------
* Merge pull request `#120 <https://github.com/strands-project/strands_navigation/issues/120>`_ from BFALacerda/hydro-devel
  adding monitored_nav to topological_navigation.launch.
* adding monitored_nav to topological_navigation.launch. default is monitored_nav without recovery behaviours
* Contributors: BFALacerda, Bruno Lacerda

0.0.8 (2014-11-11)
------------------

0.0.6 (2014-11-06)
------------------
* Corrected install locations.
* Contributors: Nick Hawes

0.0.5 (2014-11-05)
------------------
* adding joystick creation of topological map
* Added dummy script to stand in for topological navigation when missing a robot or proper simulation.
  Useful for testing.
* Adding licences and bug fix
* Added launch file for test, and test passing locally.
* Moved Vertex and Edge into strands_navigation_msgs.
  Basic test for travel_time_tester passes.
* Added travel_time_estimator to standard launch file.
* Merge topological_navigation and topological_map_manager packages.
  Added the EstimateTravelTime service to provide a clean way of getting travel times of the topological map.
* Contributors: Jaime Pulido Fentanes, Nick Hawes

0.0.4 (2014-10-30)
------------------

0.0.3 (2014-10-29)
------------------
* Merge pull request `#94 <https://github.com/strands-project/strands_navigation/issues/94>`_ from Jailander/hydro-devel
  fixing mongodb_store deps
* fixing mongodb_store deps
* Contributors: Jaime Pulido Fentanes, Marc Hanheide

0.0.2 (2014-10-29)
------------------
* 0.0.1
* added changelogs
* stupid me
* bug fix
* adding launch files to install targets
* Adding install targets
* Adding Missing TopologicalMap.msg and changing maintainer emails, names and Licences for Packages
* Adding Execute Policy server to topological_navigation.launch
* This version saves some basic navigation stats and has some additional comments important for documentation
* making sure feedback is only published once per new waypoint visited
* Adding comments and small debug
* Moving and renaming Execute Policy Action
* adding some sleeps to reduce computing load
* solving current_route error
* fixing abortion an shutdown
* adding on shutdown actions and aborting when no edge is found
* adding number of tries before aborting
* other bug fix
* fixing stupid typo
* Making sure it navigates to the next waypoint when next action is not move_base type
* back to unknown nodes at start
* bug fix 3
* removing request for outcome
* bug fix
* making the robot navigate to waypoint when next action is not move_base and it has previously failed
* Making robot navigate closest edge when not at node
* Navigating to closest node when finishing at none
* debugging 2
* printf for debugging
* testing
* setting as aborted when failed
* Including human_aware_navigation as a move_base action on policy execution_server
* Committing Execute policy server
* adding sending new goals when node Iz is reached
* Fixes bugs created by name changes of mongodb_store and moving packages between repositories
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
  Conflicts:
  topological_navigation/scripts/localisation.py
  topological_navigation/scripts/navigation.py
* adding comment
* scitos_ramp_climb is now ramp_climb
* scitos_apps_msgs has been removed.
  All the imports were unused anyway.
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to be merged first.
* bug fix
* Adding add Node controller
* Adapting Interactive Markers on Topological Map Manager to use the topological Map Publisher
  and bug fixes.
  *WARNING: Still requires a lot of testing*
* Topological navigation now uses topological map publisher
* adding topological map publisher and adapting localisation node to use it
* adding scripts to topological utils
* adding new visualization node to launch file
* Merge pull request `#69 <https://github.com/strands-project/strands_navigation/issues/69>`_ from BFALacerda/hydro-devel
  log of monitored nav events + improvements applied during g4s deployment
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
* Publishing edge move via goal feedback
* Adding Topological_map_manager
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
* Now action server returns cancelled when the target node is not found on the map
* outputting success imediately when source and target node are the same, when the action is not a "normal" navigtion action
* now it is possible to edit the influence zones from rviz
* fixing orientation reconfiguration for human aware navigation
* Adding machine tags to launch files
* now cancelling monitored navigation when top nav is preempted
* Fixing bug on topological navigation server preemption
* Minor bug fix Error Message should not appear any longer
* Not cancelling monitored navigation goal when topological navigation produces output on Node_to_IZ mode
* Adding Node_to_IZ
* printing available data too
* Added Warning when 0 or more than 1 waypoints match query for updating
* Small fix in topological map
* Now Topological Maps are stored in the topological_map collection
* Now is possible to move waypoints in Rviz using interactive marker and they will be updated on the ros_datacentre
* Making move_base care for orientation when next action is not move_base and Fixing bug when PREEMPTED
* Adding topological map python class and edges marker array for visualisation of the topological map in Rviz
* Fixing statistics bug
* Preempting topological navigation when monitored navigation is preempted
* Adding pointset to _meta information for Navigation statistics
* Merge pull request `#32 <https://github.com/strands-project/strands_navigation/issues/32>`_ from Jailander/hydro-devel
  Using Message store proxy to store statistics and Message Name Change
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
* Commit now vertex and Edge messages are capitalised, node message was moved to strands_navigation message
  Using Message store proxy to store statistics
* Added param broadcast for topological map name.
* Topological Navigation now works using message store proxy
* changing topic name
* Now publishes statistics over ros topic /TopologicalNavigation/Statistics and bug fixes
* Update package.xml
* Update CMakeLists.txt
* adding monitored navigation to topological navigation
* adding node message and move base reconfigure
* last changes on groovy version
* Adding Topological Map field to recorded statistics
* Update README.md
* Added statistics logging to mongo_db
* Logging Navigation statistics
* Adding Localisation using polygonal influence areas
* Adding Topological_Utils to repository
* Update README.md
* Update README.md
* minor changes
* Update README.md
* Changes in file structure and names
* Update README.md
* Create README.md
* reducing computational load for testing overshooting bug on Linda
* Fixing bug when target and Origin Point were the same node
* Adding Topological localisation
* Very minor changes
* adding topological navigation
* Contributors: Bruno Lacerda, Christian Dondrup, Jaime Pulido Fentanes, Marc Hanheide, Nick Hawes
