^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
