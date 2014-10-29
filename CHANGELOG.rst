^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
