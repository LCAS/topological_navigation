^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2018-07-17)
------------------

1.0.5 (2018-04-17)
------------------
* Merge pull request `#349 <https://github.com/strands-project/strands_navigation/issues/349>`_ from mudrole1/indigo-devel
  Adding waiting for the add_node service
* Fixed two arguments
* Contributors: Lenka Mudrova, Nick Hawes

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
* Contributors: Michal Staniaszek

1.0.3 (2017-01-11)
------------------

1.0.2 (2016-10-31)
------------------
* makes sense
* now the parameters `/topological_prediction/success_values` and `/topological_prediction/fail_values` and be used to set the values considered for failures and successes
* Contributors: Jaime Pulido Fentanes

1.0.1 (2016-06-21)
------------------
* really fixing start now
* Contributors: Nick Hawes

1.0.0 (2016-06-09)
------------------
* More sensible starting point
* Contributors: Nick Hawes

0.0.45 (2016-06-06)
-------------------

0.0.44 (2016-05-30)
-------------------

0.0.43 (2016-05-25)
-------------------
* Using pointset rather than map name.
* 0.0.42
* updated changelogs
* 0.0.41
* updated changelogs
* Using predictions for edge times
* Added ability to load dummy maps from yaml
* Better feedback timing as required by mdp exec.
* Aborting axserver on failure
* Simulating policy execution better.
* Contributors: Jenkins, Nick Hawes

0.0.42 (2016-03-21)
-------------------

0.0.41 (2016-03-03)
-------------------
* removing map name from query
* Contributors: Jaime Pulido Fentanes

0.0.40 (2016-02-07)
-------------------
* adding missing install targets
* prediction changes
* Contributors: Jailander, Jaime Pulido Fentanes

0.0.39 (2016-01-28)
-------------------
* removing prints and repeated node
* Fixes in topological utils
* Contributors: Jaime Pulido Fentanes

0.0.38 (2015-11-17)
-------------------
* Extending the load yaml map functionality. Now based on a class in topological navigation to prevent circular test dependencies.
* Revert "Adding first version of topological test scenarios"
* Extending the load yaml map functionality. Now based on a class in topological navigation to prevent circular test dependencies.
* now you can launch topological navigation with an empty map (meaning no nodes)
* removing edge analysis
* Removed unnecessary import
* safety commit
* creating move base testing branch
* fixes on map exporting scripts
* minor fixes
* Contributors: Christian Dondrup, Jaime Pulido Fentanes, Nick Hawes

0.0.37 (2015-08-26)
-------------------
* Fixed bug in dummy map where origin and ChargingPoint names were mixed up.
* Fix edge renaming.
* Fix node name check.
* Add utility to check map for errors.
* Add basic argument checking.
* Add utiltiy to automate renaming of map nodes.
* adding options for rotating and scaling the map and timezone management
* drawing maps in an epoch range
* coding expected speeds
* Compiles and visualises data based on nav predictions vs ground truth.
* added map_manager to  create script
* added policy and prediction stuff to dummy system
* Added script to print out count of nav stats per edge
* removing unwanted file
* drawing predicted map
* map drawing utilities
* Contributors: Jailander, Jaime Pulido Fentanes, Nick Hawes, Rares Ambrus

0.0.36 (2015-05-17)
-------------------

0.0.35 (2015-05-10)
-------------------

0.0.34 (2015-05-05)
-------------------
* Oops, that was almost embarrassing.
* Dummy system now sets top map name param.
* fixing insert yaml
* Contributors: Jaime Pulido Fentanes, Nick Hawes

0.0.32 (2015-04-12)
-------------------
* fixing bug in insert map that I inserted myself
* Contributors: Jaime Pulido Fentanes

0.0.31 (2015-04-10)
-------------------
* localisation by topic only works if the robot is in the influence zone of the node, migrate script now adds JSON string for localisation on ChargingPoint
* Fixing issues with topological Prediction
* second part of previous commit
* checking sanity on migrate scripts
* Contributors: Jaime Pulido Fentanes

0.0.29 (2015-03-23)
-------------------
* adding install targets
* Contributors: Jaime Pulido Fentanes

0.0.28 (2015-03-20)
-------------------
* removed scripts/LoadPointSet.py from install
* Contributors: Marc Hanheide

0.0.27 (2015-03-19)
-------------------
* sending the robot to waypoint when in the influence area of the target node
* removing pointset b testing
* commiting migrate script plus typo fix
* map to Json utilities
* fixing bug by which undocking edge was not being created
* bug fixes
* Now waypoint to yaml automatically Includes ChargingPoint
* tmap_to_yaml.py now includes default values for edges
* Navigation and policy_executor working with new defs
* New map format export and insertion scripts
* committing map creation script
* Adding recovery behaviours to edges
* new branch created
* Contributors: Jailander, Jaime Pulido Fentanes

0.0.26 (2015-03-18)
-------------------
* Forgot the install targets
* Contributors: Nick Hawes

0.0.25 (2015-03-18)
-------------------
* Added the option to simulate time as an argument to the file.
* Renamed to .py to be consistent.
* Contributors: Nick Hawes

0.0.24 (2015-03-17)
-------------------
* Fix in map to yaml
* Added a boolean value indicating whether the returned nodes are actual nodes in the topological map
* Clean up
* Print message
* Clean up
* returning nodes based on the mongodb node metadata
* Adding scripts for new file format
* Added map name to the service message
* Returning random data
* Adding topological node metadata query service - initial commit
* Added better handling of time for dummy navigation.
* Add list maps utility.
* Contributors: Chris Burbridge, Jailander, Nick Hawes, Rares Ambrus

0.0.23 (2014-12-17)
-------------------

0.0.22 (2014-11-26)
-------------------

0.0.21 (2014-11-23)
-------------------

0.0.20 (2014-11-21)
-------------------
* moving scripts here
* Contributors: Jaime Pulido Fentanes

0.0.19 (2014-11-21)
-------------------

0.0.18 (2014-11-21)
-------------------

0.0.17 (2014-11-21)
-------------------

0.0.16 (2014-11-21)
-------------------

0.0.15 (2014-11-19)
-------------------
* fixing bug in top_map
* Contributors: Jaime Pulido Fentanes

0.0.14 (2014-11-19)
-------------------
* adding new launch files for topological map creation
* Contributors: Jaime Pulido Fentanes

0.0.12 (2014-11-17)
-------------------

0.0.11 (2014-11-14)
-------------------

0.0.10 (2014-11-14)
-------------------
* mapping launch files
* replanning when failing
* fixing influence areas on empty map
* Contributors: Jaime Pulido Fentanes

0.0.9 (2014-11-12)
------------------

0.0.8 (2014-11-11)
------------------

0.0.6 (2014-11-06)
------------------
* Corrected install locations.
* Contributors: Nick Hawes

0.0.5 (2014-11-05)
------------------
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_navigation into hydro-devel
  Conflicts:
  topological_utils/CMakeLists.txt
* adding install targets
* adding joystick creation of topological map
* Added launch file for dummy topological navigation and install targets.
* Added dummy script to stand in for topological navigation when missing a robot or proper simulation.
  Useful for testing.
* Adding licences and bug fix
* Moved Vertex and Edge into strands_navigation_msgs.
  Basic test for travel_time_tester passes.
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
* Adding install targets
* including visualization_msgs in package xml to sort `#83 <https://github.com/strands-project/strands_navigation/issues/83>`_
* Adding Missing TopologicalMap.msg and changing maintainer emails, names and Licences for Packages
* scitos_apps_msgs has been removed.
  All the imports were unused anyway.
* Renamed datacentre_ rosparams to mongodb_
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to be merged first.
* Adding add Node controller
* adding scripts to topological utils
* Adding Topological_map_manager
* now it is possible to edit the influence zones from rviz
* Adding an script for exporting the map to a text file
* Now Station is connected to WayPoint1 through `undocking`
  ... not `docking`
* Improved waypoint to tmap script
  Now when creating the topological map from a waypoint file it will add a
  Charging node (ChargingPoint) at position {0,0,0,0,0,0,0}
  (this waypoint can't be on the waypoint file) and this node will
  be conected to the first waypoint in the file only using the
  docking action
* Adding Node_to_IZ
* Small fix in topological map
* Now Topological Maps are stored in the topological_map collection
* Now is possible to move waypoints in Rviz using interactive marker and they will be updated on the ros_datacentre
* Adding topological map python class and edges marker array for visualisation of the topological map in Rviz
* Adding interactive markers to visualization
* Adding visualise_map.py tool
* adding max distance for edge creation between topological nodes
* Commit now vertex and Edge messages are capitalised, node message was moved to strands_navigation message
  Using Message store proxy to store statistics
* Topological Navigation now works using message store proxy
* adding node message and move base reconfigure
* preliminary switch to ros_datacentre
* Adding Topological_Utils to repository
* Contributors: Bruno Lacerda, Christian Dondrup, Jaime Pulido Fentanes, Marc Hanheide, Nick Hawes
