^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
