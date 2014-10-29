^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
