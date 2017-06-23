^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_rviz_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* set version to 1.0.3 as the rest of repository
* Topmap editor (`#344 <https://github.com/strands-project/strands_navigation/issues/344>`_)
  * Initial commit
  * initial slightly modified clone of plantflag tutorial
  * Add slightly modified clone of rviz views panel
  * Plugin and tool now load properly
  * Shuffling things around, taking code from existing rviz property
  Looks like properties are the best way to look at things from rviz. Kind of a
  rudimentary approximation of messages. Not sure of the best way to transfer data
  back to the map once we want to change it, but that will come once we can view
  the map data in the panel.
  * finally have compiling base classes to modify
  * renamed classes
  * fix function call parens
  * panel displays some default values
  * More work on getting nodes and edges represented in the panel
  * Rearrange property orderings
  Property ordering is
  nodecontroller
  ^
  |
  nodeproperty (per node)
  ^      ^
  /        \
  poseproperty   edgecontroller
  ^
  |
  edgeproperty (per edge)
  Controllers are intended as a way of adding nodes and edges from the topological
  map while keeping everything contained somewhat nicely in its own class
  * Finally linked to topmap topic
  Still not able to see edges. They are being added, but not displayed in the
  panel. Something to do with the edge controller?
  * Now properly displays edges in each node
  EdgeController was missing the base class initialisation from its constructor.
  * slots for node updates connected
  * Start on python interface node to allow access to topmap modification
  Add custom rviz config to use for topmap modification, plus launch file
  * update launch to allow proper viewing of topmap
  * Now correctly refreshes on change, constructors filled in
  Very inefficient - repopulates all properties when anything in the map changes.
  Constructors all use delete. This should be changed later to not use pointers.
  Currently has a segfault when a pose property is modified, think it's because
  the current node is deleted and there is still something else referencing it.
  * First attempt at not replacing all properties when one is modified
  Connected slot in node property to node controller to notify on modification of
  xy threshold and yaw etc, need to do the same for pose and edge properties and
  see if knowing which node was changed can help fix the problem.
  * finally able to modify poses without crash
  * fix only partial deletion of properties on new map
  * fix weird space-colon
  * Easier translational movement of waypoints, generic node field updater
  Moving the waypoints that are displayed in the topological map in rviz is now
  easier - just uses 2D planar motion as opposed to multiple handles for the x and
  y dimensions.
  Added a function which calls into the database to update any property of a node.
  * linked name changing
  * Prevent possibility of node name duplication
  Also added resets to the previous value of the property when a service call
  fails, so that the properties reflect the actual values.
  * Now possible to add and remove nodes via rviz
  * can now use the edge tool to add edges
  * Fixed not loading map after update, correctly updates edges on node rename
  This should really not be the file being used - it seems like the one in util is
  used to change things and as such is more up to date.
  * Adding nodes now done via tool
  Click tool, then click on map to add node. Add shortcuts for edge tool (e), and
  node tool (n).
  * Fix edge property name, bidirectional with left click
  Also fix node tool disabling
  * use dummy navigation
  * remove unnecessary if
  * Show arrow when creating edge, disallow edges to self
  * rename package and namespace
  * updated launch file and rviz config
  * update import in interface script, add db path to args
  * update function for edge action and top_vel
  * allow edge property editing for action and top_vel
  * add localise by topic to node prop (read only)
  * add deprecation warnings to topological_map.py - should use manager.py instead
  * start on work to make manager services more useful for modifying map
  * initial work on tags for nodes
  still needs work in the manager to retrieve tags for specific nodes
  * add callback for getting tags for a specific node
  * hide tag controller property if node has no tags
  * partial switch to the using manager, updating and adding tags
  * tag addition and modification, move to manager.py in progress
  started moving topological map update to the panel rather than node controller
  so we can decide whether to update or not more easily
  * fix message fields and add messages to generation
  * redirect most calls to manager rather than interface
  Removed/moved messages to strands navigation msgs so that the manager can
  perform all required tasks
  Map updates triggered in the topological map panel as opposed to at the node
  level
  * move to subdirectory in preparation for PR
  * small script to insert empty map into a database
  * more sensible paths
  * add edge removal service
  * allow removal of tags and edges from panel
  * add confirmation dialog for remove button
  * add readme
  * little more in the readme
  * nodes in panel sorted, fix occasional segfault due iterate/delete
  * change callbacks so that functions can be called without service
  * update edge and update tolerance now pass both params
  * Readme mentions standalone flag
  * add note about using tools in arbitrary rviz session
  * try and stop compilation issue with AddEdge not being found
  * add dependency on the project messages to library generation
* Contributors: Marc Hanheide, Michal Staniaszek

* set version to 1.0.3 as the rest of repository
* Topmap editor (`#344 <https://github.com/strands-project/strands_navigation/issues/344>`_)
  * Initial commit
  * initial slightly modified clone of plantflag tutorial
  * Add slightly modified clone of rviz views panel
  * Plugin and tool now load properly
  * Shuffling things around, taking code from existing rviz property
  Looks like properties are the best way to look at things from rviz. Kind of a
  rudimentary approximation of messages. Not sure of the best way to transfer data
  back to the map once we want to change it, but that will come once we can view
  the map data in the panel.
  * finally have compiling base classes to modify
  * renamed classes
  * fix function call parens
  * panel displays some default values
  * More work on getting nodes and edges represented in the panel
  * Rearrange property orderings
  Property ordering is
  nodecontroller
  ^
  |
  nodeproperty (per node)
  ^      ^
  /        \
  poseproperty   edgecontroller
  ^
  |
  edgeproperty (per edge)
  Controllers are intended as a way of adding nodes and edges from the topological
  map while keeping everything contained somewhat nicely in its own class
  * Finally linked to topmap topic
  Still not able to see edges. They are being added, but not displayed in the
  panel. Something to do with the edge controller?
  * Now properly displays edges in each node
  EdgeController was missing the base class initialisation from its constructor.
  * slots for node updates connected
  * Start on python interface node to allow access to topmap modification
  Add custom rviz config to use for topmap modification, plus launch file
  * update launch to allow proper viewing of topmap
  * Now correctly refreshes on change, constructors filled in
  Very inefficient - repopulates all properties when anything in the map changes.
  Constructors all use delete. This should be changed later to not use pointers.
  Currently has a segfault when a pose property is modified, think it's because
  the current node is deleted and there is still something else referencing it.
  * First attempt at not replacing all properties when one is modified
  Connected slot in node property to node controller to notify on modification of
  xy threshold and yaw etc, need to do the same for pose and edge properties and
  see if knowing which node was changed can help fix the problem.
  * finally able to modify poses without crash
  * fix only partial deletion of properties on new map
  * fix weird space-colon
  * Easier translational movement of waypoints, generic node field updater
  Moving the waypoints that are displayed in the topological map in rviz is now
  easier - just uses 2D planar motion as opposed to multiple handles for the x and
  y dimensions.
  Added a function which calls into the database to update any property of a node.
  * linked name changing
  * Prevent possibility of node name duplication
  Also added resets to the previous value of the property when a service call
  fails, so that the properties reflect the actual values.
  * Now possible to add and remove nodes via rviz
  * can now use the edge tool to add edges
  * Fixed not loading map after update, correctly updates edges on node rename
  This should really not be the file being used - it seems like the one in util is
  used to change things and as such is more up to date.
  * Adding nodes now done via tool
  Click tool, then click on map to add node. Add shortcuts for edge tool (e), and
  node tool (n).
  * Fix edge property name, bidirectional with left click
  Also fix node tool disabling
  * use dummy navigation
  * remove unnecessary if
  * Show arrow when creating edge, disallow edges to self
  * rename package and namespace
  * updated launch file and rviz config
  * update import in interface script, add db path to args
  * update function for edge action and top_vel
  * allow edge property editing for action and top_vel
  * add localise by topic to node prop (read only)
  * add deprecation warnings to topological_map.py - should use manager.py instead
  * start on work to make manager services more useful for modifying map
  * initial work on tags for nodes
  still needs work in the manager to retrieve tags for specific nodes
  * add callback for getting tags for a specific node
  * hide tag controller property if node has no tags
  * partial switch to the using manager, updating and adding tags
  * tag addition and modification, move to manager.py in progress
  started moving topological map update to the panel rather than node controller
  so we can decide whether to update or not more easily
  * fix message fields and add messages to generation
  * redirect most calls to manager rather than interface
  Removed/moved messages to strands navigation msgs so that the manager can
  perform all required tasks
  Map updates triggered in the topological map panel as opposed to at the node
  level
  * move to subdirectory in preparation for PR
  * small script to insert empty map into a database
  * more sensible paths
  * add edge removal service
  * allow removal of tags and edges from panel
  * add confirmation dialog for remove button
  * add readme
  * little more in the readme
  * nodes in panel sorted, fix occasional segfault due iterate/delete
  * change callbacks so that functions can be called without service
  * update edge and update tolerance now pass both params
  * Readme mentions standalone flag
  * add note about using tools in arbitrary rviz session
  * try and stop compilation issue with AddEdge not being found
  * add dependency on the project messages to library generation
* Contributors: Marc Hanheide, Michal Staniaszek

1.0.3 (2017-01-11)
------------------

1.0.2 (2016-10-31)
------------------

1.0.1 (2016-06-21)
------------------

1.0.0 (2016-06-09)
------------------

0.0.45 (2016-06-06)
-------------------

0.0.44 (2016-05-30)
-------------------

0.0.43 (2016-05-25)
-------------------

0.0.42 (2016-03-21)
-------------------

0.0.41 (2016-03-03)
-------------------

0.0.40 (2016-02-07)
-------------------

0.0.39 (2016-01-28)
-------------------

0.0.38 (2015-11-17)
-------------------

0.0.37 (2015-08-26)
-------------------

0.0.36 (2015-05-17)
-------------------

0.0.35 (2015-05-10)
-------------------

0.0.34 (2015-05-05)
-------------------

0.0.33 (2015-04-22)
-------------------

0.0.32 (2015-04-12)
-------------------

0.0.31 (2015-04-10)
-------------------

0.0.30 (2015-03-31)
-------------------

0.0.29 (2015-03-23)
-------------------

0.0.28 (2015-03-20)
-------------------

0.0.27 (2015-03-19)
-------------------

0.0.26 (2015-03-18 23:00)
-------------------------

0.0.25 (2015-03-18 22:28)
-------------------------

0.0.24 (2015-03-17)
-------------------

0.0.23 (2014-12-17)
-------------------

0.0.22 (2014-11-26)
-------------------

0.0.21 (2014-11-23)
-------------------

0.0.20 (2014-11-21 20:11)
-------------------------

0.0.19 (2014-11-21 17:47)
-------------------------

0.0.18 (2014-11-21 17:40)
-------------------------

0.0.17 (2014-11-21 16:22)
-------------------------

0.0.16 (2014-11-21 09:38)
-------------------------

0.0.15 (2014-11-19 12:02)
-------------------------

0.0.14 (2014-11-19 08:48)
-------------------------

0.0.13 (2014-11-18)
-------------------

0.0.12 (2014-11-17)
-------------------

0.0.11 (2014-11-14 19:49)
-------------------------

0.0.10 (2014-11-14 11:30)
-------------------------

0.0.9 (2014-11-12)
------------------

0.0.8 (2014-11-11 14:06)
------------------------

0.0.7 (2014-11-11 10:44)
------------------------

0.0.6 (2014-11-06)
------------------

0.0.5 (2014-11-05)
------------------

0.0.4 (2014-10-30)
------------------

0.0.3 (2014-10-29 17:00)
------------------------

0.0.2 (2014-10-29 15:05)
------------------------
