^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_rviz_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

2.4.0 (2022-01-25)
------------------
* Merge pull request `#111 <https://github.com/magnucha/topological_navigation/issues/111>`_ from adambinch/remove_strands_dependencies
  Removing strands navigation dependencies from topological navigation.
* update
* strands dependencies removed from topological_rviz_tools
* Merge branch 'master' of github.com:LCAS/topological_navigation into francescodelduchetto-toponav2-restrictions
* Merge branch 'master' of github.com:LCAS/topological_navigation into toponav2_launch
* Merge branch 'master' of github.com:LCAS/topological_navigation into faster_route_search2
* Merge branch 'toponav2-devel-restrictions' of github.com:francescodelduchetto/topological_navigation into toponav2-devel
* Merge branch 'master' of https://github.com/adambinch/topological_navigation into adam-master
* Contributors: Adam Binch, adambinch, francescodelduchetto

2.3.0 (2021-07-15)
------------------
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#82 <https://github.com/LCAS/topological_navigation/issues/82>`_ from adambinch/fix_conflicts
  Fix conflicts
* Merge branch 'master' of github.com:LCAS/topological_navigation into fix_conflicts
  # Conflicts:
  #	topological_navigation/scripts/execute_policy_server.py
  #	topological_navigation/scripts/navigation.py
* Contributors: Adam Binch, adambinch

* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#82 <https://github.com/LCAS/topological_navigation/issues/82>`_ from adambinch/fix_conflicts
  Fix conflicts
* Merge branch 'master' of github.com:LCAS/topological_navigation into fix_conflicts
  # Conflicts:
  #	topological_navigation/scripts/execute_policy_server.py
  #	topological_navigation/scripts/navigation.py
* Contributors: Adam Binch, adambinch

2.1.0 (2020-04-20)
------------------
* Merge pull request `#8 <https://github.com/LCAS/topological_navigation/issues/8>`_ from heuristicus/move_topo_rviz_tools
  Move topo rviz tools from strands_navigation
* added install for topmap interface
* 1.2.0
* updated changelogs
* 1.1.0
* changelogs
* 1.0.8
* updated changelogs
* 1.0.7
* updated changelogs
* added install for extra dirs (`#365 <https://github.com/LCAS/topological_navigation/issues/365>`_)
* Standalone easier to run on strands robots
  Would have to run with rviz=false and then run rviz on the main PC if the
  database is on on of the side PCs. With this change can run the database on the
  side PC and then run this with launch_db:=false to not have to run rviz
  separately
* 1.0.6
* updated changelogs
* 1.0.5
* updated changelogs
* update of absolute/relative topic names for multi-robot setup
* more descriptive names for topics displayed in rviz
* Can now place nodes with RMB to stop automatic edge creation
  Fix deletion dialogue, edges and tags were swapped
* Adding waiting for services
* Update topological_edge_tool.cpp
* Adding waiting for the add_node service
* Update strands_rviz_topmap.launch
* 1.0.4
* updated changelogs
* set version to 1.0.3 as the rest of repository
* add standalone flag for when navigation is running
* Topmap editor (`#344 <https://github.com/LCAS/topological_navigation/issues/344>`_)
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
* Contributors: Bruno Lacerda, Jaime Pulido Fentanes, Jenkins, LCAS build farm, Lenka Mudrova, Marc Hanheide, Michal Staniaszek, Nick Hawes
