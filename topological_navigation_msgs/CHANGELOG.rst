^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topological_navigation_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2022-01-25)
------------------
* Merge pull request `#133 <https://github.com/magnucha/topological_navigation/issues/133>`_ from adambinch/melodic-devel
  Map manager improvements
* update
* update
* update
* Adding new msg and srv files
* Adding new msg files
* Separate srv for setting influence zone.
  Caching the map and auto saving after service calls are both optional via ros params.
* Service /add_topological_node has option to add node vertices
* Merge pull request `#116 <https://github.com/magnucha/topological_navigation/issues/116>`_ from adambinch/final_things
  A few final things.
* A few final things.
  Set `advertise_srvs` arg to False when initialising the map manager 2 allows other scripts/nodes to use its functions without advertising 20+ services.
  Descriptions of fail policy actions added to `UpdateFailPolicy.srv`.
  Tidying.
* Merge pull request `#114 <https://github.com/magnucha/topological_navigation/issues/114>`_ from adambinch/melodic-devel
  Corrected inaccurate description of the `not_fluid` arg in `UpdateEdge.srv`
* Minor changes
* Merge pull request `#111 <https://github.com/magnucha/topological_navigation/issues/111>`_ from adambinch/remove_strands_dependencies
  Removing strands navigation dependencies from topological navigation.
* strands dependencies removed from `topological_navigation/topological_utils`
* update
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
* Removed strands nav msgs from localisation
* Legacy map manager no longer dependent on strands nav msgs
* Moving topomap msgs from strands nav to toponav repo. Map manager 2 is now strands independent.
* Removing strands navigation dependencies from the Toponav repo.
* Removing strands navigation dependencies from TopoNav.
  Copying srv definitions used by toponav from strands nav to the toponav repo.
* Merge pull request `#107 <https://github.com/magnucha/topological_navigation/issues/107>`_ from adambinch/node_names
  Resolves Issue `#90 <https://github.com/magnucha/topological_navigation/issues/90>`_, Adds datum to the tmap meta, and other things.
* Service `/topological_map_manager2/update_edge` replaces `/topological_map_manager2/update_edge_action`
  setting the same args with an additional boolean arg for setting whether navigation is fluid or not.
  Uses srv type `topological_navigation_msgs.srv.UpdateEdge`
* Map manager service for adding GNSS latitude/longitude to the topological map's top-level meta info
* Merge branch 'master' of github.com:LCAS/topological_navigation into node_names
* Merge pull request `#78 <https://github.com/magnucha/topological_navigation/issues/78>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions
* Merge branch 'master' of github.com:LCAS/topological_navigation into francescodelduchetto-toponav2-restrictions
* remove satisfy_runtime_restrictions code and comment some prints
* do not call runtime_restriction but rather use an ad-hoc flag for 'fluid_navigation' in the tmap
* move action status topic now has std msg type String
* Status of move action moved from go to node action definition to its own topic /topological_navigation/move_action_status.
  Msg definition is topological_navigation_msgs/MoveActionStatus
* Merge branch 'master' of github.com:LCAS/topological_navigation into toponav2_launch
* adding possibility of satisfying runtime restrictions, not tested yet
* Merge branch 'master' of github.com:LCAS/topological_navigation into faster_route_search2
* adding services to evaluate single nodes/edges and exactPose restriction
* Merge branch 'toponav2-devel-restrictions' of github.com:francescodelduchetto/topological_navigation into toponav2-devel
* restriction manager works with runtime and planning restrictions; test script for testing
* Merge branch 'melodic-devel' of https://github.com/adambinch/topological_navigation into adam_melodic-devel
* WIP kinda of works
* WIP restrictions manager
* Contributors: Adam Binch, adambinch, francescodelduchetto

2.3.0 (2021-07-15)
------------------
* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#85 <https://github.com/LCAS/topological_navigation/issues/85>`_ from adambinch/melodic-devel
  Map manager services for updating edge action, type and goal.
* map manager service for setting the action, action type and goal for an edge
  map manager service for setting the action type and goal for any edge with a given action
* Merge pull request `#57 <https://github.com/LCAS/topological_navigation/issues/57>`_ from LCAS/toponav2-devel
  Topological Navigation version 2 Master Branch
* Merge pull request `#75 <https://github.com/LCAS/topological_navigation/issues/75>`_ from adambinch/any_edge_action
  Topological navigation can handle any type of goal.
* New manager 2 srv for updating the action type of each edge in the tmap according to the action name
* Merge pull request `#69 <https://github.com/LCAS/topological_navigation/issues/69>`_ from adambinch/pub_closest_edges
  Planning considering edges when robot current_node = none and topological localisation publishes closest edges to the robot.
* Merge branch 'pub_closest_edges' of https://github.com/adambinch/topological_navigation into adambinch-pub_closest_edges
* Topological Localisation publishes closest edges to the robot.
  Publishes the two closest edges to the robot on the topic `/closest_edges`
  with message type `topological_navigation_msgs.msg.ClosestEdges`
  This message has fields for the edge ids and the distances (to the robot) e.g.
  ---
  edge_ids: [WayPoint56_WayPoint66, WayPoint66_WayPoint56]
  distances: [0.3709999918937683, 0.3709999918937683]
  ---
  Often the two edges reported on this topic will form a bi-directional edge.
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/ayu135-combine_exec_nav
  Ayu135 combine exec nav
* Merge pull request `#67 <https://github.com/LCAS/topological_navigation/issues/67>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions implementation
* restriction manager works with runtime and planning restrictions; test script for testing
* WIP restrictions manager
* Merge pull request `#66 <https://github.com/LCAS/topological_navigation/issues/66>`_ from adambinch/melodic-devel
  Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
* if updating node restrictions then apply planning restrictions to edges involving the node.
  Set this behaviour with new boolean arg `update_edges` in srv for updating a node's restrictions
* Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
  Both are boolean sentences (default="True")
  Update restrictions services modified to account for this.
* Merge pull request `#64 <https://github.com/LCAS/topological_navigation/issues/64>`_ from adambinch/melodic-devel
  Map manager services for updating restrictions
* Map manager services for updating restrictions
  Restrictions field for a node or an edge is now a string which is a boolean sentence (default="True").
  New services `/topological_map_manager2/update_node_restrictions` and `/topological_map_manager2/update_edge_restrictions` added in the map manager 2.
* Merge pull request `#54 <https://github.com/LCAS/topological_navigation/issues/54>`_ from adambinch/edge_reconf
  Edge reconfigure integration for the new map type
* Service `update_edge_config` renamed to `add_param_to_edge_config` to better reflect what it does.
  That service and `rm_param_from_edge_config` modified to account for the changes in the previous commit.
  Constructing new class `EdgeReconfigureManager` in `navigation.py` to handle everything edge reconfigure related.
* service `update_edge_reconf` renamed to `update_edge_config`
* New service for adding/updating edge reconfigure parameters.
* Merge pull request `#44 <https://github.com/LCAS/topological_navigation/issues/44>`_ from adambinch/manager2_srvs
  All manager services available and working on new map type
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
* Created `topological_navigation_msgs` package that will contain the new msg and srv types for the new format topomap.
  Added services `/topological_map_manager2/switch_topological_map` and `/topological_map_manager2/get_edges_between_nodes`.
  Added function in map manager 2 that warns if you are trying to use it to load an old-format topomap.
  Some minor improvements.
* Contributors: Adam Binch, Ayush Sharma, Jaime Pulido Fentanes, adambinch, francescodelduchetto

* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#85 <https://github.com/LCAS/topological_navigation/issues/85>`_ from adambinch/melodic-devel
  Map manager services for updating edge action, type and goal.
* map manager service for setting the action, action type and goal for an edge
  map manager service for setting the action type and goal for any edge with a given action
* Merge pull request `#57 <https://github.com/LCAS/topological_navigation/issues/57>`_ from LCAS/toponav2-devel
  Topological Navigation version 2 Master Branch
* Merge pull request `#75 <https://github.com/LCAS/topological_navigation/issues/75>`_ from adambinch/any_edge_action
  Topological navigation can handle any type of goal.
* New manager 2 srv for updating the action type of each edge in the tmap according to the action name
* Merge pull request `#69 <https://github.com/LCAS/topological_navigation/issues/69>`_ from adambinch/pub_closest_edges
  Planning considering edges when robot current_node = none and topological localisation publishes closest edges to the robot.
* Merge branch 'pub_closest_edges' of https://github.com/adambinch/topological_navigation into adambinch-pub_closest_edges
* Topological Localisation publishes closest edges to the robot.
  Publishes the two closest edges to the robot on the topic `/closest_edges`
  with message type `topological_navigation_msgs.msg.ClosestEdges`
  This message has fields for the edge ids and the distances (to the robot) e.g.
  ---
  edge_ids: [WayPoint56_WayPoint66, WayPoint66_WayPoint56]
  distances: [0.3709999918937683, 0.3709999918937683]
  ---
  Often the two edges reported on this topic will form a bi-directional edge.
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/ayu135-combine_exec_nav
  Ayu135 combine exec nav
* Merge pull request `#67 <https://github.com/LCAS/topological_navigation/issues/67>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions implementation
* restriction manager works with runtime and planning restrictions; test script for testing
* WIP restrictions manager
* Merge pull request `#66 <https://github.com/LCAS/topological_navigation/issues/66>`_ from adambinch/melodic-devel
  Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
* if updating node restrictions then apply planning restrictions to edges involving the node.
  Set this behaviour with new boolean arg `update_edges` in srv for updating a node's restrictions
* Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
  Both are boolean sentences (default="True")
  Update restrictions services modified to account for this.
* Merge pull request `#64 <https://github.com/LCAS/topological_navigation/issues/64>`_ from adambinch/melodic-devel
  Map manager services for updating restrictions
* Map manager services for updating restrictions
  Restrictions field for a node or an edge is now a string which is a boolean sentence (default="True").
  New services `/topological_map_manager2/update_node_restrictions` and `/topological_map_manager2/update_edge_restrictions` added in the map manager 2.
* Merge pull request `#54 <https://github.com/LCAS/topological_navigation/issues/54>`_ from adambinch/edge_reconf
  Edge reconfigure integration for the new map type
* Service `update_edge_config` renamed to `add_param_to_edge_config` to better reflect what it does.
  That service and `rm_param_from_edge_config` modified to account for the changes in the previous commit.
  Constructing new class `EdgeReconfigureManager` in `navigation.py` to handle everything edge reconfigure related.
* service `update_edge_reconf` renamed to `update_edge_config`
* New service for adding/updating edge reconfigure parameters.
* Merge pull request `#44 <https://github.com/LCAS/topological_navigation/issues/44>`_ from adambinch/manager2_srvs
  All manager services available and working on new map type
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
* Created `topological_navigation_msgs` package that will contain the new msg and srv types for the new format topomap.
  Added services `/topological_map_manager2/switch_topological_map` and `/topological_map_manager2/get_edges_between_nodes`.
  Added function in map manager 2 that warns if you are trying to use it to load an old-format topomap.
  Some minor improvements.
* Contributors: Adam Binch, Ayush Sharma, Jaime Pulido Fentanes, adambinch, francescodelduchetto

* Merge branch 'master' of github.com:LCAS/topological_navigation into tmap_to_tmap2
* Merge pull request `#85 <https://github.com/LCAS/topological_navigation/issues/85>`_ from adambinch/melodic-devel
  Map manager services for updating edge action, type and goal.
* map manager service for setting the action, action type and goal for an edge
  map manager service for setting the action type and goal for any edge with a given action
* Merge pull request `#57 <https://github.com/LCAS/topological_navigation/issues/57>`_ from LCAS/toponav2-devel
  Topological Navigation version 2 Master Branch
* Merge pull request `#75 <https://github.com/LCAS/topological_navigation/issues/75>`_ from adambinch/any_edge_action
  Topological navigation can handle any type of goal.
* New manager 2 srv for updating the action type of each edge in the tmap according to the action name
* Merge pull request `#69 <https://github.com/LCAS/topological_navigation/issues/69>`_ from adambinch/pub_closest_edges
  Planning considering edges when robot current_node = none and topological localisation publishes closest edges to the robot.
* Merge branch 'pub_closest_edges' of https://github.com/adambinch/topological_navigation into adambinch-pub_closest_edges
* Topological Localisation publishes closest edges to the robot.
  Publishes the two closest edges to the robot on the topic `/closest_edges`
  with message type `topological_navigation_msgs.msg.ClosestEdges`
  This message has fields for the edge ids and the distances (to the robot) e.g.
  ---
  edge_ids: [WayPoint56_WayPoint66, WayPoint66_WayPoint56]
  distances: [0.3709999918937683, 0.3709999918937683]
  ---
  Often the two edges reported on this topic will form a bi-directional edge.
* Merge pull request `#2 <https://github.com/LCAS/topological_navigation/issues/2>`_ from francescodelduchetto/ayu135-combine_exec_nav
  Ayu135 combine exec nav
* Merge pull request `#67 <https://github.com/LCAS/topological_navigation/issues/67>`_ from francescodelduchetto/toponav2-restrictions
  Toponav2 restrictions implementation
* restriction manager works with runtime and planning restrictions; test script for testing
* WIP restrictions manager
* Merge pull request `#66 <https://github.com/LCAS/topological_navigation/issues/66>`_ from adambinch/melodic-devel
  Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
* if updating node restrictions then apply planning restrictions to edges involving the node.
  Set this behaviour with new boolean arg `update_edges` in srv for updating a node's restrictions
* Nodes and edges have two restrictions fields, one for planning restrictions and one for runtime restrictions.
  Both are boolean sentences (default="True")
  Update restrictions services modified to account for this.
* Merge pull request `#64 <https://github.com/LCAS/topological_navigation/issues/64>`_ from adambinch/melodic-devel
  Map manager services for updating restrictions
* Map manager services for updating restrictions
  Restrictions field for a node or an edge is now a string which is a boolean sentence (default="True").
  New services `/topological_map_manager2/update_node_restrictions` and `/topological_map_manager2/update_edge_restrictions` added in the map manager 2.
* Merge pull request `#54 <https://github.com/LCAS/topological_navigation/issues/54>`_ from adambinch/edge_reconf
  Edge reconfigure integration for the new map type
* Service `update_edge_config` renamed to `add_param_to_edge_config` to better reflect what it does.
  That service and `rm_param_from_edge_config` modified to account for the changes in the previous commit.
  Constructing new class `EdgeReconfigureManager` in `navigation.py` to handle everything edge reconfigure related.
* service `update_edge_reconf` renamed to `update_edge_config`
* New service for adding/updating edge reconfigure parameters.
* Merge pull request `#44 <https://github.com/LCAS/topological_navigation/issues/44>`_ from adambinch/manager2_srvs
  All manager services available and working on new map type
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
* Created `topological_navigation_msgs` package that will contain the new msg and srv types for the new format topomap.
  Added services `/topological_map_manager2/switch_topological_map` and `/topological_map_manager2/get_edges_between_nodes`.
  Added function in map manager 2 that warns if you are trying to use it to load an old-format topomap.
  Some minor improvements.
* Contributors: Adam Binch, Ayush Sharma, Jaime Pulido Fentanes, adambinch, francescodelduchetto

2.2.0 (2020-11-25)
------------------

2.1.0 (2020-04-20)
------------------

2.0.0 (2020-04-08 23:43)
------------------------

1.1.1 (2020-04-08 22:56)
------------------------
