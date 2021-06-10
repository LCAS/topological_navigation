Topological Navigation
======================

**These instructions apply only to the legacy branch of topological navigation. Instructions for the master branch (topolgical navigation 2) are to do.**
This node provides support for topological navigation in the STRANDS system. 

This module requires:
 * move_base
 * strands_datacentre
 * monitored_navigation

You will also need to create a [topological map](#Create-Topological-Maps)

## Launching Topological Localisation and Navigation

1. Once your map is inserted in the DB you can launch your topological navigation nodes using:
  `roslaunch topological_navigation topological_navigation.launch map:=topological_map_name node_by_node:=false`

*note:* When Node by node navigation is active the robot will cancel the topological navigation when it reaches the influence zone of a node and it will navigate to its waypoint aborting the action.

*note:* Statistics are being recorded in the nav_stats collection within the autonomous_patrolling

*note:* every action server for the actions stored in the topological map have to be running, for example if the ramp_climb action is required you will need the ramp_climb server running, you can run it using:
`rosrun ramp_climb ramp_climb`


## Navigate

1. You can test the topological navigation using:
  `rosrun topological_navigation nav_client.py Destination`
  Where destination is the name of the node you want to reach.

2. You can also run **RViz** `rosrun rviz rviz` and subscribe to the interactive markers `/name_of_map_go_to/update` and clicking on the Green Arrows to move the robot around.


#Create Topological Maps

There are different ways of creating **topological maps**:

1. [Simultaneous Metric and Topological Mapping](https://github.com/strands-project/strands_navigation/tree/hydro-devel/joy_map_saver)

2. [From WayPoint File](#creation-of-the-topological-map-from-waypoint-file)

3. [Using RViz](#Using-RViz)

4. [Using Gamepad](#Using-Gamepad)

5. [Edit using MongoDB Client](#Edit-using-MongoDB-Client)

*. [Using Localise by Topic] (#Using-Localise-by-Topic)


## Simultaneous Metric and Topological Mapping

@To improve

To follow this process you will need to place the robot in the Charging station and reset the odometry, once it is there you can launch the mapping by running:

`roslaunch topological_utils mapping.launch met_map:='name_of_metric_map' top_map:='name_of_topological_map'`

Once this is done undock the robot using the Visual_charging_server. 

When the robot is undocked you can move it around using the gamepad and when you wish to create a new topological node press button **'B'** once you are done save your metric map by pressing **'A'**.

To edit the topological map you can use the following methods


## Creation of the Topological map From WayPoint File

1. (**Modified**) The first step is to create the waypoint file by running:
 `rosrun topological_utils joy_add_waypoint.py name_of_the_waypoint_file.csv`

1. Next create a topological map file from the waypoint file using:
`rosrun topological_utils tmap_from_waypoints.py input_file.csv output_file.tmap` this will create a topological tree on which every waypoint in the file is a node and each waypoint is connected to every other waypoint using move_base and an octagonal influence area, for example,

  From a normal waypoint file like this:
  ```STON
  -2.3003,0.120533,-4.88295e-05,0,0,-0.0530802,0.99859
  -7.95369,-3.03503,-4.4791e-05,0,0,0.928319,-0.371784
  -8.91935,2.94528,0.00139425,0,0,0.473654,0.880711
  -2.68708,-2.23003,-0.000478224,0,0,-0.759842,0.650108
  ```
  It will create a topological map file like this:
  ```JSON
  node: 
  	ChargingPoint
  	waypoint:
  		-2.3003,0.120533,-4.88295e-05,0,0,-0.0530802,0.99859
  	edges:
  		WayPoint1, move_base
  		WayPoint2, move_base
  		WayPoint3, move_base
  	vertices:
        1.380000,0.574000
        0.574000,1.380000
        -0.574000,1.380000
        -1.380000,0.574000
        -1.380000,-0.574000
        -0.574000,-1.380000
        0.574000,-1.380000
        1.380000,-0.574000
  node: 
  	WayPoint1
  	waypoint:
  		-7.95369,-3.03503,-4.4791e-05,0,0,0.928319,-0.371784
  	edges:
  		ChargingPoint, move_base
  		WayPoint2, move_base
  		WayPoint3, move_base
  	vertices:
        1.380000,0.574000
        0.574000,1.380000
        -0.574000,1.380000
        -1.380000,0.574000
        -1.380000,-0.574000
        -0.574000,-1.380000
        0.574000,-1.380000
        1.380000,-0.574000
  node: 
  	WayPoint2
  	waypoint:
  		-8.91935,2.94528,0.00139425,0,0,0.473654,0.880711
  	edges:
  		ChargingPoint, move_base
  		WayPoint1, move_base
  		WayPoint3, move_base
  	vertices:
        1.380000,0.574000
        0.574000,1.380000
        -0.574000,1.380000
        -1.380000,0.574000
        -1.380000,-0.574000
        -0.574000,-1.380000
        0.574000,-1.380000
        1.380000,-0.574000
  node: 
  	WayPoint3
  	waypoint:
  		-2.68708,-2.23003,-0.000478224,0,0,-0.759842,0.650108
  	edges:
  		ChargingPoint, move_base
  		WayPoint1, move_base
  		WayPoint2, move_base
  	vertices:
        1.380000,0.574000
        0.574000,1.380000
        -0.574000,1.380000
        -1.380000,0.574000
        -1.380000,-0.574000
        -0.574000,-1.380000
        0.574000,-1.380000
        1.380000,-0.574000

  ```
1. *Editing the topological map*, lets say that you want to make sure that to reach a node you get there using a specific action like ramp_climb from another node then you will need to edit the topological map file. You can also edit the vertices of the influence area so it can be defined using different kinds of polygons.
  For example, lets suppose that in the previous map you want WayPoint3 to be reached only from the Waypoint1 using `ramp_climb` and that you want to define rectangular areas around your waypoints and a triangular area around the ChargingPoint, in such case your map file should look like this:

  ```JSON
  node: 
  	ChargingPoint
  	waypoint:
  		-2.3003,0.120533,-4.88295e-05,0,0,-0.0530802,0.99859
  	edges:
  		 WayPoint1, move_base
  		 WayPoint2, move_base
  	vertices:
        0.0,1.5
        -1.5,-1.5
        1.5,-1.5
  node: 
  	WayPoint1
  	waypoint:
  		-7.95369,-3.03503,-4.4791e-05,0,0,0.928319,-0.371784
  	edges:
  		ChargingPoint, move_base
  		WayPoint2, move_base
  		WayPoint3, ramp_climb
  	vertices:
        1.380000,0.574000
        -1.380000,0.574000
        -1.380000,-0.574000
        1.380000,-0.574000
  node: 
  	WayPoint2
  	waypoint:
  		-8.91935,2.94528,0.00139425,0,0,0.473654,0.880711
  	edges:
  		ChargingPoint, move_base
  		WayPoint1, move_base
  	vertices:
        0.574000,1.380000
        -0.574000,1.380000
        -0.574000,-1.380000
        0.574000,-1.380000
  node: 
  	WayPoint3
  	waypoint:
  		-2.68708,-2.23003,-0.000478224,0,0,-0.759842,0.650108
  	edges:
  		WayPoint1, ramp_climb
  	vertices:
        1.5,1.5
        -1.5,1.5
        -1.5,-1.5
        1.5,-1.5
  ```
  In this case the topological navigation node will always call ramp_climb when reaching or leaving WayPoint3, you can also edit the names of the nodes to fit your needs.
  *note:* At the moment only `move_base` and `ramp_climb` can be included.
  *note:* The polygons of the influence area will be created using the convex-hull approach so they may not correspond exactly to the vertices previously defined.
  
1. Once you are happy with your topological map you have to insert it in the strands_datacentre using:
  `rosrun topological_utils insert_map.py topological_map.tmap topological_map_name map_name`

## Using RViz 

**RViz** can be used to edit topological maps on system runtime, however there is also the possibility of inserting a basic *topological map* and editing it using *RViz*. The Following steps will guide through this process.

1. It is necessary to insert a basic map in the DB and run the topological system based on it for this a launch file is provided and it can be used in the following way:

  * Launch 2d navigation using for example:
  `roslaunch strands_movebase movebase.launch map:=/path/to/your/map.yaml with_chest_xtion:=false`
  
  * Launch empty topological map:
  `roslaunch topological_navigation topological_navigation_empty_map.launch map:='name_of_topological_map'`


This will create a basic map and run the topological system with it.

2. Once this is done you can run **Rviz**, `rosrun rviz rviz` and create two *marker array* interfaces `/topological_node_zones_array` and `/topological_edges_array` optionally an additional *marker array* interface can be created `/topological_nodes_array`. This will allow the visualization of the topological map.

3. After this the map can be edited using interactive markers:

  * **Adding Nodes:** for this objective there is one interactive marker topic called `/name_of_your_map/add_rm_node/update` that will put a green box on top of the robot. Drive the robot where you want to add a new node and press on the green box that will add a node there connected to all close by nodes by a *move_base* action.

  * **Editing Node Position:** with `/name_of_your_map_markers/update`  it is possible to move the waypoints around and change their final goal orientation.

  * **Removing Unnecessary Edges:** to remove edges there is another marker `/name_of_your_map_edges/update` that will allow you to remove edges by pressing on the red arrows.

  * **Change the Influence Zones:** Finally it is possible to change the influence zones with `/name_of_your_map_zones/update` and moving the vertices of such zones around.


## Using Gamepad

This method is basically the same as the previous method (follow steps 1 and 2, also 3 if/when needed) with the addition that in this case it is possible to add nodes at the current robot position by pressing button **'B'** on the Gamepad.


## Edit using MongoDB Client

@TODO

## Using Localise by Topic

Localise by topic is a [JSON](http://json.org/) string defined in the topological map which is empty by default `"localise_by_topic" : ""` which means that the robot will use its pose to obtained its location on the topological map. 

However in some specific cases it might be necessary to localise the robot by means of an specific topic in this cases the localise by topic string should be defined with the following **mandatory** fields 

- *topic*: the name of the topic that will be used for localisation
- *field*: the field of the topic that will be compared
- *val*: the value of *field* that will be used to set the current node (`/current_node`)

A typical localise by topic string will look like this:

```JSON
"{"topic":"battery_state","field":"charging","val":true}"
```

There are also some **optional** fields that can be set:

- *localise_anywhere*: (default value **true**) If set to **true** topological localisation will assume the robot to be at the node whenever localisation by topic corresponds to the values on the string, else this will only happen when the robot is in the influence area of the specific node.
- *persistency*: (default value **10**) is number of consecutive messages with the correct value that must arrive before localisation by topic is determined
- *timeout*: @todo

* Please note: when localised by topic is active the robot will never assume this node to be **closest node**  unless it is also **current node**
