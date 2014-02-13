Topological Navigation
======================

This node provides support for topological navigation in the STRANDS system. 

This module requires:
 * move_base
 * strands_datacentre
 * scitos_ramp_climb


## Creation of the Topological map

The first step is to insert a topological map on the strands_datacentre, this can be done from a waypoint file created by the waypoint recorder (see https://github.com/strands-project/autonomous_patrolling), following these simple steps:

1. First create a topological map file from the waypoint file using:
`rosrun topological_utils tmap_from_waypoints.py input_file.csv output_file.txt` this will create a topological tree on which every waypoint in the file is a node and each waypoint is conected to every other waypoint using move_base and an octagonal influence area, for example,

  From a normal waypoint file like this:
  ```
  -2.3003,0.120533,-4.88295e-05,0,0,-0.0530802,0.99859
  -7.95369,-3.03503,-4.4791e-05,0,0,0.928319,-0.371784
  -8.91935,2.94528,0.00139425,0,0,0.473654,0.880711
  -2.68708,-2.23003,-0.000478224,0,0,-0.759842,0.650108
  ```
  It will create a topological map file like this:
  ```
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
1. *Editing the topological map*, lets say that you want to make sure that to reach a node you get there using a specific action like ramp_climb from another node then you will need to edit the topological map file. You can also edit the vertices of the influence area so it can be defined using diferent kinds of polygons.
  For example, lets suppose that in the previous map you want WayPoint3 to be reached only from the Waypoint1 using `ramp_climb` and that you want to define rectangular areas around your waypoints and a triangular area around the ChargingPoint, in such case your map file should look like this:

  ```
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
  `rosrun topological_utils insert_map.py topological_map.txt topological_map_name map_name`

## Launching Topological Localisation and Navigation

1. Once your map is inserted in the DB you can launch your topological navigation nodes using:
  `roslaunch topological_navigation topological_navigation.launch map:=topological_map_name`

~~*note:* scitos ramp HAS to be running (in future versions this won't be necessary), you can run it using:~~
~~`rosrun scitos_ramp_climb ramp_climb`~~


## Navigate

1. You can test the topological navigation using:
  `rosrun topological_navigation nav_client.py Destination`
  Where destination is the name of the node you want to reach.

