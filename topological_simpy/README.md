# topological_simpy

This SimPy version models topological nodes as `simpy.Container` that can each hold a robot, ensuring robots cannot occupy the same spot and therefore allow to detect deadlocks. It's like every topological node now has a security guard (`simpy.Container`). The Container manages the entrance and exit of the node. The Container only allows 1 robot to occupy his node and other robots must join a queue if they want to occupy the node after the current robot leaving.

The robot has to acquire approve from the Container before using his node and the Container will release the node when the robot leaves. At the same time, the Container let the first robot in the queue to occupy the node if there is a queue.

The robot also tells the Container how long he will occupy the node when requesting permit from the Container. So other robots know how long they need to wait if they join the queue. If the waiting time is beyond their expectations, they could choose a new route that avoid the occupied node.

This demo  shows how the `simpy.Container`  is used for a picking and transporting tasks with multiple pickers and robots. The pickers harvest trays and calls a robot to load. The robot transports the trays to cold storage node. 

Features of this package:
- Independent of ROS
- Quick test on a Topo map for multi-robot path planning and  multi-picker task allocating
- Works with all topological maps

Assumptions:
- Robot requests a new node once he reaches the current node
- Robot release the previous node once he reaches halfway towards the next node
- Robot avoids occupied node when searching route
- Robot joins a queue and waits at base station when the cold storage node is being used
- Robot will consider to use new route if a planned node is occupied by other robots if the new route costs less than waiting, or choose to wait the node to be released

# Running step:

Go to topological_simpy/scripts/

`python simple_topo_sim.py`
