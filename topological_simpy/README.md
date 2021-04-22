# topological_simpy

This SimPy version models topological nodes as `simpy.Container` that can each hold a robot, ensuring robots cannot occupy the same spot and therefore allow to detect deadlocks. The pickers harvest trays and calls a robot to load. The robot transports the trays to cold storage node. 

Features of this package:
- Independent of ROS
- Quick test on a Topo map for multi-robot path planning and  multi-picker task allocating (next target)
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
