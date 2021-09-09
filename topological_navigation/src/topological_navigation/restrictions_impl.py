import tf
import rospy
import dynamic_reconfigure.client
import numpy as np

from abc import ABCMeta, abstractmethod, abstractproperty
from geometry_msgs.msg import Pose, Quaternion
from topological_navigation.point2line import pnt2line
from std_msgs.msg import String



###############################
##
# Here extend the class AbstractRestriction to create a new restriction.
# Once its defined here it will be automatically imported by the RestrictionsManager.
##
class AbstractRestriction():
    """ Abstract definition of a restriction """
    __metaclass__ = ABCMeta

    robot_state = {}
    DEFAULT_EVALUATION = True

    @abstractproperty
    def name(self):
        """ The name must correspond to the restriction variable name in the map definition """
        raise NotImplementedError()

    @abstractmethod
    def evaluate_node(self, node=None, value=None, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        raise NotImplementedError()

    @abstractmethod
    def evaluate_edge(self, edge=None, value=None, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        raise NotImplementedError()

    @abstractmethod 
    def ground_to_robot(self):
        """ Retrieves and saves internally the state of the robot necessary for evaluating the restriction  """
        raise NotImplementedError()

## utils TODO these utils should go in tmap_utils I believe

def get_node_pose(node_name, tmap):
    pose = Pose()
    for node in tmap["nodes"]:
        if node["meta"]["node"] == node_name:
            # position[:] = [node["node"]["pose"].position.x, node["node"]["pose"].position.y]
            pose = node["node"]["pose"]
            break

    return pose

# return the two nodes positions at both ends of an edge
def get_edge_positions(edge_name, tmap):
    edge_name_parts = edge_name.split("_")
    node1, node2 = np.empty((2)), np.empty((2))
    node1[:] = np.nan
    node2[:] = np.nan

    for node in tmap["nodes"]:
        if node["meta"]["node"] == edge_name_parts[0]:
            node1[:] = [node["node"]["pose"].position.x, node["node"]["pose"].position.y]
        elif node["meta"]["node"] == edge_name_parts[1]:
            node2[:] = [node["node"]["pose"].position.x, node["node"]["pose"].position.y]
    
    return node1, node2

# position must be a np array with x and y coordinates
def distance_position_edge(position, edge_name, tmap):
    enode1, enode2 = get_edge_positions(edge_name, tmap)

    return np.abs(np.cross(enode2-enode1, enode1-position))/np.linalg.norm(enode2-enode1)

# positions must be a 2d np array with x and y coordinates with shape (n, 2)
def distance_positions_edge(positions, edge_name, tmap):
    enode1, enode2 = get_edge_positions(edge_name, tmap)

    return np.abs(np.cross(enode2-enode1, enode1-positions))/np.linalg.norm(enode2-enode1)

##

class RobotType(AbstractRestriction):
    name = "robot"

    def __init__(self):
        super(AbstractRestriction, self).__init__()

        # subscribe to task topic
        def _save_robot_type(msg):
            self.robot_state.update({
                "type": msg.data
            })

        rospy.Subscriber("type", 
            String, 
            lambda msg: _save_robot_type(msg)
        )

    def _evaluate(self, value, robot_state):
        evaluation = self.DEFAULT_EVALUATION

        # default state
        if "type" not in robot_state:
            robot_state = self.robot_state

        if "type" in robot_state:
            evaluation = value.lower() == robot_state["type"].lower()
        
        return evaluation
    
    def evaluate_node(self, node, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        return self._evaluate(value, robot_state)

    def evaluate_edge(self, edge, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        return self._evaluate(value, robot_state)

    def ground_to_robot(self):
        pass

class TaskName(AbstractRestriction):
    name = "task"

    def __init__(self):
        super(AbstractRestriction, self).__init__()

        # subscribe to task topic
        def _save_robot_task(msg):
            self.robot_state.update({
                "task": msg.data
            })

        rospy.Subscriber("task", 
            String, 
            lambda msg: _save_robot_task(msg)
        )

    def _evaluate(self, value, robot_state):
        evaluation = self.DEFAULT_EVALUATION

        # default state
        if "task" not in robot_state:
            robot_state = self.robot_state

        if "task" in robot_state:
            evaluation = value.lower() == robot_state["task"].lower()
        
        return evaluation

    def evaluate_node(self, node, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        return self._evaluate(value, robot_state)

    def evaluate_edge(self, edge, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        return self._evaluate(value, robot_state)

    def ground_to_robot(self):
        pass

class ObstacleFree(AbstractRestriction):
    name = "obstacleFree"

    def __init__(self, robots):
        super(AbstractRestriction, self).__init__()
        # keeps the position of each robot with timestamp
        self.robot_nodes = {}
        self.active = True
        if len(robots) == 0:
            rospy.logwarn("No robots provided for obstacleFree restriction.")
            self.active = False

        def _save_robot_nodes(msg, robot):
            self.robot_nodes[robot] = {
                "node": msg.data,
                "timestamp_secs": rospy.Time.now().to_sec()
            }
        for robot in robots:
            rospy.Subscriber("/{}/closest_node".format(robot), 
                String, 
                lambda msg, arg: _save_robot_nodes(msg, arg), 
                callback_args=(robot)
            )

    def _get_closest_node(self, robot, secs_thr=np.inf):
        node = []
        if robot in self.robot_nodes and \
                rospy.Time.now().to_sec() - self.robot_nodes[robot]["timestamp_secs"] < secs_thr:
            node = self.robot_nodes[robot]["node"]
        return node

    # value is the min distance [m] from the obstacles that the robot must have for the node to be obstacleFree
    def evaluate_node(self, node, value, robot_state={}, tmap=None):
        """ Returns the evaluation of the restriction associated with the given entity, must return a boolean value  """
        evaluation = self.DEFAULT_EVALUATION

        if tmap is not None and self.active:
            _node_pos = get_node_pose(node, tmap)
            node_pos = np.array([_node_pos["position"]["x"], _node_pos["position"]["y"]])  

            all_nodes = [self._get_closest_node(r) for r in self.robot_nodes if r != rospy.get_namespace().strip("/")]
            distances = []
            for node_a in all_nodes:
                _node_pos_a = get_node_pose(node_a, tmap)
                node_pos_a = np.array([_node_pos_a["position"]["x"], _node_pos_a["position"]["y"]])

                distances.append(
                    np.sqrt(np.sum(np.power(node_pos - node_pos_a, 2)))
                )
            if len(distances) > 0:
                evaluation = np.min(distances) > float(value)

        return evaluation
    
    # value is the min distance [m] from the obstacle that the robot must have for the edge to be obstacleFree
    def evaluate_edge(self, edge, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        evaluation = self.DEFAULT_EVALUATION
        
        # print(self.robot_nodes, rospy.get_namespace().strip("/"))
        if tmap is not None and self.active:
            distances = []
            
            _edge_pos_a = get_node_pose(edge.split("_")[0], tmap)
            edge_pos_a = np.array([[_edge_pos_a["position"]["x"], _edge_pos_a["position"]["y"], 0.]])
            _edge_pos_b = get_node_pose(edge.split("_")[1], tmap)
            edge_pos_b = np.array([[_edge_pos_b["position"]["x"], _edge_pos_b["position"]["y"], 0.]])

            all_nodes = [self._get_closest_node(r) for r in self.robot_nodes if r != rospy.get_namespace().strip("/")]

            for node_a in all_nodes:
                _node_pos = get_node_pose(node_a, tmap)
                node_pos = np.array([[_node_pos["position"]["x"], _node_pos["position"]["y"], 0.]])

                distance = pnt2line(node_pos, edge_pos_a, edge_pos_b)

                distances.append(distance)

            if len(distances) > 0:
                evaluation = np.min(distances) > float(value)

        return evaluation

    def ground_to_robot(self):
        pass
###############################

