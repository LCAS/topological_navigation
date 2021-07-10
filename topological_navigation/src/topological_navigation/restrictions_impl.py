import tf
import rospy
import numpy as np
from abc import ABCMeta, abstractmethod, abstractproperty
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import String


###############################
##
# Here extend the class AbstractRestriction to create a new restriction.
# Once its defined here it will be automatically imported by the RestrictionsManager.
##
class AbstractRestriction():
    """ Abstract definition of a restriction that can be evaluated in real-time """
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

    def _evaluate(self, value, robot_state):
        evaluation = self.DEFAULT_EVALUATION

        if len(robot_state) == 0 or "robot" not in robot_state:
            self.ground_to_robot()
            robot_state = self.robot_state

        if "robot" in robot_state:
            evaluation = value.lower() == robot_state["robot"].lower()
        
        return evaluation
    
    def evaluate_node(self, node, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        return self._evaluate(value, robot_state)

    def evaluate_edge(self, edge, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        return self._evaluate(value, robot_state)

    def ground_to_robot(self):
        # if robot_namespace is None:
        #     # robot namespace under which this script is running
        #     robot_namespace = rospy.get_namespace()
        try:
            topic_name = "type"
            robot_type = rospy.wait_for_message(topic_name, String, timeout=2)
        except rospy.ROSException:
            rospy.logwarn("Grounding of restrcition {} failed, topic {} not published".format(self.name, topic_name))
        else:
            self.robot_state.update({
                "robot": robot_type.data
            })

class TaskName(AbstractRestriction):
    name = "task"

    def __init__(self):
        super(AbstractRestriction, self).__init__()

    def _evaluate(self, value, robot_state):
        evaluation = self.DEFAULT_EVALUATION

        # default state
        if len(robot_state) == 0 or "task" not in robot_state:
            self.ground_to_robot()
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
        # if robot_namespace is None:
        #     # robot namespace under which this script is running
        #     robot_namespace = rospy.get_namespace()
        try:
            topic_name = "task"
            robot_type = rospy.wait_for_message(topic_name, String, timeout=2)
        except rospy.ROSException:
            rospy.logwarn("Grounding of restrcition {} failed, topic {} not published".format(
                self.name, topic_name))
        else:
            self.robot_state.update({
                "task": robot_type.data
            })

class ExactPose(AbstractRestriction):
    name = "exactPose"
    MAX_DIST = 1e-1 # the maximum distance, in angle/meter, allowed to still be exactPose

    def __init__(self, robots):
        super(AbstractRestriction, self).__init__()
        # keeps the position of each robot with timestamp
        self.robot_positions = {}
        def _save_robot_pose(msg, robot):
            self.robot_positions[robot] = {
                "position": msg.position,
                "timestamp_secs": rospy.Time.now().to_sec()
            }
        for robot in robots:
            rospy.Subscriber("/{}/robot_pose".format(robot), 
                Pose, 
                lambda msg, arg: _save_robot_pose(msg, arg), 
                callback_args=(robot)
            )

    def _get_current_position(self, robot, secs_thr=10):
        position = None
        if robot in self.robot_positions and \
                rospy.Time.now().to_sec() - self.robot_positions[robot]["timestamp_secs"] < secs_thr:
            position = self.robot_positions[robot]["position"]
        return position

    # value is not used, check if the current robot_pose is very close to the pose of the node
    def evaluate_node(self, node, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        evaluation = self.DEFAULT_EVALUATION

        # default state
        if len(robot_state) == 0 or "robot_pose" not in robot_state:
            self.ground_to_robot()
            robot_state = self.robot_state

        if tmap is not None:
            _node_pos = get_node_pose(node, tmap)
            node_pos = np.array([_node_pos["position"]["x"], _node_pos["position"]["y"]])
            pose_distance = np.sqrt(np.sum(
                (node_pos - np.array([robot_state["robot_pose"].position.x, robot_state["robot_pose"].position.y])) ** 2
            ))
            node_angle_inv = (_node_pos["orientation"]["x"], _node_pos["orientation"]["y"], _node_pos["orientation"]["z"], - _node_pos["orientation"]["w"])
            robot_angle = (robot_state["robot_pose"].orientation.x, robot_state["robot_pose"].orientation.y, robot_state["robot_pose"].orientation.z, robot_state["robot_pose"].orientation.w)
            angle_distance = tf.transformations.euler_from_quaternion(
                tf.transformations.quaternion_multiply(robot_angle, node_angle_inv)
            )[2] # yaw (z axis)
            if pose_distance <= self.MAX_DIST and angle_distance <= self.MAX_DIST:
                evaluation = True
            else:
                evaluation = False

        return evaluation
    
    # value not used, check if the current robot_pose is very close to the pose of the node at the beginning of edge
    def evaluate_edge(self, edge, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """

        start_node = edge.split("_")[0]

        return self.evaluate_node(start_node, value, robot_state, tmap)

    def ground_to_robot(self):
        try:
            topic_name = "robot_pose"
            robot_pose = rospy.wait_for_message(topic_name, Pose, timeout=2)
        except rospy.ROSException:
            rospy.logwarn("Grounding of restrcition {} failed, topic {} not published".format(
                self.name, topic_name))
        else:
            self.robot_state.update({
                "robot_pose": robot_pose
            })

class ObstacleFree(AbstractRestriction):
    name = "obstacleFree"

    def __init__(self, robots):
        super(AbstractRestriction, self).__init__()
        # keeps the position of each robot with timestamp
        self.robot_positions = {}
        def _save_robot_pose(msg, robot):
            self.robot_positions[robot] = {
                "position": msg.position,
                "timestamp_secs": rospy.Time.now().to_sec()
            }
        for robot in robots:
            rospy.Subscriber("/{}/robot_pose".format(robot), 
                Pose, 
                lambda msg, arg: _save_robot_pose(msg, arg), 
                callback_args=(robot)
            )

    def _get_current_position(self, robot, secs_thr=10):
        position = None
        if robot in self.robot_positions and \
                rospy.Time.now().to_sec() - self.robot_positions[robot]["timestamp_secs"] < secs_thr:
            position = self.robot_positions[robot]["position"]
        return position

    # value is the min distance [m] from the obstacles that the robot must have for the node to be obstacleFree
    def evaluate_node(self, node, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        evaluation = self.DEFAULT_EVALUATION

        if tmap is not None:
            _node_pos = get_node_pose(node, tmap)
            node_pos = np.array([_node_pos["position"]["x"], _node_pos["position"]["y"]])  

            all_positions = [self._get_current_position(r) for r in self.robot_positions if r != rospy.get_namespace()]
            all_positions = np.array([[p.x, p.y] for p in all_positions if p is not None])

            min_distance = np.min(np.sqrt(np.sum(
                (node_pos - all_positions) ** 2
            )))
            evaluation = min_distance > float(value)

        return evaluation
    
    # value is the min distance [m] from the obstacle that the robot must have for the edge to be obstacleFree
    def evaluate_edge(self, edge, value, robot_state={}, tmap=None):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        evaluation = self.DEFAULT_EVALUATION
        
        if tmap is not None:
            all_positions = [self._get_current_position(r) for r in self.robot_positions if r != rospy.get_namespace()]
            all_positions = np.array([[p.x, p.y] for p in all_positions if p is not None])

            min_distance = np.min(distance_positions_edge(all_positions, edge, tmap))
            evaluation = min_distance > float(value)

        return evaluation

    def ground_to_robot(self):
        pass
###############################

