#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib, json, yaml
import tf, math
import operator, collections, copy

from functools import reduce  # forward compatibility for Python 3
from rospy_message_converter import message_converter
from actionlib_msgs.msg import GoalStatus


def _import(location, name):
    mod = __import__(location, fromlist=[name]) 
    return getattr(mod, name) 


class dict_tools(object):
                
                
    def get_paths_from_nested_dict(self, nested):
        paths = list(self.nested_dict_iter(nested))
        return [{"keys": item[0], "value": item[1]} for item in paths]
    
    
    def nested_dict_iter(self, nested, prefix=""):
        """
        Recursively loops through a nested dictionary. 
        For each inner-most value generates the list of keys needed to access it.
        """
        for key, value in nested.items():
            path = "{},{}".format(prefix, key)
            if isinstance(value, collections.Mapping):
                for inner_key, inner_value in self.nested_dict_iter(value, path):
                    yield inner_key, inner_value
            else:
                yield path[1:].split(","), value
    
    
    def getFromDict(self, dataDict, mapList):
        return reduce(operator.getitem, mapList, dataDict)


    def setInDict(self, dataDict, mapList, value):     
        self.getFromDict(dataDict, mapList[:-1])[mapList[-1]] = value
        return dataDict
#########################################################################################################


#########################################################################################################
class EdgeActionManager(object):
    
    
    def __init__(self):
        
        self.client = None        
        self.current_action = "none"
        self.dt = dict_tools()
        
    
    def initialise(self, edge, destination_node, origin_node=None, is_last_edge_in_route=None, robot_odometry=None):
        
        self.edge = yaml.safe_load(json.dumps(edge)) # no unicode in edge
        self.destination_node = destination_node
        self.origin_node = origin_node
        
        rospy.loginfo("Edge Action Manager: Processing edge {}".format(self.edge["edge_id"]))
        
        self.action_name = self.edge["action"]
        if self.action_name != self.current_action:
            self.preempt()

        action_type = self.edge["action_type"]        
        items = action_type.split("/")
        package = items[0]
        action_spec = items[1][:-4] + "Action"
        
        rospy.loginfo("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        action = _import(package+".msg", action_spec)
        
        rospy.loginfo("Edge Action Manager: Creating a {} client".format(self.action_name.upper()))
        self.client = actionlib.SimpleActionClient(self.action_name, action)        
        self.client.wait_for_server()
        
        rospy.loginfo("Edge Action Manager: Constructing the goal")
        self.construct_goal(action_type, copy.deepcopy(self.edge["goal"]), is_last_edge_in_route, robot_odometry)
        
        
    def preempt(self):
        
        if self.client is not None:
            status = self.client.get_state()
            if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
                self.client.cancel_all_goals()


    def construct_goal(self, action_type, goal_args, is_last_edge_in_route=None, robot_odometry=None):
        paths = self.dt.get_paths_from_nested_dict(goal_args)

        for item in paths:
            value = item["value"]

            if isinstance(value, str):

                if value.startswith("$"):
                    _property = self.dt.getFromDict(self.destination_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
                elif value.startswith("+") and self.origin_node is not None:
                    _property = self.dt.getFromDict(self.origin_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)

        # Orientation of origin node
        orig_z = self.origin_node['node']['pose']['orientation']['z']
        orig_w = self.origin_node['node']['pose']['orientation']['w']
        orig_euler = tf.transformations.euler_from_quaternion([0.0, 0.0, orig_z, orig_w])[2]

        # Flipped orientation of destination node
        flipped_orig_euler = (orig_euler + math.pi + math.pi) % (2*math.pi)

        # Orientation of destination node
        dest_z = self.destination_node['node']['pose']['orientation']['z']
        dest_w = self.destination_node['node']['pose']['orientation']['w']
        dest_euler = tf.transformations.euler_from_quaternion([0.0, 0.0, dest_z, dest_w])[2]

        # Flipped orientation of destination node
        flipped_dest_euler = (dest_euler + math.pi + math.pi) % (2*math.pi)

        # Orientation of edge
        orig_pose = self.origin_node['node']['pose']['position']
        dest_pose = self.destination_node['node']['pose']['position']
        edge_euler = math.atan2(dest_pose['y']-orig_pose['y'], dest_pose['x']-orig_pose['x'])

        # Flipped orientation of edge
        orig_pose = self.origin_node['node']['pose']['position']
        dest_pose = self.destination_node['node']['pose']['position']
        flipped_edge_euler = math.atan2(orig_pose['y']-dest_pose['y'], orig_pose['x']-dest_pose['x'])

        # Orientation of robot
        robot_z = robot_odometry.pose.pose.orientation.z
        robot_w = robot_odometry.pose.pose.orientation.w
        robot_euler = tf.transformations.euler_from_quaternion([0.0, 0.0, robot_z, robot_w])[2]

        # Find offsets from robot to the dest and flipped dest
        robot_euler_offset_from_orig_euler = abs(robot_euler - orig_euler) % (2*math.pi)
        robot_euler_offset_from_flipped_orig_euler = abs(robot_euler - flipped_orig_euler) % (2*math.pi)
        orig_euler_needs_flipping = robot_euler_offset_from_orig_euler > robot_euler_offset_from_flipped_orig_euler

        # Find offsets from robot to the dest and flipped dest
        robot_euler_offset_from_dest_euler = abs(robot_euler - dest_euler) % (2*math.pi)
        robot_euler_offset_from_flipped_dest_euler = abs(robot_euler - flipped_dest_euler) % (2*math.pi)
        dest_euler_needs_flipping = robot_euler_offset_from_dest_euler > robot_euler_offset_from_flipped_dest_euler

        # Find offsets from robot to the edge and flipped edge
        robot_euler_offset_from_edge_euler = abs(robot_euler - edge_euler) % (2*math.pi)
        robot_euler_offset_from_flipped_edge_euler = abs(robot_euler - flipped_edge_euler) % (2*math.pi)
        edge_euler_needs_flipping = robot_euler_offset_from_edge_euler > robot_euler_offset_from_flipped_edge_euler


        """
#useless piles of shite
robot euler
origin euler
flipped origin euler


destination euler
edge euler

flipped destination euler
flipped edge euler


#use destination (or flipped)

is intermediate node
    are we out of row
        #use edge instead (or flipped)


"""

        def set_destination_orientation(euler_angle, flipped_euler_angle, flip_conditional)
            euler = flipped_euler_angle if flip_conditional else euler_angle
            quart = tf.transformations.quaternion_from_euler(0.0, 0.0, euler)
            orientation['x'] = quart[0]
            orientation['y'] = quart[1]
            orientation['z'] = quart[2]
            orientation['w'] = quart[3]
            return orientation
        #goal_args['target_pose']['pose']['orientation'] = set_orientation(euler_angle)


        goal_args['target_pose']['pose']['orientation'] = set_orientation(dest_euler, flipped_dest_euler, dest_euler_needs_flipping)

        # If intermidiate node of route
        if not is_last_edge_in_route:

            # If node is in row
            if '-c' not in self.destination_node["node"]["name"]:
                goal_args['target_pose']['pose']['orientation'] = set_orientation(edge_euler, flipped_edge_euler, edge_euler_needs_flipping)


        """
        # If intermidate node, set orientation to direction of edge
        if not is_last_edge_in_route:
            # Calculate orientation of desintation node to orientation node
            origin_pose = self.origin_node['node']['pose']['position']
            destin_pose = self.destination_node['node']['pose']['position']
            angle_radians = math.atan2(destin_pose['y']-origin_pose['y'], destin_pose['x']-origin_pose['x'])

            # Apply Orientation to move_base goal
            edge_orientation = tf.transformations.quaternion_from_euler(0, 0, angle_radians)



        # If travelling to a row end
        if self.destination_node["node"]["name"].endswith('-ca'):

            if '-c' not in self.origin_node["node"]["name"]:
                # If robot is approaching row, force forwards facing
                # use original orientation
                print('ROW ROW ROW, original destination node orientation should be better')
                goal_args['target_pose']['pose']['orientation'] = self.destination_node['node']['pose']['orientation']
                pass
            else:
                # If robot is leaving row, force backwards facing
                # use flipped orientation
                print('ROW ROW ROW, flipped destination node orientation should be better')
                goal_args['target_pose']['pose']['orientation'] = self.destination_node['node']['pose']['orientation']
                dest_rot = (dest_euler + math.pi + math.pi) % (2*math.pi)
                quart = tf.transformations.quaternion_from_euler(0.0, 0.0, dest_rot-math.pi)
                goal_args['target_pose']['pose']['orientation']['x'] = quart[0]
                goal_args['target_pose']['pose']['orientation']['y'] = quart[1]
                goal_args['target_pose']['pose']['orientation']['z'] = quart[2]
                goal_args['target_pose']['pose']['orientation']['w'] = quart[3]
                pass

        else:
            # If initial offset is larger than flipped offset, use the flipped offset as this is more accurate
            if initial_offset > flipped_offset:
                print('ROW ROW ROW, flipping edge->robot orientation')
                flipped_edge_quart = tf.transformations.quaternion_from_euler(0.0, 0.0, flipped_edge_euler)
                goal_args['target_pose']['pose']['orientation']['x'] = flipped_edge_quart[0]
                goal_args['target_pose']['pose']['orientation']['y'] = flipped_edge_quart[1]
                goal_args['target_pose']['pose']['orientation']['z'] = flipped_edge_quart[2]
                goal_args['target_pose']['pose']['orientation']['w'] = flipped_edge_quart[3]
                print('Flipping goal to allow ackerman navigation to be fluid')
                #FIXME: this should be handled by yaw tolerance
            else:
                print('ROW ROW ROW, original edge->robot orientation')
                # Apply edge orientation to move_base goal
                edge_quart = tf.transformations.quaternion_from_euler(0, 0, edge_euler)
                goal_args['target_pose']['pose']['orientation']['x'] = edge_quart[0]
                goal_args['target_pose']['pose']['orientation']['y'] = edge_quart[1]
                goal_args['target_pose']['pose']['orientation']['z'] = edge_quart[2]
                goal_args['target_pose']['pose']['orientation']['w'] = edge_quart[3]
                print('Orientation offset better for current robot alignment so not flipping')
        """


        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)

    def execute(self):
        
        rospy.loginfo("Edge Action Manager: Executing the action...")
        self.client.send_goal(self.goal)
        self.current_action = self.action_name
#########################################################################################################
