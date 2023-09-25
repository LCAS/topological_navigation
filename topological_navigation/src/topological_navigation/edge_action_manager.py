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

        if not is_last_edge_in_route:
            # Calculate orientation of desintation node to orientation node
            origin_pose = self.origin_node['node']['pose']['position']
            destin_pose = self.destination_node['node']['pose']['position']
            angle_radians = math.atan2(destin_pose['y']-origin_pose['y'], destin_pose['x']-origin_pose['x'])

            # Apply Orientation to move_base goal
            quart = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
            goal_args['target_pose']['pose']['orientation']['x'] = quart[0]
            goal_args['target_pose']['pose']['orientation']['y'] = quart[1]
            goal_args['target_pose']['pose']['orientation']['z'] = quart[2]
            goal_args['target_pose']['pose']['orientation']['w'] = quart[3]


        # Calculate orientation of goal and robot
        goal_z = goal_args['target_pose']['pose']['orientation']['z']
        goal_w = goal_args['target_pose']['pose']['orientation']['w']
        goal_euler = tf.transformations.euler_from_quaternion([0.0, 0.0, goal_z, goal_w])[2]

        robot_z = robot_odometry.pose.pose.orientation.z
        robot_w = robot_odometry.pose.pose.orientation.w
        robot_euler = tf.transformations.euler_from_quaternion([0.0, 0.0, robot_z, robot_w])[2]


        #if the robot is closer to the initial goal orientation than the 180 goal orientation
        robot_rot = robot_euler + math.pi

        initial_rot = goal_euler + math.pi
        initial_offset = abs(robot_rot - initial_rot) % (2*math.pi)

        flipped_rot = (goal_euler + math.pi + math.pi) % (2*math.pi)
        flipped_offset = abs(robot_rot - flipped_rot)

        print('robot_rot', robot_rot *(180/math.pi))
        print('initial_rot', initial_rot *(180/math.pi))
        print('flipped_rot', flipped_rot *(180/math.pi))

        print('initial_offset', initial_offset *(180/math.pi))
        print('flipped_offset', flipped_offset *(180/math.pi))

        # If initial offset is larger than flipped offset, use the flipped offset as this is more accurate
        if initial_offset > flipped_offset:
            quart = tf.transformations.quaternion_from_euler(0.0, 0.0, flipped_rot-math.pi)
            goal_args['target_pose']['pose']['orientation']['x'] = quart[0]
            goal_args['target_pose']['pose']['orientation']['y'] = quart[1]
            goal_args['target_pose']['pose']['orientation']['z'] = quart[2]
            goal_args['target_pose']['pose']['orientation']['w'] = quart[3]
            print('Flipping goal to allow ackerman navigation to be fluid')
            #FIXME: this should be handled by yaw tolerance
        else:
            print('Orientation offset better for current robot alignment so not flipping')

        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)

    def execute(self):
        
        rospy.loginfo("Edge Action Manager: Executing the action...")
        self.client.send_goal(self.goal)
        self.current_action = self.action_name
#########################################################################################################
