#! /usr/bin/env python

"""
    This node is used to manually create a topomap live using the robot driven using
    an controller, buttons are pressed to add/remove nodes and to generate
    the topomap.
    Author:Sergi Molina <sergi.molina@sagarobotics.com> 2022
"""

from copy import deepcopy
import rospy
import rospkg
import datetime
from geometry_msgs.msg import Pose
from math import sqrt
import numpy as np
from os import path
from scipy.stats import linregress
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse
import yaml


class RobotTmapping(object):

    def __init__(self):

        # Params
        self.pointset = rospy.get_param("~tmap")
        self.tmap_file = rospy.get_param("~tmap_file")
        self.site_name = rospy.get_param("~site_name", 'riseholme_rtk')
        self.insert_map = rospy.get_param("~insert_map", False)
        self.dbscan_eps = rospy.get_param('~dbscan_eps', 4.0)
        self.min_num_rows = rospy.get_param('~min_num_rows', 2)
        self.closest_node_threshold = rospy.get_param('~closest_node_threshold', 0.5)
        self.button_lock_button_ind = rospy.get_param('~button_lock_button_ind', 6)  # Back
        self.add_node_button_ind = rospy.get_param('~add_node_button_ind', 2)  # X
        self.remove_node_button_ind = rospy.get_param('~remove_node_button_ind', 1)  # B
        self.generate_tmap_button_ind = rospy.get_param('~generate_tmap_button_ind', 3)  # Y
        self.use_inorder_clustering = rospy.get_param('~use_inorder_clustering', False)
        
        # Disable until toponav 2 supports mongodb storage of new map.
        self.insert_map = False

        # Variables
        self.nodes = []  # A list of node Pose msgs that will be used to generate the tmap
        self.previous_button = None
        self.node_id = 0

        template_dir = rospkg.RosPack().get_path("topological_navigation")
        self.template_node = self.load_yaml(template_dir + "/resources/template_node_2.yaml")
        self.template_edge = self.load_yaml(template_dir + "/resources/template_edge.yaml")


        # Services
        save_waypoints_srv = rospy.Service('/tmapping_robot/save_waypoints', Trigger, self.save_waypoints_srv_cb)
        save_map_srv = rospy.Service('/tmapping_robot/save_map', Trigger, self.generate_tmap_srv_cb)

        # Subscribers
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('/joy', Joy, self.joy_cb)

        # Publishers
        self.node_vis_pub = rospy.Publisher('/nodes', MarkerArray, queue_size=1)
        node_vis = Marker(action=Marker.DELETEALL)
        self.node_vis_pub.publish([node_vis])

        # Load existing nodes from tmap_file if there are any
        self.get_tmap_nodes()


    def load_yaml(self,filename):
        with open(filename,'r') as f:
            return yaml.load(f)
            

    def save_yaml(self,filename, data, dfs=True):
        with open(filename,'w') as f:
            return yaml.dump(data, f, default_flow_style=dfs)

    def get_tmap_nodes(self):
        print("Getting current nodes from tmap file")
        print(self.tmap_file)
        if path.exists(self.tmap_file):
            self.topomap = self.load_yaml(self.tmap_file)
            print("Existing tmap: ", self.topomap)
            if self.topomap is not None:
                for node in self.topomap["nodes"]:
                    pose = Pose()
                    pose.position.x =  node["node"]["pose"]["position"]["x"]
                    pose.position.y =  node["node"]["pose"]["position"]["y"]
                    pose.orientation.x = node["node"]["pose"]["orientation"]["x"]
                    pose.orientation.y = node["node"]["pose"]["orientation"]["y"]
                    pose.orientation.z = node["node"]["pose"]["orientation"]["z"]
                    pose.orientation.w = node["node"]["pose"]["orientation"]["w"]
                    self.nodes.append([self.node_id, pose])
                    self.node_id += 1
                    self.update_node_markers()
            else:
                self.init_map()
        else:
            self.init_map()
        print('There are', len(self.nodes), 'nodes in the existing map.')


    def robot_pose_cb(self, msg):
        self.robot_pose_msg = msg


    def joy_cb(self, msg):
        buttons = msg.buttons

        if buttons[self.button_lock_button_ind]:
            if buttons[self.add_node_button_ind]:
                if self.previous_button is None or self.add_node_button_ind != self.previous_button:
                    self.add_node()
                    self.previous_button = self.add_node_button_ind
            elif buttons[self.remove_node_button_ind]:
                if self.remove_node_button_ind != self.previous_button:
                    self.remove_node()
                    self.previous_button = self.remove_node_button_ind
            elif buttons[self.generate_tmap_button_ind]:
                if self.generate_tmap_button_ind != self.previous_button:
                    self.generate_tmap()
                    self.previous_button = self.generate_tmap_button_ind
            else:
                self.previous_button = None


    def add_node(self):
        print("Adding node")
        pose = self.robot_pose_msg
        dist, nearest_id, _ = self.get_nearest_node(pose)
        if not dist or dist > self.closest_node_threshold:
            self.nodes.append([self.node_id, pose])
            print("New node: ", pose)
            print("Num nodes: ", len(self.nodes))
            self.update_node_markers()
            self.node_id +=1
        else:
            print("Too close to an existing node, won't add one!")


    def remove_node(self):
        print("Removing nearest node")
        pose = self.robot_pose_msg
        dist, nearest_id, ind = self.get_nearest_node(pose)
        if not dist:
            print("List is empty!")
        else:
            if dist > 5:
                print("Not near any nodes so not removing any!")
            else:
                self.remove_marker(nearest_id)
                self.nodes.pop(ind)
                print("Num nodes: ", len(self.nodes))


    def update_node_markers(self):

        diameter = 0.7

        node_vis_array = []

        for node in (self.nodes):
            node_vis = Marker()
            node_vis.type = node_vis.SPHERE
            node_vis.header.frame_id = 'map'
            node_vis.id = node[0]
            node_vis.pose.position.x = node[1].position.x
            node_vis.pose.position.y = node[1].position.y
            node_vis.pose.position.z = node[1].position.z + diameter/2
            node_vis.pose.orientation = node[1].orientation
            node_vis.scale.x = diameter
            node_vis.scale.y = diameter
            node_vis.scale.z = diameter
            node_vis.color.a = 1
            node_vis.color.r = 1
            node_vis.color.g = 0.6
            node_vis.color.b = 0
            node_vis.lifetime = rospy.Duration()

            node_vis_array.append(node_vis)

        self.node_vis_pub.publish(node_vis_array)


    def remove_marker(self, id):
        node_vis = Marker(id=id, action=Marker.DELETE)
        self.node_vis_pub.publish([node_vis])


    def get_nearest_node(self, pose):
        dist = None
        id = None
        ind = None
        for i, node in enumerate(self.nodes):
            d = sqrt((pose.position.x - node[1].position.x)**2
                 + (pose.position.y - node[1].position.y)**2)
            if not dist or d < dist:
                dist = d
                id = node[0]
                ind = i
        print(dist, id, ind)
        return dist, id, ind


    def rotate_influence_zone(self, node, quat):

        _, _, theta =  euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        R = np.array([[np.cos(theta),-np.sin(theta)],
                      [np.sin(theta),np.cos(theta)]])

        verts = deepcopy(node["node"]["verts"])
        new_verts = []
        for vert in verts:

            new_vert = np.array([vert["x"],vert["y"]])
            new_vert = np.dot(R, new_vert)

            new_verts.append({"x": float(new_vert[0]), "y": float(new_vert[1])})

        node["node"]["verts"] = new_verts

        return node
    

    def init_map(self):
        self.topomap = {}
        self.topomap["name"] = self.pointset
        self.topomap["metric_map"] = self.site_name
        self.topomap["pointset"] = self.pointset
        
        self.topomap["transformation"] = {}
        self.topomap["transformation"]["rotation"] = {}
        self.topomap["transformation"]["rotation"]["w"] = 1.0
        self.topomap["transformation"]["rotation"]["x"] = 0.0
        self.topomap["transformation"]["rotation"]["y"] = 0.0
        self.topomap["transformation"]["rotation"]["z"] = 0.0
        self.topomap["transformation"]["translation"] = {}
        self.topomap["transformation"]["translation"]["x"] = 0.0
        self.topomap["transformation"]["translation"]["y"] = 0.0
        self.topomap["transformation"]["translation"]["z"] = 0.0
        self.topomap["transformation"]["child"] = "topo_map"
        self.topomap["transformation"]["parent"] = "map"
        
        self.topomap["meta"] = {}
        self.topomap["meta"]["last_updated"] = self.get_time()
        self.topomap["nodes"] = []
        
        
    def get_time(self):
        return datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


    def set_meta(self):
        for node in self.topomap["nodes"]:
            node["meta"]["map"] = self.site_name
            node["meta"]["pointset"] = self.pointset


    def sort_nodes(self, class_member_mask, points):
        print(class_member_mask)
        cluster = points[class_member_mask]
        line_fit = linregress(cluster[:,1], cluster[:,2])
        print(line_fit)
        if abs(line_fit.slope) < 1:
            print("Sorting nodes in x direction")
            sorted_nodes = cluster[np.argsort(cluster[:, 1])]
        else:
            print("Sorting nodes in y direction")
            sorted_nodes = cluster[np.argsort(cluster[:, 2])]
        print("Sorted nodes: ", sorted_nodes)
        return sorted_nodes

    def generate_tmap(self):

        if not self.nodes:
            rospy.logwarn("No nodes yet, nothing to save!")
            return

        print("Generating tmap from node list")
        self.init_map()
        # if self.use_inorder_clustering:
        #     edges_found = self.find_edges_inorder()
        # else:
        #     edges_found = self.find_edges()

        # if edges_found:
        #     print("Creating topomap with edges")
        #     for i, node_data in enumerate(self.nodes_sorted):
        #         # print("creating node " + node_data[0])
        #         node = deepcopy(self.template_node)

        #         node["meta"]["node"] = node_data[0]
        #         node["node"]["name"] = node_data[0]

        #         ind = [x for x, e in enumerate(self.nodes) if e[0] == node_data[1]][0]

        #         node["node"]["pose"]["position"]["x"] = float(self.nodes[ind][1].position.x)
        #         node["node"]["pose"]["position"]["y"] = float(self.nodes[ind][1].position.y)

        #         node["node"]["pose"]["orientation"]["x"] = float(self.nodes[ind][1].orientation.x)
        #         node["node"]["pose"]["orientation"]["y"] = float(self.nodes[ind][1].orientation.y)
        #         node["node"]["pose"]["orientation"]["z"] = float(self.nodes[ind][1].orientation.z)
        #         node["node"]["pose"]["orientation"]["w"] = float(self.nodes[ind][1].orientation.w)
                
        #         node["node"]["properties"]["xy_goal_tolerance"] = 0.5
        #         node["node"]["properties"]["yaw_goal_tolerance"] = 6.3

        #         node = self.rotate_influence_zone(node, self.nodes[ind][1].orientation)

        #         self.topomap["nodes"].append(node)

        #     # Add edges
        #     for edge in self.edges:
        #         origin = edge[0]
        #         destination = edge[1]
        #         for node in self.topomap["nodes"]:
        #             if node["node"]["name"] == origin:
        #                 edge_id_str = origin + "_" + destination
        #                 # print "adding edge " + edge_id_str + ' ' + edge[2]
                        
        #                 new_edge = deepcopy(self.template_edge)
        #                 new_edge["action"] = edge[2]
        #                 new_edge["edge_id"] = edge_id_str
        #                 new_edge["node"] = destination

        #                 node["node"]["edges"].append(new_edge)
        # else:
        print("Just adding nodes as I could not find the edges automatically")
        self.save_waypoints()

        self.set_meta()

        print("saving yaml")
        self.save_yaml(self.tmap_file, self.topomap, False)

        print("saved yaml")

        # if edges_found:
        #     if self.insert_map:
        #         tmap_path =  path.dirname(path.realpath(self.tmap_file))
        #         status = call("rosrun topological_utils load_yaml_map.py -f tmap.tmap",cwd=tmap_path,shell=True)
        #         print("TMAP inserted to mongodb, pointset is: " + self.pointset)
        #         print("TMAP file is: " + self.tmap_file)
        # else:
        #     print("Haven't created edges so won't insert incomplete topo map")


    def generate_tmap_srv_cb(self, req):
        self.generate_tmap()
        return TriggerResponse(success=True, message="Done!")


    def save_waypoints_srv_cb(self, req):
        self.save_waypoints()
        self.save_yaml(self.tmap_file, self.topomap, False)
        return TriggerResponse(success=True, message="Done!")


    def save_waypoints(self):
        print("Saving tmap waypoints only")
        self.init_map()
        for i, node_pose in enumerate(self.nodes):

            # print("creating node " + str(i))

            node = deepcopy(self.template_node)

            node["meta"]["node"] = 'n' + str(i)
            node["node"]["name"] = 'n' + str(i)

            node["node"]["pose"]["position"]["x"] = float(node_pose[1].position.x)
            node["node"]["pose"]["position"]["y"] = float(node_pose[1].position.y)

            node["node"]["pose"]["orientation"]["x"] = float(node_pose[1].orientation.x)
            node["node"]["pose"]["orientation"]["y"] = float(node_pose[1].orientation.y)
            node["node"]["pose"]["orientation"]["z"] = float(node_pose[1].orientation.z)
            node["node"]["pose"]["orientation"]["w"] = float(node_pose[1].orientation.w)
            
            node["node"]["properties"]["xy_goal_tolerance"] = 0.3
            node["node"]["properties"]["yaw_goal_tolerance"] = 0.3

            node = self.rotate_influence_zone(node, node_pose[1].orientation)

            self.topomap["nodes"].append(node)



if __name__ == '__main__':
    rospy.init_node('vogui_tmapping_node')
    RobotTmapping()
    rospy.spin()