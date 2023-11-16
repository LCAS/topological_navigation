#!/usr/bin/env python

import rclpy 
import json
import sys
import math
from rclpy.executors import MultiThreadedExecutor
from time import sleep
from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from topological_navigation_msgs.action import GotoNode, ExecutePolicyMode
from topological_navigation_msgs.action import ExecutePolicyMode
from rclpy.action import ActionClient 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from topological_navigation_msgs.msg import TopologicalRoute
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup 
from builtin_interfaces.msg import Duration
# from topological_navigation.route_search2 import RouteChecker, TopologicalRouteSearch2, get_route_distance
def get_node(nodes_list, name):
    for i in nodes_list:
        if i['node']['name'] == name:
            return i['node']

class TopoMap2Vis(rclpy.node.Node):
    _pallete=[[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,0.0,1.0],[0.0,1.0,0.0],[1.0,1.0,0.0],[1.0,0.0,1.0],[0.0,1.0,1.0],[1.0,1.0,1.0]]
    def __init__(self, name) :
        super().__init__(name)
        self.declare_parameter('~no_goto', rclpy.Parameter.Type.BOOL) 
        self.no_go = self.get_parameter_or("~no_goto", rclpy.Parameter('bool', rclpy.Parameter.Type.BOOL, False)).value
    
        self.no_goto = self.no_go
        self.get_logger().info("===============no_goto========== {} ".format(self.no_goto))
        self.killall=False
        self.topological_map = None
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]
        self.in_feedback=False
        self.action_status = 0 
        self.status_mapping = {}
        self.status_mapping[0] = "STATUS_UNKNOWN"
        self.status_mapping[1] = "STATUS_ACCEPTED"
        self.status_mapping[2] = "STATUS_EXECUTING"
        self.status_mapping[3] = "STATUS_CANCELING"
        self.status_mapping[4] = "STATUS_SUCCEEDED"
        self.status_mapping[5] = "STATUS_CANCELED"
        self.status_mapping[6] = "STATUS_ABORTED"
        self.goal_cancle_error_codes = {} 
        self.goal_cancle_error_codes[0] = "ERROR_NONE"
        self.goal_cancle_error_codes[1] = "ERROR_REJECTED"
        self.goal_cancle_error_codes[2] = "ERROR_UNKNOWN_GOAL_ID"
        self.goal_cancle_error_codes[3] = "ERROR_GOAL_TERMINATED"

        rclpy.get_default_context().on_shutdown(self._on_node_shutdown)

        self.latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.get_logger().info("Waiting for topo_map")
        self.goal_handle =  None 

        self.callback_group_map = ReentrantCallbackGroup()
        self.callback_goto_client = ReentrantCallbackGroup()

        self.topmap_pub = self.create_publisher(MarkerArray, 'topological_map_visualisation', qos_profile=self.latching_qos)
        self.routevis_pub = self.create_publisher(MarkerArray, 'topological_route_visualisation', qos_profile=self.latching_qos)
        self.topo_map_sub = self.create_subscription(String, "/topological_map_2", self.topo_map_cb, qos_profile=self.latching_qos, callback_group=self.callback_group_map)
        self.topo_route_sub = self.create_subscription(TopologicalRoute, "topological_navigation/Route", self.route_cb, qos_profile=self.latching_qos)

        if not self.no_goto:
            self.get_logger().info("==============o========== {} ".format(1))
            self.action_server_name = '/topological_navigation'
            self._goto_server = InteractiveMarkerServer(self, "go_to_node")
            self.client = ActionClient(self, GotoNode, self.action_server_name, callback_group=self.callback_goto_client)
            while rclpy.ok():
                try:
                    rclpy.spin_once(self)
                    if not self.client.server_is_ready():
                        self.get_logger().info("Visulize Manager: Waiting for the action server  {}...".format(self.action_server_name))
                        self.client.wait_for_server(timeout_sec=1)
                    if self.client.server_is_ready():
                        self.get_logger().info("Visulize Manager: the action server  {} is avaible ".format(self.action_server_name))
                        break
                except Exception as e:
                    self.get_logger().error("  {} ".format(e))
                    pass 
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                if self.topological_map is not None:
                    self.get_logger().info("Start generating map...")
                    self.create_map_marker()
                    self.get_logger().info("End map map...")
                    break 
            except Exception as e:
                self.get_logger().error("  {} ".format(e))
                pass   
        
       

        self.get_logger().info("All Done ...")

    def topo_map_cb(self, msg):
        self.topological_map = json.loads(msg.data)
        self.get_logger().info("Received new topo_map")
        self.get_logger().info(self.topological_map['name'])
        # self.lnodes = self.topological_map
        # self.topol_map = self.lnodes["pointset"]
        # self.rsearch = TopologicalRouteSearch2(self.lnodes)
        # self.route_checker = RouteChecker(self.lnodes)
        # self.get_logger().info("==============o========== {} ".format(2))
        # self.create_map_marker()


    def route_cb(self, msg):
        self.clear_route() # clear the last route
        self.route_marker = MarkerArray()
        self.route_marker.markers=[]
        idn=0
        for i in range(1,len(msg.nodes)):
            self.route_marker.markers.append(self.get_route_marker(msg.nodes[i-1], msg.nodes[i], idn))
            idn+=1
        self.routevis_pub.publish(self.route_marker)
        
        
    def clear_route(self):
        self.route_marker = MarkerArray()
        self.route_marker.markers=[]
        marker = Marker()
        marker.action = marker.DELETEALL
        self.route_marker.markers.append(marker)
        self.routevis_pub.publish(self.route_marker) 

    def get_route_marker(self, origin, end, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = 'topo_map'
        marker.type = marker.ARROW
        V1=Point()
        V2=Point()
        origin_node = get_node(self.topological_map['nodes'], origin)
        end_node = get_node(self.topological_map['nodes'], end)
        V1=self.node2pose(origin_node['pose']).position
        V1.z += 0.25
        V2=self.node2pose(end_node['pose']).position
        V2.z += 0.25
        marker.pose.orientation.w= 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4
        marker.color.a = 0.5
        marker.color.r = 0.33
        marker.color.g = 0.99
        marker.color.b = 0.55
        marker.points.append(V1)
        marker.points.append(V2)
        marker.lifetime.sec = 2 
        marker.ns='/route'
        return marker


    def create_map_marker(self):
        # self.get_logger().info("==============o========== {} ".format(3))
        del self.map_markers
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]
        self.actions=[]
        idn=0
        for i in self.topological_map['nodes']:
            # self.get_logger().info("==============o========== {} ".format(i))
            self.map_markers.markers.append(self.get_node_marker(i['node'], idn))
            idn+=1
            self.map_markers.markers.append(self.get_name_marker(i['node'], idn))
            idn+=1
            self.map_markers.markers.append(self.get_zone_marker(i['node'], idn))
            idn+=1
            # self.get_logger().info("==============o111========== {} ".format(4))
            for j in i['node']['edges']:
                # self.get_logger().info("==============o========== {} ".format(6))
                self.update_actions(j['action'])
                # self.get_logger().info("==============o========== {} ".format(7))
                marker = self.get_edge_marker(i['node'], j, idn)
                if marker:
                    self.map_markers.markers.append(marker)
                    idn += 1
                # self.get_logger().info("==============o========== {} ".format(8))
        
        if not self.no_goto:
            for i in self.topological_map['nodes']:
                self.create_goto_marker(i['node'])

        legend =0
        for k in self.actions:
            marker = self.get_legend_marker(k, legend, idn)
            self.map_markers.markers.append(marker)
            idn += 1
            legend+=1
        self.topmap_pub.publish(self.map_markers)


    def create_goto_marker(self, node):
        """
        """
        marker = InteractiveMarker()
        marker.header.frame_id = node['parent_frame']
        marker.scale = 1.0
        marker.name = node['name']
        marker.description = ''#node['name']

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append(self.makeArrow( marker.scale ))
        marker.controls.append(control)
    
        self._goto_server.insert(marker, feedback_callback=self.goto_feedback_cb)
        self._goto_server.applyChanges()

        pos = self.node2pose(node['pose'])

        if pos is not None:
            pos.position.z=pos.position.z+0.5
            self._goto_server.setPose( marker.name, pos )
            self._goto_server.applyChanges()


    def makeArrow(self, scale):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = scale * 0.5
        marker.scale.y = scale * 0.25
        marker.scale.z = scale * 0.15
        duration_msg = Duration()
        duration_msg.sec = 3 
        duration_msg.nanosec = 0
        marker.lifetime = duration_msg
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        return marker

    def preempt_action(self):
        if self.client is not None:
            if not self.client.server_is_ready():
                self.get_logger().info("Visulize Manager: Waiting for the action server  {}...".format(self.action_server_name))
                self.client.wait_for_server(timeout_sec=2)
            
            if not self.client.server_is_ready():
                self.get_logger().info("Visulize Manager: action server  {} not responding ... can not perform any action".format(self.action_server_name))
                return True
            
            if self.goal_handle is None:
                self.get_logger().info("Visulize Manager: there is no goal to stop it is already cancelled with status {}".format(self.action_status))
                return True 
             
            cancel_future = self.client._cancel_goal_async(self.goal_handle)
            self.get_logger().info("Waiting till terminating the current preemption")
            while rclpy.ok():
                try: 
                    rclpy.spin_once(self)
                    if cancel_future.done() and self.goal_get_result_future.done():
                        self.action_status = 5
                        self.get_logger().info("The goal cancel error code {} ".format(self.get_goal_cancle_error_msg(cancel_future.result().return_code)))
                        return True 
                except Exception as e:
                    pass 

    def get_goal_cancle_error_msg(self, status_code):
        try:
            return self.goal_cancle_error_codes[status_code]
        except Exception as e:
            self.get_logger().error("Goal cancle code {}".format(status_code))
            return self.goal_cancle_error_codes[0]
        
    def goto_feedback_cb(self, feedback):
        self.get_logger().info("====================goto_feedback_cb=================================")
        if not self.in_feedback:
            self.in_feedback=True
            self.get_logger().info('GOTO: '+str(feedback.marker_name))
            self.preempt_action()

            navgoal = GotoNode.Goal()
            navgoal.target = feedback.marker_name
            self.goal = navgoal
            self.execute()
            return 
            #navgoal.origin = orig
            # Sends the goal to the action server.
            # self.client.send_goal(navgoal)
            # rospy.Timer(rospy.Duration(1.0), self.timer_cb, oneshot=True)    # This is needed so the callback is only triggered once


    def feedback_callback(self, feedback_msg):
        self.nav_client_feedback = feedback_msg.feedback
        self.get_logger().info("Distance to goal: {} ".format(self.nav_client_feedback))
        return 
    
    def execute(self):     
        if not self.client.server_is_ready():
            self.get_logger().info("Visulize Manager: Waiting for the action server  {}...".format(self.action_server_name))
            self.client.wait_for_server(timeout_sec=2)
        
        if not self.client.server_is_ready():
            self.get_logger().info("Visulize Manager: action server  {} not responding ... can not perform any action".format(self.action_server_name))
            return 
        
        self.get_logger().info("Visulize Manager: Executing the action...")
        send_goal_future = self.client.send_goal_async(self.goal,  feedback_callback=self.feedback_callback)
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                if send_goal_future.done():
                    self.goal_handle = send_goal_future.result()
                    break
                self.get_logger().info("=========================== tryiiiii")
            except Exception as e:
                self.get_logger().error(" Error while executing go to node policy {} ".format(e))
                pass 

        if not self.goal_handle.accepted:
            self.get_logger().error('The goal rejected')
            return False

        self.get_logger().info('The goal accepted')
        self.goal_get_result_future = self.goal_handle.get_result_async()
        self.get_logger().info("Waiting for {} action to complete".format(self.action_server_name))
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                if self.goal_get_result_future.done():
                    status = self.goal_get_result_future.result().status
                    self.action_status = status
                    self.get_logger().info("Visulize Manager: Executing the action response with status {}".format(self.get_status_msg(self.action_status)))
                    self.current_action = self.action_name
                    self.goal_resposne = self.goal_get_result_future.result() 
                    return True 
            except Exception as e:
                pass 

    # def timer_cb(self, event) :
    #     self.in_feedback = False


    def get_colour(self, number):
        """
        """
        return self._pallete[number]

    def update_actions(self, action):
        """
        """
        if action not in self.actions:
            self.actions.append(action)


    def get_legend_marker(self, action, row, idn):
        """
        """
        col=self.get_colour(self.actions.index(action))
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text=action
        marker.pose.position.x= 1.0
        marker.pose.position.y= 0.0 + (0.18*row)
        marker.pose.position.z= 0.2
        marker.pose.orientation.w= 1.0
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = col[0]
        marker.color.g = col[1]
        marker.color.b = col[2]
        marker.ns='/legend'
        return marker


    def get_edge_marker(self, node, edge, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = node['parent_frame']
        marker.type = marker.LINE_LIST
        V1=Point()
        V2=Point()
        V1=self.node2pose(node['pose']).position
        V1.z += 0.1
        to_node=get_node(self.topological_map['nodes'], edge['node'])
        col=self.get_colour(self.actions.index(edge['action']))
        #self.get_logger().info col
        if to_node:
            V2= self.node2pose(to_node['pose']).position
            V2.z += 0.1
            marker.pose.orientation.w= 1.0
            marker.scale.x = 0.1
            marker.color.a = 0.5
            marker.color.r = col[0]
            marker.color.g = col[1]
            marker.color.b = col[2]
            marker.points.append(V1)
            marker.points.append(V2)
            marker.ns='/edges'
            return marker
        else:
            self.get_logger().warning("No node %s found" %edge.node)
            return None


    def get_node_marker(self, node, idn):
        """
        """
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = node['parent_frame']
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.6
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.7
        marker.pose = self.node2pose(node['pose'])
        marker.pose.position.z += 0.1
        marker.ns='/nodes'
        return marker


    def get_name_marker(self, node, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = node['parent_frame']
        marker.type = marker.TEXT_VIEW_FACING
        marker.text= node['name']
        marker.pose = self.node2pose(node['pose'])
        marker.pose.position.z += 0.2
        #marker.pose.position.y += 0.2
        marker.scale.z = 0.20
        marker.color.a = 1.00
        marker.color.r = 1.00
        marker.color.g = 1.00
        marker.color.b = 1.00
        marker.ns='/names'
        return marker


    def node2pose(self, pose):
        node_pose=Pose()
        node_pose.position.x = pose['position']['x']
        node_pose.position.y = pose['position']['y']
        node_pose.position.z = pose['position']['z']
        node_pose.orientation.w = pose['orientation']['w']
        node_pose.orientation.x = pose['orientation']['x']
        node_pose.orientation.y = pose['orientation']['y']
        node_pose.orientation.z = pose['orientation']['z']
        return node_pose


    def get_zone_marker(self, node, idn):
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.pose.orientation.w= 1.0
        marker.scale.x = 0.1
        marker.color.a = 0.8
        marker.color.r = 0.7
        marker.color.g = 0.1
        marker.color.b = 0.2

        for j in node['verts'] :
            Vert = Point()
            Vert.z = 0.05
            Vert.x = node['pose']['position']['x'] + j['x']
            Vert.y = node['pose']['position']['y'] + j['y']
            marker.points.append(Vert)

        Vert = Point()
        Vert.z = 0.05
        Vert.x = node['pose']['position']['x'] + node['verts'][0]['x']
        Vert.y = node['pose']['position']['y'] + node['verts'][0]['y']
        marker.points.append(Vert)
        marker.ns='/zones'
        return marker

    def _on_node_shutdown(self):
        """
        """
        self.clear_route()
        self.killall=True
        self.get_logger().info("See you later")

def main():
    rclpy.init(args=None)
    server = TopoMap2Vis('topomap2_visualisation')
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info('shutting down topomap2_visualisation node\n')
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()


