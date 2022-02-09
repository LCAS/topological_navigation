#!/usr/bin/env python

import rospy
import json
import sys
import math
import actionlib

from time import sleep

from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import String

from visualization_msgs.msg import *

from interactive_markers.interactive_marker_server import *

from topological_navigation_msgs.msg import GotoNodeGoal, GotoNodeAction, TopologicalRoute

def get_node(nodes_list, name):
    for i in nodes_list:
        if i['node']['name'] == name:
            return i['node']


class TopoMap2Vis(object):
    _pallete=[[0,0,0],[1,0,0],[0,0,1],[0,1,0],[1,1,0],[1,0,1],[0,1,1],[1,1,1]]
    def __init__(self, name, no_goto=False) :
        """
        """
        self.no_goto = no_goto
        self.killall=False
        self.topological_map = None
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]
        self.in_feedback=False

        rospy.on_shutdown(self._on_node_shutdown)

        self.topmap_pub = rospy.Publisher('topological_map_visualisation', MarkerArray, queue_size = 1, latch=True)
        self.routevis_pub = rospy.Publisher('topological_route_visualisation', MarkerArray, queue_size = 1)
        self.topo_map_sub = rospy.Subscriber("topological_map_2", String, self.topo_map_cb)
        self.topo_route_sub = rospy.Subscriber("/topological_navigation/Route", TopologicalRoute, self.route_cb)

        rospy.loginfo("Waiting for topo_map")

        if not no_goto:
            self._goto_server = InteractiveMarkerServer("go_to_node")
            self.client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
            self.client.wait_for_server()

        rospy.loginfo("All Done ...")
        rospy.spin()

    def topo_map_cb(self, msg):
        """
        """
        self.topological_map = json.loads(msg.data)
        rospy.loginfo("Received new topo_map")
        print(self.topological_map['name'])
        self.create_map_marker()


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
        rospy.sleep(0.1)


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
        marker.lifetime = rospy.Duration(120)
        marker.ns='/route'
        return marker


    def create_map_marker(self):
        """
        """
        del self.map_markers
        self.map_markers = MarkerArray()
        self.map_markers.markers=[]
        self.actions=[]
        idn=0
        for i in self.topological_map['nodes']:
            self.map_markers.markers.append(self.get_node_marker(i['node'], idn))
            idn+=1
            self.map_markers.markers.append(self.get_name_marker(i['node'], idn))
            idn+=1
            self.map_markers.markers.append(self.get_zone_marker(i['node'], idn))
            idn+=1
            for j in i['node']['edges']:
                self.update_actions(j['action'])
                marker = self.get_edge_marker(i['node'], j, idn)
                if marker:
                    self.map_markers.markers.append(marker)
                    idn += 1
        
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
        marker.scale = 1
        marker.name = node['name']
        marker.description = ''#node['name']

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append(self.makeArrow( marker.scale ))
        marker.controls.append(control)
    
        self._goto_server.insert(marker, self.goto_feedback_cb)
        self._goto_server.applyChanges()

        pos = self.node2pose(node['pose'])

        if pos is not None:
            pos.position.z=pos.position.z+0.15
            self._goto_server.setPose( marker.name, pos )
            self._goto_server.applyChanges()


    def makeArrow(self, scale):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = scale * 0.5
        marker.scale.y = scale * 0.25
        marker.scale.z = scale * 0.15
        marker.lifetime.secs = 3
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        return marker


    def goto_feedback_cb(self, feedback):
        if not self.in_feedback:
            self.in_feedback=True
            print('GOTO: '+str(feedback.marker_name))
            self.client.cancel_all_goals()
            navgoal = GotoNodeGoal()
            navgoal.target = feedback.marker_name
            #navgoal.origin = orig
            # Sends the goal to the action server.
            self.client.send_goal(navgoal)
            rospy.Timer(rospy.Duration(1.0), self.timer_cb, oneshot=True)    # This is needed so the callback is only triggered once

    
    def timer_cb(self, event) :
        self.in_feedback = False


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
        #print col
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
            rospy.logwarn("No node %s found" %edge.node)
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
        rospy.loginfo("See you later")
        #sleep(2)


if __name__ == '__main__':
    nogoto_mode=False
    argc = len(sys.argv)
    print(argc)
    if argc > 1:
        if '-no_goto' in sys.argv or '-n' in sys.argv :
            nogoto_mode = True

    rospy.init_node('topomap2_visualisation')
    server = TopoMap2Vis(rospy.get_name(), no_goto=nogoto_mode)
