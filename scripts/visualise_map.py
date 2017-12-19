#!/usr/bin/env python

import sys
import rospy
import pymongo
import json
import sys
import math


from time import sleep

from threading import Timer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from mongodb_store.message_store import MessageStoreProxy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from strands_navigation_msgs.msg import TopologicalNode

from topological_navigation.topological_map import *


class TopologicalMapVis(object):
    _killall_timers=False


    def __init__(self, name, filename) :

        rospy.on_shutdown(self._on_node_shutdown)

        self.update_needed=False
        self.in_feedback=False
        
        self._point_set=filename

        self.map_pub = rospy.Publisher('topological_nodes_array', MarkerArray)
        self.map_zone_pub = rospy.Publisher('topological_node_zones_array', MarkerArray)
        self.map_edge_pub = rospy.Publisher('topological_edges_array', MarkerArray)

        self.menu_handler = MenuHandler()
        
        self._update_everything()
      
        t = Timer(1.0, self.timer_callback)
        t.start()       
        #self.run_analysis()
        rospy.loginfo("All Done ...")
        rospy.spin()



    def _create_marker_array(self):
             
        for i in self.topo_map.nodes :
            #print i
            marker = Marker()
            marker.header.frame_id = "/map"
            #marker.header.stamp = rospy.now()
            marker.type = marker.ARROW
            marker.scale.x = 0.3
            marker.scale.y = 0.15
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 0.7
            #marker.lifetime=2
            marker.pose = i._get_pose()
            marker.pose.position.z = marker.pose.position.z+0.1
            self.map_nodes.markers.append(marker)

        idn = 0
        for m in self.map_nodes.markers:
            m.id = idn
            idn += 1
    


    def _create_edges_array(self):
        for node in self.topo_map.nodes :
            for i in node.edges :
                marker = Marker()
                marker.header.frame_id = "/map"
                #marker.header.stamp = rospy.now()
                marker.type = marker.LINE_LIST
                #marker.lifetime=2
                ind = self.topo_map._get_node_index(i['node'])
                #print "%s -> %s" %(node.name, self.topo_map.nodes[ind].name)
                V1=Point()
                V2=Point()
                V1= (node._get_pose()).position
                V2= (self.topo_map.nodes[ind]._get_pose()).position
                marker.scale.x = 0.1
                marker.color.a = 0.8
                marker.color.r = 0.2
                marker.color.g = 0.8
                marker.color.b = 0.2
                marker.points.append(V1)
                marker.points.append(V2)
                self.map_edges.markers.append(marker)
                edge_name=node.name+"_"+self.topo_map.nodes[ind].name
                self._edge_marker(edge_name, V1, V2, edge_name)
        idn = 0
        for m in self.map_edges.markers:
            m.id = idn
            idn += 1            



    def _create_vertices_array(self) :

        for node in self.topo_map.nodes :
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.LINE_STRIP
            #marker.lifetime=2
            marker.scale.x = 0.1
            marker.color.a = 0.8
            marker.color.r = 0.7
            marker.color.g = 0.1
            marker.color.b = 0.2
            node._get_coords()
            count=0
            for i in node.vertices :
                #print i[0], i[1]
                Vert = Point()
                Vert.z = 0.05
                Vert.x = node.px + i[0]
                Vert.y = node.py + i[1]
                marker.points.append(Vert)
                vertname = "%s-%d" %(node.name, count)
                Pos = Pose()
                Pos.position = Vert
                self._vertex_marker(vertname, Pos, vertname)
                count+=1
            marker.points.append(marker.points[0])
            self.node_zone.markers.append(marker)
        
        idn = 0
        for m in self.node_zone.markers:
            m.id = idn
            idn += 1 



    def _create_interactive_markers(self):
        for i in self.topo_map.nodes :
            self._create_marker(i.name, i._get_pose(), i.name)



    def _initMenu(self):
        self.h_first_entry = self.menu_handler.insert( "First Entry" )
        entry = self.menu_handler.insert( "deep", parent=h_first_entry)
        entry = self.menu_handler.insert( "sub", parent=entry );
        entry = self.menu_handler.insert( "menu", parent=entry, callback=deepCb );
    
        self.menu_handler.setCheckState( self.menu_handler.insert( "Show First Entry", callback=enableCb ), MenuHandler.CHECKED )
    
        self.sub_menu_handle = self.menu_handler.insert( "Switch" )
        for i in range(5):
            s = "Mode " + str(i)
            self.h_mode_last = self.menu_handler.insert( s, parent=sub_menu_handle, callback=modeCb )
            self.menu_handler.setCheckState( self.h_mode_last, MenuHandler.UNCHECKED)
        # check the very last entry
        menu_handler.setCheckState( self.h_mode_last, MenuHandler.CHECKED )



    def _edge_marker(self, marker_name, point1, point2, marker_description="edge marker") :
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = "/map"
        marker.name = marker_name
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.CYLINDER
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "context_menu"
        control.interaction_mode = InteractiveMarkerControl.MENU
        marker.controls.append(control)

        self._edge_server.insert(marker, self._menu_feedback)
        self._edge_server.applyChanges()

        pose = Pose()
        
        pose.position.x = (point1.x+point2.x)/2
        pose.position.y = (point1.y+point2.y)/2
        pose.position.z = (point1.z+point2.z)/2

        if pose is not None:
            pose.position.z=pose.position.z+0.15
            self._edge_server.setPose( marker.name, pose )
            self._edge_server.applyChanges()


    def _create_marker(self, marker_name, pose, marker_description="waypoint marker") :
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = "/map"
        marker.name = marker_name
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )

        # move x
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        #move y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        #rotate z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        self._marker_server.insert(marker, self._marker_feedback)
        self._marker_server.applyChanges()

        if pose is not None:
            pose.position.z=pose.position.z+0.15
            self._marker_server.setPose( marker.name, pose )
            self._marker_server.applyChanges()



    def _vertex_marker(self, marker_name, pose, marker_description="vertex marker"):
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = "/map"
        marker.name = marker_name
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = 0.25
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.25
        box_marker.color.r = 0.5
        box_marker.color.g = 0.0
        box_marker.color.b = 0.5
        box_marker.color.a = 0.8

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        #box_control.always_visible = False
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )

        # move x
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.always_visible = False
#        control.name = "move_x"
#        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.name = "move_plane"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(control)


        self._vertex_server.insert(marker, self._vertex_feedback)
        self._vertex_server.applyChanges()

        if pose is not None:
            pose.position.z=pose.position.z+0.15
            self._vertex_server.setPose( marker.name, pose )
            self._vertex_server.applyChanges()



    def _marker_feedback(self, feedback):
        self.in_feedback=True
        self.topo_map.update_node_waypoint(feedback.marker_name, feedback.pose)
        self.update_needed=True


    def _menu_feedback(self, feedback):
        print feedback


    def _vertex_feedback(self, feedback):
        self.in_feedback=True
        vertex_name = feedback.marker_name.rsplit('-', 1)
        node_name = vertex_name[0]
        vertex_index = int(vertex_name[1])
        self.topo_map.update_node_vertex(node_name, vertex_index, feedback.pose)
        self.update_needed=True


    def timer_callback(self):
        
        if self.update_needed and not self.in_feedback :
            self._delete_everything()
            self._update_everything()
        else :
            if self.in_feedback :
                self.in_feedback=False
            self.map_pub.publish(self.map_nodes)
            self.map_edge_pub.publish(self.map_edges)
            self.map_zone_pub.publish(self.node_zone)
        
        if not self._killall_timers :
            t = Timer(1.0, self.timer_callback)
            t.start()



    def _update_everything(self) :

        print "loading file from map %s" %self._point_set
        self.topo_map = topological_map(self._point_set)
        print "Done"
        
        self._marker_server = InteractiveMarkerServer(self._point_set+"_markers")      
        self._vertex_server = InteractiveMarkerServer(self._point_set+"_zones")
        self._edge_server = InteractiveMarkerServer(self._point_set+"_edges")

        self.map_edges = MarkerArray()
        self.map_nodes = MarkerArray()
        self.node_zone = MarkerArray()

        self._create_interactive_markers()
        self._create_marker_array()
        self._create_edges_array()
        self._create_vertices_array()
        
        self.update_needed=False


        
    def _delete_everything(self) :
        
        del self._marker_server
        del self._vertex_server

        del self.map_edges
        del self.map_nodes
        del self.node_zone
        del self.topo_map        



    def _on_node_shutdown(self):
        self._killall_timers=True
        #sleep(2)


if __name__ == '__main__':
    mapname=str(sys.argv[1])
    rospy.init_node('topological_visualisation')
    server = TopologicalMapVis(rospy.get_name(),mapname)
