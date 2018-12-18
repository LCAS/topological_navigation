#!/usr/bin/env python

import rospy
import math
import tf




from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from strands_navigation_msgs.msg import TopologicalMap
from topological_navigation.marker_arrays import *
from strands_navigation_msgs.msg import NavRoute


class PoliciesVis(object):
#    _killall_timers=False

    def __init__(self) :
        self._killall=False
        #rospy.on_shutdown(self._on_node_shutdown)
        self.route_nodes = NavRoute()
        self.map_edges = MarkerArray()
        #self.update_needed=False
        
        rospy.loginfo("Creating Publishers ...")
        self.policies_pub = rospy.Publisher('topological_edges_policies', MarkerArray)
        rospy.loginfo("Done ...")
        
        
        rospy.loginfo("Creating subscriber ...")      
        self.subs = rospy.Subscriber("mdp_plan_exec/current_policy_mode", NavRoute, self.policies_callback)       
        rospy.loginfo("Done ...")

        #Waiting for Topological Map        
        self.map_received=False
        rospy.Subscriber('topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while not self.map_received and not self._killall :
            rospy.sleep(rospy.Duration.from_sec(0.05))
        rospy.loginfo(" ...done")
        

        rospy.loginfo("All Done ...")


    def _update_everything(self) :
        print "updating ..."
        print self.route_nodes

        self.map_edges.markers=[]
        
        counter=0
        total = len(self.route_nodes.source)

        print 'updating '+str(total)+' edges'        
        while counter < total :
            print 'Creating edge '+str(counter)
            source = self.route_nodes.source[counter]
            ori = self.get_node(self.lnodes, source)
            targ = self.find_action(ori.name, self.route_nodes.edge_id[counter])
            if targ:
                #print ori.name,ori.pose.position      
                target = self.get_node(self.lnodes, targ)
                self.added_sources.append(source)
                #print target.name,target.pose.position
                if targ in self.added_sources:
                    self.create_edge(ori.pose.position, target.pose.position, "blue")
                else:
                    self.create_edge(ori.pose.position, target.pose.position, "red")
            counter+=1
        
        
        idn = 0
        for m in self.map_edges.markers:
            m.id = idn
            idn += 1
        self.policies_pub.publish(self.map_edges)
        print "All Done"


    def create_edge(self, point1, point2, color):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.ARROW
        
        
        pose = Pose()
        
        pose.position.x = point1.x
        pose.position.y = point1.y
        pose.position.z = point1.z
        angle = math.atan2((point2.y-point1.y),(point2.x-point1.x))
        
        qat = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.w = qat[3]
        pose.orientation.x = qat[0]
        pose.orientation.y = qat[1]
        pose.orientation.z = qat[2]                
        
        r = math.hypot((point2.y-point1.y),(point2.x-point1.x))#/3.0
        marker.scale.x = r
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 0.95
        marker.color.r = 0.9
        marker.color.g = 0.1
        marker.color.b = 0.1
        if color == "red":
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.1
        if color == "blue":
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 0.9
        marker.pose = pose
        self.map_edges.markers.append(marker)



    """
        get_node
        
        Given a topological map and a node name it returns the node object
    """
    def get_node(self, top_map, node_name):
        print 'looking for: '+node_name
        for i in top_map.nodes:
            #print i.name
            if i.name == node_name:
                return i
        return None


    """
     Find Action
         
    """
#    def find_action(self, source, target):
    def find_action(self, source, edge_id):
        #print 'Searching for action between: %s -> %s' %(source, target)
        found = False
        #action = 'none'
        target = 'none'
        for i in self.lnodes.nodes :
            if i.name == source :
                for j in i.edges:
                    if j.edge_id == edge_id:
                        #action = j.action
                        target = j.node
                found = True
        if not found:
            rospy.logwarn("source node not found")
            return None
        return target
       


    def policies_callback(self, msg) :
        print "GOT policies"
        self.added_sources = []
        self.route_nodes = msg
        self._update_everything()


    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        print "got map"
        self.lnodes = msg
#        print self.lnodes.nodes
        self.map_received = True 
        

    def on_node_shutdown(self):
        self._killall=True
        #sleep(2)