#!/usr/bin/env python

import sys
import rospy
import json


import rostopic
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import strands_navigation_msgs.srv

from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap

from topological_navigation.tmap_utils import *
#import topological_navigation.msg



class TopologicalNavLoc(object):
       
    
    def __init__(self, name) :
        self.throttle_val = rospy.get_param("~LocalisationThrottle", 3)
        self.only_latched = rospy.get_param("~OnlyLatched", True)
        self.throttle = self.throttle_val
        self.node="Unknown"
        self.wpstr="Unknown"
        self.cnstr="Unknown"

        self.wp_pub = rospy.Publisher('/closest_node', String, latch=True, queue_size=1)
        self.cn_pub = rospy.Publisher('/current_node', String, latch=True, queue_size=1)
        
        self.rec_map=False
        self.loc_by_topic=[]
        
        #This service returns a list of nodes that have a given tag
        self.get_tagged_srv=rospy.Service('/topological_localisation/get_nodes_with_tag', strands_navigation_msgs.srv.GetTaggedNodes, self.get_nodes_wtag_cb)
        
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")
        
        while not self.rec_map :
            rospy.sleep(rospy.Duration.from_sec(0.1))
        

        rospy.loginfo("Subscribing to localise topics")
        for j in self.nodes_by_topic:
            toptyp = rostopic.get_topic_class(j['topic'])
            rospy.loginfo("Subscribing to %s" %j['topic'])
            rospy.Subscriber(j['topic'], toptyp[0], self.Callback, callback_args=j)

        rospy.loginfo("Subscribing to robot pose")
        rospy.Subscriber("/robot_pose", Pose, self.PoseCallback)

        rospy.loginfo("NODES BY TOPIC: %s" %self.names_by_topic)
        rospy.loginfo("NO GO NODES: %s" %self.nogos)

        rospy.loginfo("All Done ...")
        rospy.spin()


    def PoseCallback(self, msg):
        if(self.throttle%self.throttle_val==0):
            #rospy.loginfo("NO GO NODES: %s" %self.nogos)
            self.distances=[]
            for i in self.tmap.nodes:
                d= get_distance_node_pose(i, msg)#get_distance_to_node(i, msg)
                a={}
                a['node'] = i
                a['dist'] = d
                self.distances.append(a)

            self.distances = sorted(self.distances, key=lambda k: k['dist'])
            #print self.distances
            closeststr='none'
            currentstr='none'
            
            not_loc=True
            if self.loc_by_topic:
                test_node=get_node(self.tmap, self.loc_by_topic[0])
                if self.point_in_poly(test_node, msg):
                    not_loc=False
                    closeststr=str(self.loc_by_topic[0])
                    currentstr=str(self.loc_by_topic[0])

            if not_loc:
                ind = 0
                while not_loc and ind<len(self.distances) and ind<3 :
                    if self.distances[ind]['node'].name not in self.names_by_topic:
                        if self.point_in_poly(self.distances[ind]['node'], msg) :
                            currentstr=str(self.distances[ind]['node'].name)
                            closeststr=currentstr
                            not_loc=False
                    ind+=1
                          
                ind = 0
                not_loc=True
                while not_loc and ind<len(self.distances) and closeststr=='none' :
                    if self.distances[ind]['node'].name not in self.nogos and self.distances[ind]['node'].name not in self.names_by_topic :
                        closeststr=str(self.distances[ind]['node'].name)
                        not_loc=False
                    ind+=1
                #currentstr=str(self.distances[0]['node'])

            self.publishTopics(closeststr, currentstr)
            self.throttle=1
        else:
            self.throttle +=1


    def publishTopics(self, wpstr, cnstr) :
        if self.only_latched :
            if self.wpstr != wpstr:
                self.wp_pub.publish(wpstr)
            if self.cnstr != cnstr:
                self.cn_pub.publish(cnstr)
        else:
            self.wp_pub.publish(wpstr)
            self.cn_pub.publish(cnstr)
        self.wpstr=wpstr
        self.cnstr=cnstr



    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.names_by_topic=[]
        self.nodes_by_topic=[]
        self.nogos=[]

        self.tmap = msg        
        self.rec_map=True
        
        for i in self.tmap.nodes:
            if i.localise_by_topic:
                a= json.loads(i.localise_by_topic)
                a['name'] = i.name
                self.nodes_by_topic.append(a)
                self.names_by_topic.append(a['name'])
                
        #print "NO GO NODES"
        self.nogos = self.get_no_go_nodes()
        #print self.nogos


    def Callback(self, msg, item):
        #print item
        val = getattr(msg, item['field'])
        if val == item['val'] :
            if item['name'] not in self.loc_by_topic:
                self.loc_by_topic.append(item['name'])
        else:
            if item['name'] in self.loc_by_topic:
                self.loc_by_topic.remove(item['name'])



    def get_nodes_wtag_cb(self,req):
        tlist = []
        rlist=[]

        rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
        try:
            cont = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = cont(req.tag)
            tagnodes = resp1.nodes
        except rospy.ServiceException, e:
            rospy.loggerr("Service call failed: %s"%e)
            rlist.append(tlist)
            return rlist
        
        ldis = [x['node'].name for x in self.distances]
        for i in ldis:
            if i in tagnodes:
                tlist.append(i)
        rlist.append(tlist)
        return rlist

    """
     Get No_Go_Nodes
     
     This function gets the list of No go nodes
    """
    def get_no_go_nodes(self):
        rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
        try:
            get_prediction = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = get_prediction('no_go')
            #print resp1
            return resp1.nodes
        except rospy.ServiceException, e:
            rospy.loggerr(Service call failed: %s"%e)
            return []
        


    def point_in_poly(self,node,pose):
        x=pose.position.x-node.pose.position.x
        y=pose.position.y-node.pose.position.y
        
        n = len(node.verts)
        inside = False
    
        p1x = node.verts[0].x
        p1y = node.verts[0].y
        for i in range(n+1):
            p2x = node.verts[i % n].x
            p2y = node.verts[i % n].y
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y
        return inside


if __name__ == '__main__':
    rospy.init_node('topological_localisation')
    server = TopologicalNavLoc(rospy.get_name())