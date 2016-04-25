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

from threading import Thread


class LocaliseByTopicSubscriber(object):
    """
    Helper class for localise by topic subcription. Callable to start subsriber
    thread.
    """
    def __init__(self, topic, callback, callback_args):
        self.topic = topic
        self.callback = callback
        self.callback_args = callback_args
        self.sub = None
        self.t = None

    def get_topic_type(self, topic, blocking=False):
        """
        Get the topic type.
        !!! Overriden from rostopic !!!

        :param topic: topic name, ``str``
        :param blocking: (default False) block until topic becomes available, ``bool``

        :returns: topic type, real topic name and fn to evaluate the message instance
          if the topic points to a field within a topic, e.g. /rosout/msg. fn is None otherwise. ``(str, str, fn)``
        :raises: :exc:`ROSTopicException` If master cannot be contacted
        """
        topic_type, real_topic, msg_eval = rostopic._get_topic_type(topic)
        if topic_type:
            return topic_type, real_topic, msg_eval
        elif blocking:
            sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
            while not rospy.is_shutdown():
                topic_type, real_topic, msg_eval = rostopic._get_topic_type(topic)
                if topic_type:
                    return topic_type, real_topic, msg_eval
                else:
                    rostopic._sleep(10.) # Change! Waiting for 10 seconds instead of 0.1 to reduce load
        return None, None, None

    def __call__(self):
        """
        When called start a new thread that waits for the topic type and then
        subscribes. This is therefore non blocking and waits in the background.
        """
        self.t = Thread(target=self.subscribe)
        self.t.start()

    def subscribe(self):
        """
        Get the topic type and subscribe to topic. Subscriber is kept alive as
        long as the instance of the class is alive.
        """
        rostopic.get_topic_type = self.get_topic_type # Monkey patch
        topic_type = rostopic.get_topic_class(self.topic, True)[0]
        rospy.loginfo("Subscribing to %s" % self.topic)
        self.sub = rospy.Subscriber(
            name=self.topic,
            data_class=topic_type,
            callback=self.callback,
            callback_args=self.callback_args
        )


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

        self.force_check=True
        self.rec_map=False
        self.loc_by_topic=[]
        self.persist={}

        self.current_pose=Pose()
        self.previous_pose=Pose()
        self.previous_pose.position.x=1000 #just give a random big value so this is tested

        #This service returns a list of nodes that have a given tag
        self.get_tagged_srv=rospy.Service('/topological_localisation/get_nodes_with_tag', strands_navigation_msgs.srv.GetTaggedNodes, self.get_nodes_wtag_cb)
        self.get_tagged_srv=rospy.Service('/topological_localisation/localise_pose', strands_navigation_msgs.srv.LocalisePose, self.localise_pose_cb)

        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")

        while not self.rec_map :
            rospy.sleep(rospy.Duration.from_sec(0.1))


        rospy.loginfo("Subscribing to localise topics")
        subscribers = []
        for j in self.nodes_by_topic:
            # Append to list to keep the instance alive and the subscriber active.
            subscribers.append(LocaliseByTopicSubscriber(
                topic=j['topic'],
                callback=self.Callback,
                callback_args=j
            ))
            # Calling instance of class to start subsribing thread.
            subscribers[-1]()

        rospy.loginfo("Subscribing to robot pose")
        rospy.Subscriber("/robot_pose", Pose, self.PoseCallback)

        rospy.loginfo("NODES BY TOPIC: %s" %self.names_by_topic)
        rospy.loginfo("NO GO NODES: %s" %self.nogos)

        rospy.loginfo("All Done ...")
        rospy.spin()

    """
     get_distances_to_pose

     This function returns the distance from each waypoint to a pose in an organised way
    """
    def get_distances_to_pose(self, pose):
        distances=[]
        for i in self.tmap.nodes:
            d= get_distance_node_pose(i, pose)#get_distance_to_node(i, msg)
            a={}
            a['node'] = i
            a['dist'] = d
            distances.append(a)
    
        distances = sorted(distances, key=lambda k: k['dist'])
        return distances

    """
     PoseCallback

     This function receives the Robot Pose and localises 
     the robot in topological space
    """
    def PoseCallback(self, msg):
        self.current_pose = msg
        if(self.throttle%self.throttle_val==0):
            #rospy.loginfo("NO GO NODES: %s" %self.nogos)
            self.distances =[]
            self.distances = self.get_distances_to_pose(msg)
            closeststr='none'
            currentstr='none'

            not_loc=True
            if self.loc_by_topic:
                #print self.loc_by_topic
                for i in self.loc_by_topic:
                    if not_loc:
                        if not i['localise_anywhere']:      #If it should check the influence zone to localise by topic
                            test_node=get_node(self.tmap, i['name'])
                            if self.point_in_poly(test_node, msg):
                                not_loc=False
                                closeststr=str(i['name'])
                                currentstr=str(i['name'])
                        else:                               # If not, it is localised!!!
                            not_loc=False
                            closeststr=str(i['name'])
                            currentstr=str(i['name'])


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
                # No go nodes and Nodes localisable by topic are ONLY closest node when the robot is within them
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
        self.update_loc_by_topic()
        #print "NO GO NODES"
        self.nogos = self.get_no_go_nodes()
        #print self.nogos


    """
    update_loc_by_topic

     This function updates the localisation by topic variables
    """
    def update_loc_by_topic(self) :
        for i in self.tmap.nodes:
            if i.localise_by_topic:
                a= json.loads(i.localise_by_topic)
                a['name'] = i.name
                if not a.has_key('localise_anywhere'):
                    a['localise_anywhere']=True
                if not a.has_key('persistency'):
                    a['persistency']=10
                self.nodes_by_topic.append(a)
                self.names_by_topic.append(a['name'])
        print self.nodes_by_topic



    def Callback(self, msg, item):
        #needed for not checking the localise by topic when the robot hasn't moved and making sure it does when the new
        #position is close (<10) to the last one it was detected
        if self.force_check:
            dist = 1.0
        else:
            dist = get_distance(self.current_pose, self.previous_pose)

        if dist>0.10:
            #print self.persist
            val = getattr(msg, item['field'])
            if val == item['val'] :
                if self.persist.has_key(item['name']):
                    if self.persist[item['name']] < item['persistency']:
                        self.persist[item['name']]+=1
                else:
                    self.persist[item['name']]=0

                #if item['name'] not in self.loc_by_topic:
                if item['name'] not in [x['name'] for x in self.loc_by_topic] and self.persist[item['name']] < item['persistency']:
                    #if item['persistency']
                    self.loc_by_topic.append(item)
                    self.previous_pose = self.current_pose
                    self.force_check=False
                else:
                    self.force_check=True
            else:
                if item['name'] in self.persist:
                    self.persist.pop(item['name'])
                #if item['name'] in self.loc_by_topic:
                if item['name'] in [x['name'] for x in self.loc_by_topic]:
                    #self.loc_by_topic.remove(item['name'])
                    self.loc_by_topic.remove(item)
                    self.previous_pose = self.current_pose
                    self.force_check=True



    def get_nodes_wtag_cb(self,req):
        tlist = []
        rlist=[]

        try:
            rospy.wait_for_service('/topological_map_manager/get_tagged_nodes', timeout=3)
            cont = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = cont(req.tag)
            tagnodes = resp1.nodes
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

        ldis = [x['node'].name for x in self.distances]
        for i in ldis:
            if i in tagnodes:
                tlist.append(i)
        rlist.append(tlist)
        return rlist

    """
     localise_pose_cb

     This function gets the node and closest node for a pose
    """
    def localise_pose_cb(self, req):
        not_loc=True
        distances=[]
        distances=self.get_distances_to_pose(req.pose)
        closeststr='none'
        currentstr='none'


        ind = 0
        while not_loc and ind<len(distances) and ind<3 :
            if self.point_in_poly(distances[ind]['node'], req.pose) :
                currentstr=str(self.distances[ind]['node'].name)
                closeststr=currentstr
                not_loc=False
            ind+=1

        ind = 0
        while not_loc and ind<len(self.distances) :
            if self.distances[ind]['node'].name not in self.nogos :
                closeststr=str(self.distances[ind]['node'].name)
                not_loc=False
            ind+=1
            
        return currentstr, closeststr


        

    """
     Get No_Go_Nodes

     This function gets the list of No go nodes
    """
    def get_no_go_nodes(self):
        try:
            rospy.wait_for_service('/topological_map_manager/get_tagged_nodes', timeout=3)
            get_prediction = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = get_prediction('no_go')
            #print resp1
            return resp1.nodes
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)



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