#!/usr/bin/env python
###################################################################################################################
import sys, json
import rospy, rostopic, tf
import strands_navigation_msgs.srv

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from topological_navigation_msgs.msg import ClosestEdges

from topological_navigation.tmap_utils import *
from topological_navigation.point2line import pnt2line

from threading import Thread
from numpy import round


class LocaliseByTopicSubscriber(object):
    """
    Helper class for localise by topic subcription. Callable to start subsriber
    thread.
    """
    def __init__(self, topic, callback, callback_args):
        
        self.topic = rospy.get_namespace() + topic
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


    def close(self):
        self.sub.unregister()
        

    def __del__(self):
        self.close()
###################################################################################################################    


###################################################################################################################    
class TopologicalNavLoc(object):


    def __init__(self, name, wtags):
        
        self.throttle_val = rospy.get_param("~LocalisationThrottle", 3)
        self.only_latched = rospy.get_param("~OnlyLatched", True)
        self.throttle = self.throttle_val
        self.node="Unknown"
        self.wpstr="Unknown"
        self.cnstr="Unknown"
        self.closest_edge_ids = []
        self.closest_edge_dists = []
        
        # TODO: remove Temporary arg until tags functionality is MongoDB independent
        self.with_tags = wtags

        self.subscribers=[]
        self.wp_pub = rospy.Publisher('closest_node', String, latch=True, queue_size=1)
        self.cn_pub = rospy.Publisher('current_node', String, latch=True, queue_size=1)
        self.ce_pub = rospy.Publisher('closest_edges', ClosestEdges, latch=True, queue_size=1)

        self.force_check=True
        self.rec_map=False
        self.loc_by_topic=[]
        self.persist={}

        self.current_pose=Pose()
        self.previous_pose=Pose()
        self.previous_pose.position.x=1000 #just give a random big value so this is tested

        #This service returns a list of nodes that have a given tag
        self.get_tagged_srv=rospy.Service('topological_localisation/get_nodes_with_tag', strands_navigation_msgs.srv.GetTaggedNodes, self.get_nodes_wtag_cb)
        self.loc_pos_srv=rospy.Service('topological_localisation/localise_pose', strands_navigation_msgs.srv.LocalisePose, self.localise_pose_cb)

        rospy.Subscriber('/topological_map_2', String, self.MapCallback)
            
        rospy.loginfo("Waiting for Topological map ...")
        while not self.rec_map :
            rospy.sleep(rospy.Duration.from_sec(0.1))
        
        rospy.loginfo("NODES BY TOPIC: %s" %self.names_by_topic)
        rospy.loginfo("NO GO NODES: %s" %self.nogos)
        
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tmap_frame = self.tmap["transformation"]["child"]
        
        rospy.loginfo("Listening to the tf transform between {} and {}".format(self.tmap_frame, self.base_frame))
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
        self.PoseCallback()
        
        rospy.spin()

    
    def get_distances_to_pose(self, pose):
        """
        This function returns the distance from each waypoint to a pose in an organised way
        """
        distances = []
        for node in self.tmap["nodes"]:
            dist = get_distance_node_pose_from_tmap2(node, pose)
            a = {}
            a["node"] = node
            a["dist"] = dist
            distances.append(a)
        
        distances = sorted(distances, key=lambda k: k["dist"])
        return distances
    
    
    def get_edge_distances_to_pose(self, pose):
        """
        This function returns the distance from each edge to a pose in an organised way
        """
        pnt = (pose.position.x, pose.position.y, 0)
        distances = []
        
        for node in self.tmap["nodes"]:
            start = (node["node"]["pose"]["position"]["x"], node["node"]["pose"]["position"]["y"], 0)
            
            for edge in node["node"]["edges"]:
                dest = get_node_from_tmap2(self.tmap, edge["node"])
                end = (dest["pose"]["position"]["x"], dest["pose"]["position"]["y"], 0)
                dist,_ = pnt2line(pnt, start, end)
                
                a = {}
                a["edge_id"] = edge["edge_id"]
                a["dist"] = dist
                distances.append(a)
                 
        distances = sorted(distances, key=lambda k: k["dist"])
        return distances


    def PoseCallback(self):
        """
        This function receives the topo_map to base_link tf transform and localises 
        the robot in topological space
        """
        while not rospy.is_shutdown():
            
            try:
                now = rospy.Time(0)
                self.listener.waitForTransform(self.tmap_frame, self.base_frame, now, rospy.Duration(2.0))
                (trans,rot) = self.listener.lookupTransform(self.tmap_frame, self.base_frame, now)
            except Exception as e:
                rospy.logerr(e)
                self.rate.sleep()
                continue
        
            msg = Pose()
            msg.position.x = trans[0]
            msg.position.y = trans[1]
            msg.position.z = trans[2]
            msg.orientation.x = rot[0]
            msg.orientation.y = rot[1]
            msg.orientation.z = rot[2]
            msg.orientation.w = rot[3]
            
            self.current_pose = msg            
            if(self.throttle%self.throttle_val==0):
                self.distances =[]
                self.distances = self.get_distances_to_pose(msg)
                closeststr='none'
                currentstr='none'
                
                edge_distances = self.get_edge_distances_to_pose(msg)
                
                if len(edge_distances) > 1:
                    closest_edges = edge_distances[:2]
                else:
                    closest_edges = edge_distances * 2
                
                not_loc=True
                if self.loc_by_topic:
                    for i in self.loc_by_topic:
                        if not_loc:
                            if not i['localise_anywhere']:      #If it should check the influence zone to localise by topic
                                test_node = get_node_from_tmap2(self.tmap, i['name'])
                                if self.point_in_poly(test_node, msg):
                                    not_loc=False
                                    closeststr=str(i['name'])
                                    currentstr=str(i['name'])
                                    self.force_check = False
                            else:                               # If not, it is localised!!!
                                not_loc=False
                                closeststr=str(i['name'])
                                currentstr=str(i['name'])
                                self.force_check = False
                else:
                    self.force_check = True
    
                if not_loc:
                    ind = 0
                    while not_loc and ind<len(self.distances) and ind<3:
                        name = self.distances[ind]['node']['node']['name']
                        if name not in self.names_by_topic:
                            if self.point_in_poly(self.distances[ind]['node'], msg) :
                                currentstr=str(name)
                                closeststr=currentstr
                                not_loc=False
                        ind+=1
                              
                    ind = 0
                    not_loc=True
                    # No go nodes and Nodes localisable by topic are ONLY closest node when the robot is within them
                    while not_loc and ind<len(self.distances) and closeststr=='none' :
                        name = self.distances[ind]['node']['node']['name']
                        if name not in self.nogos and name not in self.names_by_topic :
                            closeststr=str(name)
                            not_loc=False
                        ind+=1
    
                self.publishTopics(closeststr, currentstr, closest_edges)
                self.throttle=1
            else:
                self.throttle +=1
                
            self.rate.sleep()


    def publishTopics(self, wpstr, cnstr, closest_edges) :
        
        def pub_closest_edges(closest_edge_ids, closest_edge_dists):
            msg = ClosestEdges()
            msg.edge_ids = closest_edge_ids
            msg.distances = closest_edge_dists
            self.ce_pub.publish(msg)
            
        closest_edge_ids = [edge["edge_id"] for edge in closest_edges]
        closest_edge_dists = list(round([edge["dist"] for edge in closest_edges], 3))
        if len(set(closest_edge_dists)) == 1:
            closest_edge_ids.sort()
        
        if self.only_latched :
            if self.wpstr != wpstr:
                self.wp_pub.publish(wpstr)
            if self.cnstr != cnstr:
                self.cn_pub.publish(cnstr)
            if self.closest_edge_ids != closest_edge_ids \
                or self.closest_edge_dists != closest_edge_dists:
                pub_closest_edges(closest_edge_ids, closest_edge_dists)
        else:
            self.wp_pub.publish(wpstr)
            self.cn_pub.publish(cnstr)
            pub_closest_edges(closest_edge_ids, closest_edge_dists)
            
        self.wpstr=wpstr
        self.cnstr=cnstr
        self.closest_edge_ids = closest_edge_ids
        self.closest_edge_dists = closest_edge_dists
        

    def MapCallback(self, msg):
        """
        This function receives the Topological Map
        """
        self.names_by_topic=[]
        self.nodes_by_topic=[]
        self.nogos=[]

        self.tmap = json.loads(msg.data) 
        
        self.rec_map=True
        self.update_loc_by_topic()
        # TODO: remove Temporary arg until tags functionality is MongoDB independent
        if self.with_tags:
            self.nogos = self.get_no_go_nodes()
        else:
            self.nogos=[]

        rospy.loginfo("Subscribing to localise topics")

        for i in self.subscribers:
            del i
        self.subscribers = []
        for j in self.nodes_by_topic:
            # Append to list to keep the instance alive and the subscriber active.
            self.subscribers.append(LocaliseByTopicSubscriber(
                topic=j['topic'],
                callback=self.Callback,
                callback_args=j
            ))
            # Calling instance of class to start subsribing thread.
            self.subscribers[-1]()


    def update_loc_by_topic(self):
        """
        This function updates the localisation by topic variables
        """
        for i in self.tmap['nodes']:
            if i['node']['localise_by_topic']:
                a= json.loads(i['node']['localise_by_topic'])
                a['name'] = i['node']['name']
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
            val = getattr(msg, item['field'])
            
            if val == item['val']:
                if self.persist.has_key(item['name']):
                    if self.persist[item['name']] < item['persistency']:
                        self.persist[item['name']]+=1
                else:
                    self.persist[item['name']]=0
                    
                if item['name'] not in [x['name'] for x in self.loc_by_topic] and self.persist[item['name']] < item['persistency']:
                    self.loc_by_topic.append(item)
                    self.previous_pose = self.current_pose
                    
            else:
                if item['name'] in self.persist:
                    self.persist.pop(item['name'])
                    
                if item['name'] in [x['name'] for x in self.loc_by_topic]:
                    self.loc_by_topic.remove(item)
                    self.previous_pose = self.current_pose
                    

    def get_nodes_wtag_cb(self,req):
        
        tlist = []
        rlist=[]

        try:
            rospy.wait_for_service('/topological_map_manager2/get_tagged_nodes', timeout=3)
            cont = rospy.ServiceProxy('/topological_map_manager2/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
                
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


    def localise_pose_cb(self, req):
        """
        This function gets the node and closest node for a pose
        """
        not_loc=True
        distances=[]
        distances=self.get_distances_to_pose(req.pose)
        closeststr='none'
        currentstr='none'

        ind = 0
        while not_loc and ind<len(distances) and ind<3 :
            if self.point_in_poly(distances[ind]['node'], req.pose) :
                name = distances[ind]['node']['node']['name']
                currentstr=str(name)
                closeststr=currentstr
                not_loc=False
            ind+=1

        ind = 0
        while not_loc and ind<len(distances) :
            name = distances[ind]['node']['node']['name']
            if name not in self.nogos :
                closeststr=str(name)
                not_loc=False
            ind+=1
            
        return currentstr, closeststr


    def get_no_go_nodes(self):
        """
        This function gets the list of No go nodes
        """
        try:
            rospy.wait_for_service('/topological_map_manager2/get_tagged_nodes', timeout=3)
            get_prediction = rospy.ServiceProxy('/topological_map_manager2/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
                
            resp1 = get_prediction('no_go')
            return resp1.nodes
        
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)


    def point_in_poly(self,node,pose):
        
        x=pose.position.x-node["node"]["pose"]["position"]["x"]
        y=pose.position.y-node["node"]["pose"]["position"]["y"]

        n = len(node["node"]["verts"])
        inside = False

        p1x = node["node"]["verts"][0]["x"]
        p1y = node["node"]["verts"][0]["y"]
        for i in range(n+1):
            p2x = node["node"]["verts"][i % n]["x"]
            p2y = node["node"]["verts"][i % n]["y"]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y
            
        return inside
###################################################################################################################        


###################################################################################################################
if __name__ == '__main__':
    rospy.init_node('topological_localisation')
    wtags=True
    argc = len(sys.argv)
    if argc > 1:
        if '-notags' in sys.argv:
            wtags = False
    server = TopologicalNavLoc(rospy.get_name(), wtags)
###################################################################################################################    