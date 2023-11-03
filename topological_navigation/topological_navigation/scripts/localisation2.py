#!/usr/bin/env python
###################################################################################################################
import sys, json, numpy as np
import rclpy, tf2_ros
import topological_navigation_msgs.srv
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32
from topological_navigation_msgs.msg import ClosestEdges
from topological_navigation_msgs.srv import GetTaggedNodes, LocalisePose
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from topological_navigation.tmap_utils import *
from topological_navigation.point2line import pnt2line
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time 
from threading import Thread, Event 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup 
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor 


class LocaliseByTopicSubscriber(object):
    """
    Helper class for localise by topic subcription. Callable to start subsriber
    thread.
    """
    def __init__(self, topic, callback, callback_args):
        
        self.topic = rclpy.get_namespace() + topic
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
            while not rclpy.is_shutdown():
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
        self.get_logger().info("Subscribing to %s" % self.topic)
        self.sub = rclpy.Subscriber(
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
class TopologicalNavLoc(rclpy.node.Node):

    def __init__(self, name, wtags):
        super().__init__(name)
        
        self.declare_parameter('~LocalisationThrottle', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('~OnlyLatched', rclpy.Parameter.Type.BOOL) 
        self.declare_parameter('~base_frame', rclpy.Parameter.Type.STRING)

        self.throttle_val = self.get_parameter_or("~LocalisationThrottle", Parameter('int', Parameter.Type.INTEGER, 3)).value
        self.only_latched = self.get_parameter_or("OnlyLatched", Parameter('bool', Parameter.Type.BOOL, True)).value 
        self.base_frame = self.get_parameter_or("~base_frame", Parameter('str', Parameter.Type.STRING, "base_link")).value

        self.throttle = self.throttle_val
        self.node="Unknown"
        self.wpstr="Unknown"
        self.closest_dist = 10e5-1
        self.cnstr="Unknown"
        self.closest_edge_ids = []
        self.closest_edge_dists = []
        self.node_poses = {}
        
        # TODO: remove Temporary arg until tags functionality is MongoDB independent
        self.with_tags = wtags

        self.subscribers=[]

        self.qos = QoSProfile(depth=1, 
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.wp_pub = self.create_publisher(String, 'closest_node', self.qos)
        self.wd_pub = self.create_publisher(Float32,'closest_node_distance', self.qos)
        self.cn_pub = self.create_publisher(String, 'current_node', self.qos)
        self.ce_pub = self.create_publisher(ClosestEdges, 'closest_edges', self.qos)

        self.force_check = True
        self.rec_map = False
        self.set_nogos = False  
        self.loc_by_topic = []
        self.persist = {}

        self.current_pose=Pose()
        self.previous_pose=Pose()
        self.previous_pose.position.x = 1000.0 #just give a random big value so this is tested

        self.service_get_tagged_done_event  = Event()
        self.callback_group = ReentrantCallbackGroup()
        self.callback_localize_pose = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.get_tagged_srv = self.create_service(GetTaggedNodes, '/topological_localisation/get_nodes_with_tag'
                                ,  self.get_nodes_wtag_cb, callback_group=self.callback_group)
        self.loc_pos_srv = self.create_service(LocalisePose, '/topological_localisation/localise_pose'
                                               , self.localise_pose_cb, callback_group=self.callback_localize_pose)

        self.subs_topmap = self.create_subscription(String, '/topological_map_2', self.MapCallback, 1)
        self.subs_topmap  # prevent unused variable warning

        self.get_logger().info("Localisation waiting for the Topological Map...")
        
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.rate = self.create_rate(20.0)
        
        self.create_timer(1.0, self.pose_callback, callback_group=self.timer_cb_group)

        
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
        try:
            pnts = np.array(self.vectors_start.shape[0] * [[pose.position.x, pose.position.y, 0]])
            distances = pnt2line(pnts, self.vectors_start, self.vectors_end)
            closest_edges = [self.dist_edge_ids[index] for index in np.argsort(distances)]
        except Exception as e:
            self.get_logger().warning("Cannot get distance to edges: {}".format(e))
            closest_edges = []
            distances = np.array([])
        
        return closest_edges, np.sort(distances)
        

    def pose_callback(self):
        """
        This function receives the topo_map to base_link tf transform and localises 
        the robot in topological space
        """
        if(self.rec_map is False):
            self.get_logger().warning("Wating for the topological map")
            return 
        elif self.set_nogos is False:
            if self.with_tags:
                self.nogos = self.get_no_go_nodes()
                self.set_nogos = True 
            else:
                self.nogos=[]
            self.get_logger().info("NO GO NODES: %s" %self.nogos)

        try:
            trans = self.tf_buffer.lookup_transform(self.tmap_frame, self.base_frame, rclpy.time.Time())
            msg = Pose()
            msg.position.x = trans.transform.translation.x 
            msg.position.y = trans.transform.translation.y 
            msg.position.z = trans.transform.translation.z 
            msg.orientation.x = trans.transform.rotation.x
            msg.orientation.y = trans.transform.rotation.y 
            msg.orientation.z = trans.transform.rotation.z 
            msg.orientation.w = trans.transform.rotation.w 
            
            self.current_pose = msg   
            if(self.throttle%self.throttle_val==0):
                self.distances = []
                self.distances = self.get_distances_to_pose(msg)
                closeststr='none'
                currentstr='none'
                
                closest_edges, edge_dists = self.get_edge_distances_to_pose(msg)
                if len(closest_edges) > 1:
                    closest_edges = closest_edges[:2]
                    edge_dists = edge_dists[:2]
                
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
                
                # distance to physically closest node.
                closest_dist = np.round(self.distances[0]["dist"], 3)
                self.publishTopics(closeststr, closest_dist, currentstr, closest_edges, list(np.round(edge_dists, 3)))
                self.throttle=1
            else:
                self.throttle +=1
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {self.tmap_frame} to {self.base_frame}: {ex}')
            self.rate.sleep()
            pass  
        

    def get_string_msgs(self, str):
        msg =  String()
        msg.data = str
        return msg 

    def get_float32_msgs(self, num):
        msg =  Float32()
        msg.data = num
        return msg 

    def publishTopics(self, wpstr, closest_dist, cnstr, closest_edge_ids, closest_edge_dists) :
        
        def pub_closest_edges(closest_edge_ids, closest_edge_dists):
            msg = ClosestEdges()
            msg.edge_ids = closest_edge_ids
            msg.distances = closest_edge_dists
            self.ce_pub.publish(msg)
            
        if len(set(closest_edge_dists)) == 1:
            closest_edge_ids.sort()
        
        if self.only_latched :
            if self.wpstr != wpstr:
                self.wp_pub.publish(self.get_string_msgs(wpstr))
            if self.closest_dist != closest_dist:
                self.wd_pub.publish(self.get_float32_msgs(closest_dist))
            if self.cnstr != cnstr:
                self.cn_pub.publish(self.get_string_msgs(cnstr))
            if self.closest_edge_ids != closest_edge_ids \
                or self.closest_edge_dists != closest_edge_dists:
                pub_closest_edges(closest_edge_ids, closest_edge_dists)
        else:
            self.wp_pub.publish(self.get_string_msgs(wpstr))
            self.wd_pub.publish(self.get_float32_msgs(closest_dist))
            self.cn_pub.publish(self.get_string_msgs(cnstr))
            pub_closest_edges(closest_edge_ids, closest_edge_dists)
            
        self.wpstr = wpstr
        self.closest_dist = closest_dist
        self.cnstr = cnstr
        self.closest_edge_ids = closest_edge_ids
        self.closest_edge_dists = closest_edge_dists
        

    def MapCallback(self, msg):
        """
        This function receives the Topological Map
        """
        if(self.rec_map is False):
            self.names_by_topic = []
            self.nodes_by_topic = []
            self.nogos = []

            self.tmap = json.loads(msg.data) 
            self.tmap_frame = self.tmap["transformation"]["child"]
            self.get_logger().info("Localisation received the Topological Map")
            
            self.get_edge_vectors()
            self.update_loc_by_topic()
            
            self.get_logger().info("Creating localise by topic subscribers...")

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
            
            self.get_logger().info("NODES BY TOPIC: %s" %self.names_by_topic)
            self.get_logger().info("Listening to the tf transform between {} and {}".format(self.tmap_frame, self.base_frame))
            self.rec_map = True
            
            
    def get_edge_vectors(self):
        
        node_poses = {}
        for node in self.tmap["nodes"]:
            node_poses[node["node"]["name"]] = node["node"]["pose"]
        
        self.dist_edge_ids = []
        vectors_start = []
        vectors_end = []
        
        for node in self.tmap["nodes"]:
            orig_pose = node_poses[node["node"]["name"]]
            start = [orig_pose["position"]["x"], orig_pose["position"]["y"], 0]
            
            for edge in node["node"]["edges"]:
                dest_pose = node_poses[edge["node"]]
                
                if node["node"]["name"] != edge["node"]:
                    self.dist_edge_ids.append(edge["edge_id"])
                    end = [dest_pose["position"]["x"], dest_pose["position"]["y"], 0]
                    
                    vectors_start.append(start)
                    vectors_end.append(end)
                else:
                    self.get_logger().error("Cannot get distance to edge {}: Destination is equal to origin".format(edge["edge_id"]))
        
        self.vectors_start = np.array(vectors_start)
        self.vectors_end = np.array(vectors_end)


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
                    

    def get_nodes_wtag_cb(self, req, res):
        res.nodes = []
        try:
            cli = self.create_client(GetTaggedNodes, '/topological_map_manager2/get_tagged_nodes')
            if not cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning('/topological_map_manager2/get_tagged_nodes service not available')
                return res 
            self.service_get_tagged_done_event.clear()
            event  = Event()
            def done_callback(future):
                nonlocal event 
                event.set() 

            cli_req = GetTaggedNodes.Request()
            cli_req.tag = req.tag 
            cli_future = cli.call_async(req)
            cli_future.add_done_callback(done_callback)
            event.wait()
            get_prediction = cli_future.result()
            # self.get_logger().info(get_prediction)
            tagnodes = get_prediction.nodes
            # for x in self.distances:
            ldis = [x["node"]["node"]["name"] for x in self.distances]
            for i in ldis:
                if i in tagnodes:
                    res.nodes.append(i)
            return res 
        except (Exception) as e:
            self.get_logger().error("Service call /topological_map_manager2/get_tagged_nodes failed: %s"%e)
            return res 


    def localise_pose_cb(self, req, res):
        """
        This function gets the node and closest node for a pose
        """
        not_loc = True
        distances = []
        distances = self.get_distances_to_pose(req.pose)
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
        res.current_node = currentstr
        res.closest_node = closeststr 
        return res 


    def get_no_go_nodes(self):
        """
        This function gets the list of No go nodes
        """
        cli = self.create_client(GetTaggedNodes, '/topological_map_manager2/get_tagged_nodes')
        # while not self.cli.wait_for_service(timeout_sec=3.0):
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('/topological_map_manager2/get_tagged_nodes service not available')
            return []
        else:
            cli_req = GetTaggedNodes.Request()
            get_prediction = cli.call(cli_req)
            return get_prediction.nodes

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
def main(args=None):
    rclpy.init(args=args)
    wtags = True
    node = TopologicalNavLoc('topological_localisation', wtags)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('shutting down localisation node\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
    main()


