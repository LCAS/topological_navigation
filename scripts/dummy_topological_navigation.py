#!/usr/bin/env python

import sys
import topological_navigation.testing
import rospy
from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.publisher import map_publisher
from topological_navigation.msg import GotoNodeAction, GotoNodeResult, GotoNodeFeedback
import actionlib 
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

class DummyTopologicalNavigator():

    def __init__(self, size, simulate_time, map_name=None):
        if map_name:
            self.map_name = map_name
        else:
            self.map_name = self.create_and_insert_map(size = size)
        self.publisher = map_publisher(self.map_name)
        self.nav_result   =  GotoNodeResult()   
        self.nav_feedback = GotoNodeFeedback() 
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)
        self.cn_pub = rospy.Publisher('/current_node', String, queue_size=1)
        self.cl_pub = rospy.Publisher('/closest_node', String, queue_size=1)
        self.cn = 'ChargingPoint'
        self.simulate_time = simulate_time
        self.time_srv = None
        if self.simulate_time:
            from strands_navigation_msgs.srv import EstimateTravelTime
            time_srv_name = 'topological_navigation/travel_time_estimator'
            try:
                rospy.wait_for_service(time_srv_name, timeout=10)
                self.time_srv = rospy.ServiceProxy(time_srv_name, EstimateTravelTime)
            except Exception, e:
                rospy.logwarn('travel time service not available')
            


    def start(self):
        self.nav_server.start()                 
        while not rospy.is_shutdown():
             self.cn_pub.publish(String(self.cn))
             self.cl_pub.publish(String(self.cn))
             rospy.sleep(1)


    def nav_callback(self, goal):
        print 'called with nav goal %s'%goal.target

        self.nav_feedback.route = 'Starting...'
        self.nav_server.publish_feedback(self.nav_feedback)

        self.nav_feedback.route = '%s to %s by %s' % (self.cn, goal.target, 'dummy_action')
        self.nav_server.publish_feedback(self.nav_feedback)

        # wait for completion or prempt

        if self.time_srv:
            target = rospy.get_rostime() + self.time_srv(self.cn, goal.target).travel_time
            while not rospy.is_shutdown() and not self.nav_server.is_preempt_requested() and rospy.get_rostime() < target:
                rospy.sleep(0.5)
        else:
            rospy.sleep(1)

        if self.nav_server.is_preempt_requested():
            print "done preempted"            
            self.nav_result.success = False
            self.nav_server.set_preempted(self.nav_result)
        else:
            print "done normal"     
            self.cn = goal.target            
            self.nav_result.success = True       
            self.nav_server.set_succeeded(self.nav_result)
        
        self.nav_feedback.route = goal.target
        self.nav_server.publish_feedback(self.nav_feedback)

        print "nav complete" 

    def create_and_insert_map(self, size = 5, separation = 5.0):
        self.nodes = topological_navigation.testing.create_cross_map(width = size, height = size, nodeSeparation = separation)
        self.top_map_store = MessageStoreProxy(collection='topological_maps')

        map_name = 'dummy_map'

        meta = {}
        meta['map'] = map_name
        meta['pointset'] = map_name

        for (node_name, node) in self.nodes.iteritems():
            meta["node"] = node_name
            node.map = meta['map']
            node.pointset = meta['pointset']
            self.top_map_store.insert(node,meta)
        
        return map_name
    


if __name__ == '__main__':
    rospy.init_node('dummy_topological_navigator')
    size = rospy.get_param('~size', 5)
    sim_times = rospy.get_param('~simulate_time', False)
    map_name =  rospy.get_param('~map', None)
    if map_name:
        rospy.loginfo('simulating map: %s' % map_name)
        rospy.set_param('topological_map_name', map_name)        
    else:
        rospy.set_param('dummy_map', map_name)

    navigator = DummyTopologicalNavigator(size, sim_times, map_name)        
    navigator.start()






       
        


        

   