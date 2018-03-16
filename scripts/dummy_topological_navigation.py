#!/usr/bin/env python

import sys
import topological_navigation.testing
import rospy
from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.manager import map_manager
from topological_navigation.msg import GotoNodeAction, GotoNodeResult, GotoNodeFeedback
from strands_navigation_msgs.msg import *
import actionlib 
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from topological_navigation.load_maps_from_yaml import YamlMapLoader
import random

class DummyTopologicalNavigator():

    def __init__(self, size, simulate_time, map_name=None):
        if map_name is not None:
            self.map_name = map_name
        else:
            self.map_name = self.create_and_insert_map(size = size)
        
        self.manager = map_manager(self.map_name)
        self.node_names = set([node.name for node in self.manager.nodes.nodes])
        
        self.nav_result   =  GotoNodeResult()   
        self.nav_feedback = GotoNodeFeedback() 
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)

        self.policy_result   =  ExecutePolicyModeResult()   
        self.policy_feedback = ExecutePolicyModeFeedback()
        self.policy_server = actionlib.SimpleActionServer('topological_navigation/execute_policy_mode', ExecutePolicyModeAction, execute_cb = self.policy_callback, auto_start = False)

        self.cn_pub = rospy.Publisher('current_node', String, queue_size=1)
        self.cl_pub = rospy.Publisher('closest_node', String, queue_size=1)
        self.cn = random.choice(list(self.node_names))
        self.simulate_time = simulate_time
        
        if self.simulate_time:
            from strands_navigation_msgs.srv import PredictEdgeState
            time_srv_name = 'topological_prediction/predict_edges'
            try:
                rospy.wait_for_service(time_srv_name, timeout=120)
                self.time_srv = rospy.ServiceProxy(time_srv_name, PredictEdgeState)
                response = self.time_srv(rospy.get_rostime())
                self.edge_times = {response.edge_ids[i]: response.durations[i] for i in range(len(response.edge_ids))}
                # print self.edge_times

            except Exception, e:
                print e
                rospy.logwarn('travel time service not available')
            


    def start(self):
        self.nav_server.start()                 
        self.policy_server.start()                 
        while not rospy.is_shutdown():
             self.cn_pub.publish(String(self.cn))
             self.cl_pub.publish(String(self.cn))
             rospy.sleep(1)


    def policy_callback(self, goal):
        
        # print 'called with policy goal %s'%goal

        # this no longer seems valid
        # target is the one which is not in the source list
        # if len(goal.route.source) == 0:
        #     target_node = self.cn
        # else:
        #     target_node = self.node_names - set(goal.route.source)

        

        target_node = None
        
        if self.cn in goal.route.source:
            # get index of current state from source list, and map it to edge to take
            edge = goal.route.edge_id[goal.route.source.index(self.cn)]
            # split edge to get target waypoint
            target_node = edge.split('_')[1]

            print target_node

            if self.simulate_time:
                target = rospy.get_rostime() + self.edge_times[edge]
                while not rospy.is_shutdown() and not self.policy_server.is_preempt_requested() and rospy.get_rostime() < target:
                    rospy.sleep(0.5)
            else:
                rospy.sleep(1)

            
        if self.policy_server.is_preempt_requested():
            # print "done preempted"            
            self.policy_result.success = False
            self.policy_server.set_preempted(self.policy_result)
        elif target_node is None:


            # print "done failed to find target node"     
            self.policy_result.success = False       
            self.policy_server.set_succeeded()
        else:
            # print "done normal"     

            self.cn = target_node            

            self.policy_feedback.current_wp = self.cn 
            self.policy_feedback.status = GoalStatus.SUCCEEDED
            self.policy_server.publish_feedback(self.policy_feedback)

            rospy.sleep(0.1)

            self.policy_result.success = True       
            self.policy_server.set_succeeded(self.policy_result)
        
        # print "policy mode execution complete" 
     

    def nav_callback(self, goal):
        # print 'called with nav goal %s'%goal.target

        self.nav_feedback.route = 'Starting...'
        self.nav_server.publish_feedback(self.nav_feedback)

        self.nav_feedback.route = '%s to %s by %s' % (self.cn, goal.target, 'dummy_action')
        self.nav_server.publish_feedback(self.nav_feedback)

        # wait for completion or prempt

        if self.simulate_time:
            # target = rospy.get_rostime() + self.edge_times[edge]
            # lack of edge info so faking for now
            target = rospy.get_rostime() + rospy.Duration(20)
            while not rospy.is_shutdown() and not self.nav_server.is_preempt_requested() and rospy.get_rostime() < target:
                rospy.sleep(0.5)
        else:
            rospy.sleep(1)

        self.nav_feedback.route = goal.target
        self.nav_server.publish_feedback(self.nav_feedback)

        if self.nav_server.is_preempt_requested():
            # print "done preempted"            
            self.nav_result.success = False
            self.nav_server.set_preempted(self.nav_result)
        else:
            # print "done normal"     
            self.cn = goal.target            
            self.nav_result.success = True       
            self.nav_server.set_succeeded(self.nav_result)
        

         # print "nav complete" 

    def create_and_insert_map(self, size = 5, separation = 5.0):
        self.nodes = topological_navigation.testing.create_line_map(width = size, nodeSeparation = separation)
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
    size = rospy.get_param('~size', 20)
    sim_times = rospy.get_param('~simulate_time', False)
    map_name =  rospy.get_param('~map', None)
    map_file = rospy.get_param('~yaml_map', None)

    if map_file is not None and len(map_file) > 0:
        rospy.loginfo('loading map from yaml: %s' % map_file)
        map_loader = YamlMapLoader()
        data = map_loader.read_maps(map_file)
        map_loader.insert_maps(data=data, force=True)
        map_name = data[0][0]['node']['pointset']        
        rospy.set_param('topological_map_name', map_name)        

    elif map_name is not None and len(map_name) > 0:
        rospy.loginfo('simulating map: %s' % map_name)
        rospy.set_param('topological_map_name', map_name)        

    else:
        rospy.set_param('topological_map_name', 'dummy_map')

    navigator = DummyTopologicalNavigator(size, sim_times, map_name)        
    navigator.start()






       
        


        

   