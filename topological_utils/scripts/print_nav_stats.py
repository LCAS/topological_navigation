#!/usr/bin/env python

import rospy
from topological_navigation_msgs.msg import TopologicalMap, NavStatistics, NavRoute
from mongodb_store.message_store import MessageStoreProxy
#from strands_executive_msgs.msg import Task
#from strands_executive_msgs.srv import AddTasks
#from strands_executive_msgs import task_utils

class NavRelaxant(object):
    def __init__(self, count_threshold):
        super(NavRelaxant, self).__init__()     
        self.node_pairs = []
        rospy.Subscriber('topological_map', TopologicalMap, self.map_callback)
        self.msg_store = MessageStoreProxy(collection='nav_stats')


    def map_callback(self, msg):            
        node_pairs = []
        for node in msg.nodes:
            for edge in node.edges:
                node_pairs.append((node.name, edge.node, edge.edge_id))                
        self.node_pairs = node_pairs

    def print_pair(self, start, end):
        count = len(self.msg_store.query(NavStatistics._type, {"origin": start, "target": end, "final_node": end}))
        rospy.loginfo('Nav stats from  %s to %s: %s' % (start, end, count))

    def print_nav_stats(self):

        # only really needed for testing
        while len(self.node_pairs) == 0 and not rospy.is_shutdown():
            rospy.sleep(1)
            rospy.loginfo('Waiting for nodes')


        for (start, end, edge_id) in self.node_pairs:
            self.print_pair(start, end)

if __name__ == '__main__':
    rospy.init_node('nav_relaxant')
    relaxant = NavRelaxant(count_threshold=rospy.get_param('~count_threshold', 5))
    relaxant.print_nav_stats()
