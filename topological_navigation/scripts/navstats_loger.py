#!/usr/bin/env python2


import calendar
from datetime import datetime

import rospy

from strands_navigation_msgs.msg import NavStatistics
from mongodb_store.message_store import MessageStoreProxy

class TopologicalNavStatsSaver(object):
    """
     Class for Topological Navigation stats logging
    
    """
    def __init__(self) :
        rospy.Subscriber('topological_navigation/Statistics', NavStatistics, self.statsCallback)
        
        rospy.spin()
        
    def statsCallback(self, msg):
        #print "---------------------------------------------------"
        meta = {}
        meta["type"] = "Topological Navigation Stat"
        epoch = datetime.strptime(msg.date_at_node, '%A, %B %d %Y, at %H:%M:%S hours').timetuple()# .time()
        #print calendar.timegm(epoch)
        meta["epoch"] = calendar.timegm(epoch)#calendar.timegm(self.stat.date_at_node.timetuple())
        meta["date"] = msg.date_at_node
        meta["pointset"] = msg.topological_map
        
        #print meta

        msg_store = MessageStoreProxy(collection='nav_stats')
        msg_store.insert(msg,meta)


if __name__ == '__main__':
    mode="normal"
    rospy.init_node('topological_navstats_logger')
    server = TopologicalNavStatsSaver()