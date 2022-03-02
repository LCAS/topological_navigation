#!/usr/bin/env python
"""
Created on Fri Jan 22 10:34:58 2021

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import unittest, rostest
import rospy, json

from std_msgs.msg import String
from topological_navigation_msgs.srv import GetEdgesBetweenNodes, GetEdgesBetweenNodesRequest

PKG = "topological_navigation"


class MapManagerTester(unittest.TestCase):
    
    def __init__(self, *args): 
        super(MapManagerTester, self).__init__(*args)
        rospy.init_node('map_manager_tester')

        self.map_received = False
        rospy.Subscriber("/topological_map_2", String, self.map_callback)
        

    def map_callback(self, msg):
        self.tmap2 = json.loads(msg.data)        
        self.map_received = True
        
        
    def test_map_manager(self):
        
        rospy.sleep(5.0)
        self.assertTrue(self.map_received)
        
        try:
            s = rospy.ServiceProxy("/topological_map_manager2/get_edges_between_nodes", GetEdgesBetweenNodes)
            s.wait_for_service(timeout=5.0)
            
            req = GetEdgesBetweenNodesRequest()
            req.nodea = "WayPoint1"
            req.nodeb = "WayPoint2"
            res = s(req)
            
            self.assertGreater(len(res.ids_a_to_b), 0)
            self.assertGreater(len(res.ids_b_to_a), 0)
            
            self.assertEqual(res.ids_a_to_b[0], "WayPoint1_WayPoint2")
            self.assertEqual(res.ids_b_to_a[0], "WayPoint2_WayPoint1")
            
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logfatal(e)
            self.assertTrue(False)
#########################################################################################################    


#########################################################################################################
if __name__ == "__main__":
    rostest.rosrun(PKG, "map_manager_tester", MapManagerTester)
#########################################################################################################