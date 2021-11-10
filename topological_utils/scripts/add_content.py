#!/usr/bin/env python

import sys
import rospy
import topological_navigation_msgs.srv 


def content_client(node, content):
    rospy.wait_for_service('/topological_map_manager/add_content_to_node')
    try:
        cont = rospy.ServiceProxy('/topological_map_manager/add_content_to_node', topological_navigation_msgs.srv.AddContent)
        resp1 = cont(node, content)
        return resp1
    except rospy.ServiceException as e:
        return "Service call failed: %s"%e
    

if __name__ == "__main__":
    print(sys.argv)

    filename=str(sys.argv[1])
    waypoint=str(sys.argv[2])
    #map_name=str(sys.argv[3])

    json_data=open(filename, 'rb').read()
    #text0 = "Hello ROS World"
    print("Sending %s"%(json_data))
    resp=content_client(waypoint, json_data)
    print(resp)
    
