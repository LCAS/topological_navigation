#!/usr/bin/env python

import json
import sys
import rospy


from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode
#from topological_utils.msg import vertex
#from topological_utils.msg import edge
#from geometry_msgs.msg import Pose
from ros_datacentre.message_store import MessageStoreProxy


if __name__ == '__main__':
    #filename=str(sys.argv[1])
    point_set=str(sys.argv[1])
    #map_name=str(sys.argv[3])

    msg_store = MessageStoreProxy()

    query_meta = {}
    query_meta["pointset"] = point_set
    available = len(msg_store.query(node._type, {}, query_meta)) > 0

    if available <= 0 :
        rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
        rospy.logerr("Available pointsets: "+str(available))
        raise Exception("Can't find waypoints.")

    else :
        query_meta = {}
        query_meta["pointset"] = point_set
        message_list = msg_store.query(node._type, {}, query_meta)

        points = []
        for i in message_list:
            #print i[0].name
            b = topological_node(i[0].name)
            edges = []
            for j in i[0].edges :
                data = {}
                data["node"]=j.node
                data["action"]=j.action
                edges.append(data)
            b.edges = edges
            
            verts = []
            for j in i[0].verts :
                data = [j.x,j.y]
                verts.append(data)
            b._insert_vertices(verts)

            c=i[0].pose
            waypoint=[str(c.position.x), str(c.position.y), str(c.position.z), str(c.orientation.x), str(c.orientation.y), str(c.orientation.z), str(c.orientation.w)]
            b.waypoint = waypoint
            b._get_coords()

            points.append(b)
        
        for point in points :
            print point.__dict__
