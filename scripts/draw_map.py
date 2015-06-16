#! /usr/bin/env python

import rospy
import yaml
import sys

#from sensor_msgs.msg import Image
import cv2
#from cv_bridge import CvBridge
#from matplotlib import pyplot as plt
import numpy as np

from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.tmap_utils import *
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.msg import TopologicalNode


def draw_arrow(image, p, q, color, arrow_magnitude=2, thickness=1, line_type=8, shift=0):
    # adapted from http://mlikihazar.blogspot.com.au/2013/02/draw-arrow-opencv.html
    
    # draw arrow tail
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # calc angle of the arrow
    angle = np.arctan2(p[1]-q[1], p[0]-q[0])
    # starting point of first line of arrow head
    p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle + np.pi/4)))
    # draw first half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # starting point of second line of arrow head
    p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle - np.pi/4)))
    # draw second half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)



class DrawMap(object):

    def __init__(self, filename):
        
        print "loading %s"%filename
        yaml_data=open(filename, "r")
    
        points = self.loadMap('aaf_deployment')
        points.nodes.sort(key=lambda node: node.name)
        #print points
        data = yaml.load(yaml_data)        
        print data
        
        photo = cv2.imread('/home/jaime/test_aaf/maps/topo/aaf_slam_map/aaf3.pgm',3)
        height, width, depth = photo.shape
        print height, width, depth
        origin=[]
        origin.append(int(-(data['origin'][0]/0.05)))
        origin.append(int(-(data['origin'][1]/0.05)))
        #origin.append(int(-(data['origin'][0]/0.05)))
        #origin.append(int(-(data['origin'][1]/0.05)))
#        origin.append(int(100))
#        origin.append(int(100))
        print origin
#        photo = 
        cv2.rectangle(photo,(origin[0]-50,origin[1]-50),(origin[0]+50,origin[1]+50),(0,255,0),5)
        for i in points.nodes:
            xval = (int(i.pose.position.x/0.05))+origin[0]
            yval = origin[1]-(int(i.pose.position.y/0.05))
            print xval, yval
            cv2.circle(photo, (int(xval), int(yval)), 8, (0,0,255), 1)
            for j in i.edges:
                targ_pos = get_node(points, j.node).pose.position
                xval2 = (int(targ_pos.x/0.05))+origin[0]
                yval2 = origin[1]-(int(targ_pos.y/0.05))
                draw_arrow(photo, (xval, yval), (xval2, yval2), (255,0,0))
        
        
        cv2.imwrite('/home/jaime/topmap.jpg',photo)
#        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
#        cv2.imshow('image', photo)
#        cv2.waitKey(0)
#        cv2.destroyAllWindows()
        
#        plt.imshow(photo, interpolation = 'bicubic')
#        plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
#        plt.show()




    def loadMap(self, point_set) :
        msg_store = MessageStoreProxy(collection='topological_maps')
        print "Loading Pointset %s" %point_set
        query_meta = {}
        query_meta["pointset"] = point_set

        # waiting for the map to be in the mongodb_store
        ntries=1
        map_found=False
        
        while not map_found :
            available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0
            #print available
            if available <= 0 :
                rospy.logerr("Desired pointset '"+point_set+"' not in datacentre, try :"+str(ntries))
                #rospy.logerr("Available pointsets: "+str(available))
                if ntries <=10 :
                    ntries+=1
                    rospy.sleep(rospy.Duration.from_sec(6))
                else :
                    raise Exception("Can't find waypoints.")
                    sys.exit(2)
            else:
                map_found=True
 
 
        query_meta = {}
        query_meta["pointset"] = point_set
              
        message_list = msg_store.query(TopologicalNode._type, {}, query_meta)

        points = TopologicalMap()
        points.name = point_set
        #points.map = point_set
        points.pointset = point_set
        #string last_updated
        for i in message_list:
            b = i[0]
            points.nodes.append(b)
        
        points.map = points.nodes[0].map
        return points
    


if __name__ == '__main__':
    rospy.init_node('draw_map')
    filename = '/home/jaime/test_aaf/maps/topo/aaf_slam_map/aaf3.yaml'
    server = DrawMap(filename)
