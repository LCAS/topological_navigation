#! /usr/bin/env python

import rospy
import yaml
import sys

#from sensor_msgs.msg import Image
import cv2
import cv
#from cv_bridge import CvBridge
#from matplotlib import pyplot as plt
import numpy as np


from geometry_msgs.msg import Point

from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.tmap_utils import *
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.msg import TopologicalNode
from nav_msgs.msg import OccupancyGrid

def draw_arrow(image, V1, V2, color, origin, arrow_magnitude=5, thickness=1, line_type=8, shift=0):
    # adapted from http://mlikihazar.blogspot.com.au/2013/02/draw-arrow-opencv.html

    xval = (int(V1.x/0.05))+origin[0]#map2d.info.resolution))+origin[0]
    yval = origin[1]+(int(V1.y/0.05))#map2d.info.resolution))

    V3 = Point()
    V3.x = (V1.x+V2.x)/2
    V3.y = (V1.y+V2.y)/2
    
    p=(xval,yval)
    
    xval2 = (int(V3.x/0.05))+origin[0]#map2d.info.resolution))+origin[0]
    yval2 = origin[1]+(int(V3.y/0.05))#map2d.info.resolution))    
    q=(xval2,yval2)
    
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
        #print "loading %s"%filename
        #yaml_data=open(filename, "r")
        
        
        map2d = self.get2dmap()
        #print map2d.info
        dat= [128 if i < 0 else ((100-i)*2.5) for i in map2d.data]

        points = self.loadTMap()
        points.nodes.sort(key=lambda node: node.name)        
        
        
        imgdata = np.asarray(dat, dtype = np.uint8)
        imgdata = imgdata.reshape((map2d.info.height,map2d.info.width), order='C')
        #imgdata = cv2.flip(imgdata, 0)
        imgdata = cv2.cvtColor(imgdata, cv.CV_GRAY2BGRA);
                

        origin=[]
        origin.append(int(-(map2d.info.origin.position.x/map2d.info.resolution)))
        origin.append(int(-(map2d.info.origin.position.y/map2d.info.resolution)))
        
        thick=1
        for i in points.nodes:
            V1=i.pose.position
            xval = (int(V1.x/map2d.info.resolution))+origin[0]
            yval = origin[1]+(int(V1.y/map2d.info.resolution))
            print xval, yval
            cv2.circle(imgdata, (int(xval), int(yval)), 8, (0,0,255,255), 1)
            for j in i.edges:
                V2 = get_node(points, j.node).pose.position
                draw_arrow(imgdata, V1, V2, (255,0,0,(thick*25)), origin, thickness=2, line_type=1)
                if thick<10:
                    thick+=1
                else:
                    thick=1
        
        imgdata = cv2.flip(imgdata, 0)
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)       
        cv2.imshow('image', imgdata)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        
        cv2.imwrite('/home/jaime/playmap.png',imgdata)
#        threshed = cv2.adaptiveThreshold(im, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
#        cv2.imwrite('/home/jaime/2map.jpg',threshed)
        
        #img = Image.fromarray(imgdata, 'RGB')
        #img.save('my.png')
        #print points


    def get2dmap(self) :
        try:
            msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=10.0)
            #self.top_map = msg
            #self.lnodes = msg.nodes
            return msg
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return


    def loadTMap(self) :
        try:
            msg = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=10.0)
            #self.top_map = msg
            #self.lnodes = msg.nodes
            return msg
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return


if __name__ == '__main__':
    rospy.init_node('draw_map')
    filename = '/home/jaime/test_aaf/maps/topo/aaf_slam_map/aaf3.yaml'
    server = DrawMap(filename)
