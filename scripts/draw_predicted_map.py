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
from strands_navigation_msgs.srv import *



def predict_edges(epoch):
    rospy.wait_for_service('/topological_prediction/predict_edges')
    try:
        get_prediction = rospy.ServiceProxy('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState)
        #now = 
        #prediction_time = now + rospy.Duration(seconds_from_now) 
        print "Requesting prediction for %s"%epoch
        resp1 = get_prediction(epoch)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


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

    def __init__(self, epoch):
        map2d = self.get2dmap()
        dat= [128 if i < 0 else ((100-i)*2.5) for i in map2d.data]

        points = self.loadTMap()
        points.nodes.sort(key=lambda node: node.name)        

        map2dimg = self.draw_2d_map(dat, map2d.info)
        est = predict_edges(epoch)
        topmapimg = self.draw_top_map(points, map2d.info, map2dimg, est)
        self.save_images(map2dimg, topmapimg)
        #print est.edge_ids
        
        
    def draw_2d_map(self, data, info):
        imgdata = np.asarray(data, dtype = np.uint8)
        imgdata = imgdata.reshape((info.height, info.width), order='C')
        imgdata = cv2.cvtColor(imgdata, cv.CV_GRAY2BGRA);
        return imgdata


    def draw_top_map(self, points, info, map2dimg, est):
        topmap_image = map2dimg.copy()

        origin=[]
        origin.append(int(-(info.origin.position.x/info.resolution)))
        origin.append(int(-(info.origin.position.y/info.resolution)))
        
        #thick=1        
        for i in points.nodes:
            V1=i.pose.position
            for j in i.edges:
                V2 = get_node(points, j.node).pose.position
                #print j.edge_id, est.probs[est.edge_ids.index(j.edge_id)], est.durations[est.edge_ids.index(j.edge_id)]
                thick = int(est.probs[est.edge_ids.index(j.edge_id)]*5)
                draw_arrow(topmap_image, V1, V2, (255,0,0,255), origin, thickness=thick, arrow_magnitude=thick+5, line_type=1)
            xval = (int(V1.x/info.resolution))+origin[0]
            yval = origin[1]+(int(V1.y/info.resolution))
            #print xval, yval
            cv2.circle(topmap_image, (int(xval), int(yval)), 10, (0,0,255,255), -1)
        
        return topmap_image


    def save_images(self, map2dimg, topmapimg):
        #map2dimg = cv2.flip(map2dimg, 0)
        topmapimg = cv2.flip(topmapimg, 0)
        
        #img = cv2.addWeighted(map2dimg,0.1,topmapimg,1.0,0)
        
        #cv2.imwrite('playmap.png',map2dimg)
        cv2.imwrite('predmap.png',topmapimg)
        #cv2.imwrite('playmapb.png',img)



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
    
    if len(sys.argv) == 2:
        epoch = rospy.Time(secs=int(sys.argv[1]))
    else:
        epoch = rospy.Time.now()
    
    server = DrawMap(epoch)
