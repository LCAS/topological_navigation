#! /usr/bin/env python

import rospy
import yaml
import sys

#from sensor_msgs.msg import Image
import cv2
import cv

from datetime import tzinfo, timedelta, datetime
import pytz
#from pytz import UTC

import matplotlib as mpl
import matplotlib.cm as cm
import numpy as np


from geometry_msgs.msg import Point

from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.tmap_utils import *
from topological_navigation_msgs.msg import TopologicalMap
from topological_navigation_msgs.msg import TopologicalNode
from nav_msgs.msg import OccupancyGrid
from topological_navigation_msgs.srv import *




def usage():
    print "\nFor one image of the current map:"
    print "\t rosrun topological_utils draw_predicted_map.py"
    print "\nFor one image of one specific timestamp:"
    print "\t rosrun topological_utils draw_predicted_map.py -time epoch"
    print "For images in a range use (every two hours):"
    print "\t rosrun topological_utils draw_predicted_map.py -range from_epoch to_epoch"
    print "\n\n"
    #print "For all the stats from a date until now use:"
    #print "\t rosrun topological_navigation topological_prediction.py -range from_epoch -1"
    #print "For all the stats until one date:"
    #print "\t rosrun topological_navigation topological_prediction.py -range 0 to_epoch"
    #https://en.wikipedia.org/wiki/List_of_tz_database_time_zones


def predict_edges(epoch):
    rospy.wait_for_service('topological_prediction/predict_edges')
    try:
        get_prediction = rospy.ServiceProxy('topological_prediction/predict_edges', topological_navigation_msgs.srv.PredictEdgeState)
        print "Requesting prediction for %s"%epoch
        resp1 = get_prediction(epoch)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def rotate_about_center(src, angle, scale=1.):
    w = src.shape[1]
    h = src.shape[0]
    rangle = np.deg2rad(angle)  # angle in radians
    # now calculate new image width and height
    nw = (abs(np.sin(rangle)*h) + abs(np.cos(rangle)*w))*scale
    nh = (abs(np.cos(rangle)*h) + abs(np.sin(rangle)*w))*scale
    # ask OpenCV for the rotation matrix
    rot_mat = cv2.getRotationMatrix2D((nw*0.5, nh*0.5), angle, scale)
    # calculate the move from the old center to the new center combined
    # with the rotation
    rot_move = np.dot(rot_mat, np.array([(nw-w)*0.5, (nh-h)*0.5,0]))
    # the move only affects the translation, so update the translation
    # part of the transform
    rot_mat[0,2] += rot_move[0]
    rot_mat[1,2] += rot_move[1]
    return cv2.warpAffine(src, rot_mat, (int(math.ceil(nw)), int(math.ceil(nh))), flags=cv2.INTER_LANCZOS4)


def draw_arrow(image, V1, V2, color, origin, arrow_magnitude=5, thickness=1, line_type=8, shift=0):
    xval = (int(V1.x/0.05))+origin[0]
    yval = origin[1]+(int(V1.y/0.05))

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

    def __init__(self, epoch, props):
        freq =7200
        self.norm = mpl.colors.Normalize(vmin=0, vmax=100)
        self.cmap = cm.jet
        self.colmap = cm.ScalarMappable(norm=self.norm, cmap=self.cmap)

        map2d = self.get2dmap()
        dat= [128 if i < 0 else ((100-i)*2.5) for i in map2d.data]

        points = self.loadTMap()
        points.nodes.sort(key=lambda node: node.name)        
        map2dimg = self.draw_2d_map(dat, map2d.info)

        if 'rotate' in props:
            self.rotate=props['rotate']
        else:
            self.rotate=0

        if 'scale' in props:
            self.scale=props['rotate']
        else:
            self.scale=1.0
        
        
        if 'tz' in props:
            self.timezone = pytz.timezone(props['tz'])
        else:
            self.timezone = pytz.timezone('GMT')

        if len(epoch) <2:
            self.epoch = epoch[0]
            self.max_vel=0.01

            est = predict_edges(self.epoch)
            topmapimg = self.draw_top_map(points, map2d.info, map2dimg, est)
            self.save_images(map2dimg, topmapimg)
        else :
            print "DO THIS"
            print epoch
            self.epoch = epoch[0]

            while self.epoch.secs <= epoch[1].secs:
                print self.epoch.secs
                self.max_vel=0.01
                est = predict_edges(self.epoch)
                topmapimg = self.draw_top_map(points, map2d.info, map2dimg, est)
                self.save_images(map2dimg, topmapimg)
                self.epoch = self.epoch + rospy.Duration(freq)
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
                indx= est.edge_ids.index(j.edge_id)
                #print j.edge_id, est.probs[est.edge_ids.index(j.edge_id)], est.durations[est.edge_ids.index(j.edge_id)]
                dist=math.hypot((V1.x-V2.x),(V1.y-V2.y))
                vel = (dist/float(est.durations[indx].secs))
                if vel > self.max_vel:
                    self.max_vel=round(vel,2)
                    
        
        for i in points.nodes:
            V1=i.pose.position
            for j in i.edges:
                V2 = get_node(points, j.node).pose.position
                indx= est.edge_ids.index(j.edge_id)
                #print j.edge_id, est.probs[est.edge_ids.index(j.edge_id)], est.durations[est.edge_ids.index(j.edge_id)]
                dist=math.hypot((V1.x-V2.x),(V1.y-V2.y))
                vel = round((dist/float(est.durations[indx].secs))/self.max_vel,2)
                print j.edge_id, vel#cd 
                a= self.colmap.to_rgba(int(vel*100))
                thick = int(est.probs[indx]*5)
                draw_arrow(topmap_image, V1, V2, (int(a[0]*255),int(a[1]*255),int(a[2]*255),255), origin, thickness=thick, arrow_magnitude=thick+5, line_type=1)
            xval = (int(V1.x/info.resolution))+origin[0]
            yval = origin[1]+(int(V1.y/info.resolution))
            #print xval, yval
            cv2.circle(topmap_image, (int(xval), int(yval)), 10, (0,0,255,255), -1)
        

        return topmap_image


    def save_images(self, map2dimg, topmapimg):
        topmapimg = cv2.flip(topmapimg, 0)
        print topmapimg.shape
        topmapimg = rotate_about_center(topmapimg, self.rotate, self.scale)
        
        size = topmapimg.shape[0]+200, topmapimg.shape[1]+200, 3

        out_image = np.zeros(size, dtype=np.uint8)
        out_image = cv2.cvtColor(out_image, cv.CV_BGR2BGRA);
        
        out_image[100:100+topmapimg.shape[0],100:100+topmapimg.shape[1]] = topmapimg


        ts = datetime.fromtimestamp(self.epoch.secs, tz=pytz.UTC).astimezone(self.timezone).strftime('%a %d %b %Y %H:%M:%S (%Z)')        
        height, width, depth = out_image.shape
        font = cv2.FONT_HERSHEY_SIMPLEX
        size1 = cv2.getTextSize(ts, font, 2, 5)
        hval = (width-size1[0][0])/2
        cv2.putText(out_image,ts,(hval, size1[0][1]+10), font, 2,(255,255,255,255),5)
        
        
        size1 = cv2.getTextSize('Predicted Speed (m/s)', font, 1.5, 5)
        hval = (width-size1[0][0])/2
        vval = height-95+size1[0][1]
        cv2.putText(out_image,'Predicted Speed (m/s)',(hval, vval), font, 1.2,(255,255,255,255),3)
        
        step=(width*0.8)/100
        #step=int((width)/100)
        barwend=(width*0.1)
        print width, step, barwend

        bareend=width*0.9
        for i in range(0, 100):
            a= self.colmap.to_rgba(i)
            v1=int(round(barwend+(step*(i))))
            v2=int(round(v1+step))
            cv2.rectangle(out_image, (v1,height-50), (v2,height), (int(a[0]*255),int(a[1]*255),int(a[2]*255),255), -1)

        size1 = cv2.getTextSize('0', font, 1.5, 5)
        hval = int(round(barwend))-size1[0][0]
        vval = height-50+size1[0][1]
        cv2.putText(out_image,'0',(hval, vval), font, 1.2,(255,255,255,255),3)

        size1 = cv2.getTextSize(str(self.max_vel), font, 1.5, 5)
        hval = int(round(bareend)+5)#-size1[0][0]
        vval = height-50+size1[0][1]
        cv2.putText(out_image,str(self.max_vel),(hval, vval), font, 1.2,(255,255,255,255),3)

        #print barwend+step*(101)
        #cv2.line(out_image,(barwend+(step*(101)),height-50), (barwend+(step*(101)),height), (255,255, 255, 255), 6)
        datestr = datetime.fromtimestamp(self.epoch.secs).strftime('%Y-%m-%d_%H-%M-%S')
        filename = 'predmap_'+datestr+'.png'
        cv2.imwrite(filename,out_image)



    def get2dmap(self) :
        try:
            msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=10.0)
            return msg
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return


    def loadTMap(self) :
        try:
            msg = rospy.wait_for_message('topological_map', TopologicalMap, timeout=10.0)
            return msg
        except rospy.ROSException :
            rospy.logwarn("Failed to get topological map")
            return


if __name__ == '__main__':
    rospy.init_node('draw_map')
    epochs=[]
    props={}
    #if len(sys.argv) < 2:
    if '-h' in sys.argv or '--help' in sys.argv:
        usage()
        sys.exit(1)
    else:
        if '-range' in sys.argv:
            ind = sys.argv.index('-range')
            epochs.append(rospy.Time.from_sec(float(sys.argv[ind+1])))
            epochs.append(rospy.Time.from_sec(float(sys.argv[ind+2])))
            print epochs
        elif '-time' in sys.argv:
            ind = sys.argv.index('-time')
            epochs.append(rospy.Time.from_sec(float(sys.argv[ind+1])))
            print epochs
        else:
            epochs.append(rospy.Time.now())
        
        if '-rotate' in sys.argv:
            ind = sys.argv.index('-rotate')
            props['rotate'] = float(sys.argv[ind+1])
        if '-scale' in sys.argv:
            ind = sys.argv.index('-scale')
            props['scale'] = float(sys.argv[ind+1])
        if '-tz' in sys.argv:
            ind = sys.argv.index('-tz')
            props['tz'] = str(sys.argv[ind+1])
    
    server = DrawMap(epochs, props)
