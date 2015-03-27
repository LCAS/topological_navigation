#!/usr/bin/env python

import sys
import rospy
import actionlib
import pymongo
import json
import sys
import math



from datetime import datetime


from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose
from std_msgs.msg import String
#import scitos_apps_msgs.msg

from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy

from strands_navigation_msgs.msg import NavStatistics

from strands_navigation_msgs.msg import TopologicalMap
#from topological_navigation.topological_node import *

from strands_navigation_msgs.srv import *
import fremenserver.msg


def get_node(name, clist):
    for i in clist:
        if i.name == name:
            return i

def node_dist(node1,node2):
    dist = math.sqrt((node1.pose.position.x - node2.pose.position.x)**2 + (node1.pose.position.y - node2.pose.position.y)**2 )
    return dist


class TopologicalNavPred(object):
       
    
    def __init__(self, name) :
        
        self.lnodes = []
        self.eids = []
        self.map_received =False

        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        
        rospy.loginfo("Waiting for Topological map ...")
        
        while not self.map_received:
            pass

        rospy.loginfo("Creating fremen server client.")
        self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
        self.FremenClient.wait_for_server()
        rospy.loginfo(" ...done")

        self.predict_srv=rospy.Service('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState, self.predict_edge_cb)
        #This service returns a list of nodes that have a given tag

        self.get_list_of_edges()
        #print "List of edges"
        #print self.eids
        
        self.gather_stats()
        
        rospy.loginfo("All Done ...")
        rospy.spin()


    def get_predict(self, epoch):
        print "requesting prediction for time %d" %epoch
        edges_ids=[]
        dur=[]
        prob=[]
        for i in self.models:
            edges_ids.append(i["model_id"])
            fremgoal = fremenserver.msg.FremenGoal()
            fremgoal.operation = 'predict'
            fremgoal.id = i["model_id"]
            fremgoal.times.append(epoch)
            fremgoal.order = 2
            
            self.FremenClient.send_goal(fremgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
        
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
        
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()  # A FibonacciResult
            #print ps.probabilities[0]
            prob.append(ps.probabilities[0])
            if ps.probabilities[0] >=0.1:
                dur.append(i["dist"]/ps.probabilities[0])
            else :
                dur.append(i["dist"]/0.1)

        #print edges_ids, prob, dur
        return edges_ids, prob, dur
        

    def predict_edge_cb(self, req):
        return self.get_predict(req.epoch)


    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self.map_received = True
        #print self.lnodes

    def get_list_of_edges(self):        
        for i in self.lnodes.nodes :
            for j in i.edges:
                if j.edge_id not in self.eids:
                    val={}
                    val["edge_id"]=j.edge_id
                    val["model_id"]=self.lnodes.name+'__'+j.edge_id
                    val["ori"]=i.name
                    val["dest"]=j.node
                    ddn=get_node(j.node, self.lnodes.nodes)
                    val["dist"]= node_dist(i,ddn)/j.top_vel
                    self.eids.append(val)


    def gather_stats(self):
        msg_store = MessageStoreProxy(collection='nav_stats')
        #db.topological_maps.find({ "_meta.tag":"AAA" })

        to_add=[]        
        for i in self.eids:
            
            query = {"topological_map": self.lnodes.name, "edge_id":i["edge_id"]}
            #query = {"_meta.tag": tag, "pointset": self.nodes.name}
            query_meta = {}
            query_meta["pointset"] = self.lnodes.name
    
            #print query, query_meta
            available = msg_store.query(NavStatistics._type, query, query_meta)
            
            edge_mod={}
            edge_mod["model_id"]= i["model_id"]#self.lnodes.name+'__'+i["edge_id"]
            edge_mod["dist"]= i["dist"]#self.lnodes.name+'__'+i["edge_id"]
            edge_mod["models"]=[]
            
            for j in available:                
                val = {}
                if j[0].status == 'success':
                    val["st"] = 1
                else:
                    val["st"] = 0
                val["epoch"] = int(datetime.strptime(j[0].date_started, "%A, %B %d %Y, at %H:%M:%S hours").strftime('%s'))
                edge_mod["models"].append(val)
                
            if len(available) > 0 :
                to_add.append(edge_mod)
                
        self.create_fremen_models(to_add)


    def create_fremen_models(self, models):
        self.models = models
        for i in models:
            print "-----------------"
            mid = i["model_id"]
            print mid
            print i["dist"]
            times=[]
            states=[]
            for j in i["models"]:
                times.append(j["epoch"])
                states.append(j["st"])
            print times
            print states
            fremgoal = fremenserver.msg.FremenGoal()
            fremgoal.operation = 'add'
            fremgoal.id = mid
            fremgoal.times = times
            fremgoal.states = states
        
            # Sends the goal to the action server.
            self.FremenClient.send_goal(fremgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
        
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
        
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()  # A FibonacciResult
            print ps

#        for i in available:
#            nname= i[1]['node']
#            a.append(nname)
#          
#        mm.append(a)
#
#        return mm


if __name__ == '__main__':
    rospy.init_node('topological_prediction')
    server = TopologicalNavPred(rospy.get_name())