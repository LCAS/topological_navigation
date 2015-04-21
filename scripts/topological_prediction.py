#!/usr/bin/env python

import sys
import rospy
import actionlib
import pymongo
import json
import sys
import math
import time

from datetime import datetime

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from nav_msgs.srv import *


import strands_navigation_msgs.msg
from strands_navigation_msgs.msg import TopologicalNode
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import NavStatistics
from strands_navigation_msgs.msg import TopologicalMap

from topological_navigation.tmap_utils import *

from strands_navigation_msgs.srv import *
import fremenserver.msg



def get_model(name, models):
    for i in models:
        if i["model_id"] == name:
            return i


class TopologicalNavPred(object):

    _feedback = strands_navigation_msgs.msg.BuildTopPredictionFeedback()
    _result   = strands_navigation_msgs.msg.BuildTopPredictionResult()

    def __init__(self, name) :
        self.edgid=[]
        self.lnodes = []
        self.eids = []
        self.unknowns = []
        self.map_received =False

        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        
        rospy.loginfo("Waiting for Topological map ...")
        
        while not self.map_received:
            pass

        rospy.loginfo("Creating fremen server client.")
        self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
        self.FremenClient.wait_for_server()
        rospy.loginfo(" ...done")


        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(name, strands_navigation_msgs.msg.BuildTopPredictionAction, execute_cb = self.BuildCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        self.get_list_of_edges()
        rospy.loginfo("Gathering Data")
        self.gather_stats()
        rospy.loginfo(" ...done")

        self.predict_srv=rospy.Service('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState, self.predict_edge_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()


    def get_predict(self, epoch):
        print "requesting prediction for time %d" %epoch
        edges_ids=[]
        dur=[]
        #prob=[]
        
        eids = [x['edge_id'] for x in self.models]
        mods = [x['model_id'] for x in self.models]
        ords = [x['order'] for x in self.models]


        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'forecast'
        fremgoal.ids = mods
        fremgoal.times.append(epoch)
        #print i["order"]
        fremgoal.order = -1
        fremgoal.orders = ords#i["order"]
        
        self.FremenClient.send_goal(fremgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.FremenClient.get_result()  # A FibonacciResult

        print ps

        prob = list(ps.probabilities)

        print mods
        print ords
        print prob        
        
        for j in range(len(mods)):
            
            i=get_model(mods[j], self.models)
            edges_ids.append(eids[j])
         
            #print ps.probabilities[0]
            if prob[j] < 0.01 :
                prob[j] = 0.01
            
            dur_c=[]
            if prob[j] >=0.1:
                est_dur = i["dist"]/prob[j]
                dur_c.append(est_dur)
            else :
                est_dur = i["dist"]/0.1
                dur_c.append(est_dur)
            
            
            for j in i['models']:
                if j['st'] :
                    dur_c.append(j['optime'])
                    
            ava= rospy.Duration(sum(dur_c) / float(len(dur_c)))
            dur.append(ava)


       
        for i in self.unknowns:
            edges_ids.append(i["edge_id"])
            prob.append(0.5)
            est_dur = rospy.Duration(i["dist"]/0.5)
            dur.append(est_dur)


        #print edges_ids, prob, dur
        return edges_ids, prob, dur
        

    def predict_edge_cb(self, req):
        print req
        return self.get_predict(req.epoch.secs)


    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self.map_received = True
        #print self.lnodes


    def get_list_of_edges(self):
        #self.eids = []
        
        rospy.loginfo("Querying for list of edges")
        for i in self.lnodes.nodes :
            for j in i.edges:
                if j.edge_id not in self.edgid :
                    self.edgid.append(j.edge_id)
                    val={}
                    val["edge_id"]=j.edge_id
                    val["model_id"]=self.lnodes.name+'__'+j.edge_id
                    val["ori"]=i.name
                    val["dest"]=j.node
                    ddn=get_node(self.lnodes, j.node)
                    if j.top_vel>= 0.1:
                        val["dist"]= get_distance_to_node(i,ddn)/j.top_vel
                    else :
                        val["dist"]= get_distance_to_node(i,ddn)/0.1
                    self.eids.append(val)
        fdbmsg = 'Done. %d edges found' %len(self.edgid)
        rospy.loginfo(fdbmsg)


    """
     BuildCallBack
     
     This Functions is called when the Action Server is called to build the models again
    """
    def BuildCallback(self, goal):
        self.cancelled = False

        start_time = time.time()        
        self.get_list_of_edges()
        elapsed_time = time.time() - start_time
        self._feedback.result = "%d edges found in %.3f seconds \nGathering stats ..." %(len(self.eids),elapsed_time)
        self._as.publish_feedback(self._feedback)
        self.gather_stats()        
        
        elapsed_time = time.time() - start_time
        self._feedback.result = "Finished after %.3f seconds" %elapsed_time 
        self._as.publish_feedback(self._feedback)       #Publish Feedback

        print "Finished after %.3f seconds" %elapsed_time
        
        if not self.cancelled :     
            self._result.success = True
            self._as.set_succeeded(self._result)
        else:
            self._result.success = False
            self._as.set_preempted(self._result)


    def preemptCallback(self):
        self.cancelled = True



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
            edge_mod["order"]=-1
            edge_mod["edge_id"]=i["edge_id"]
            
            for j in available:                
                val = {}
                if j[0].status == 'success':
                    val["st"] = 1
                else:
                    val["st"] = 0
                val["epoch"] = int(datetime.strptime(j[0].date_started, "%A, %B %d %Y, at %H:%M:%S hours").strftime('%s'))
                val["optime"] = j[0].operation_time
                edge_mod["models"].append(val)
                
            if len(available) > 0 :
                to_add.append(edge_mod)
            else :
                self.unknowns.append(edge_mod)
                
        self.create_fremen_models(to_add)



    def create_fremen_models(self, models):
        self.models = models
        for i in models:
            print "-------CREATING MODEL----------"
            
            mid = i["model_id"]
            print mid
            #print i["dist"]
            times=[]
            states=[]
            for j in i["models"]:
                times.append(j["epoch"])
                states.append(j["st"])
            #print times
            #print states
            fremgoal = fremenserver.msg.FremenGoal()
            fremgoal.operation = 'add'
            fremgoal.id = mid
            fremgoal.times = times
            fremgoal.states = states
            
            # Sends the goal to the action server.
            self.FremenClient.send_goal(fremgoal)
            
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
            
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()  # A FibonacciResult
            #print ps
            
            print "--- EVALUATE ---"
            frevgoal = fremenserver.msg.FremenGoal()
            frevgoal.operation = 'evaluate'
            frevgoal.id = mid
            frevgoal.times = times
            frevgoal.states = states
            frevgoal.order = 5
            
            # Sends the goal to the action server.
            self.FremenClient.send_goal(frevgoal)
            
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
            
            # Prints out the result of executing the action
            pse = self.FremenClient.get_result()  # A FibonacciResult
            print pse.errors
            print "chosen order %d" %pse.errors.index(min(pse.errors))
            i["order"] = pse.errors.index(min(pse.errors))



if __name__ == '__main__':
    rospy.init_node('topological_prediction')
    server = TopologicalNavPred(rospy.get_name())