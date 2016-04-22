#!/usr/bin/env python

import sys
import rospy
import actionlib
import pymongo
import json
import sys
import math
import time

from threading import Lock
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


def usage():
    print "\nFor using all the available stats use:"
    print "\t rosrun topological_navigation topological_prediction.py"
    print "For all the stats in a range use:"
    print "\t rosrun topological_navigation topological_prediction.py -range from_epoch to_epoch"
    print "For all the stats from a date until now use:"
    print "\t rosrun topological_navigation topological_prediction.py -range from_epoch -1"
    print "For all the stats until one date:"
    print "\t rosrun topological_navigation topological_prediction.py -range 0 to_epoch"

def get_model(name, models):
    for i in models:
        if i["model_id"] == name:
            return i


class TopologicalNavPred(object):

    _feedback = strands_navigation_msgs.msg.BuildTopPredictionFeedback()
    _result   = strands_navigation_msgs.msg.BuildTopPredictionResult()

    def __init__(self, epochs) :
        self.lnodes = []
        self.map_received =False
        self.range = epochs
        self.srv_lock=Lock()
        name= rospy.get_name()
        action_name = name+'/build_temporal_model'

        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        
        rospy.loginfo("Waiting for Topological map ...")
        
        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
            rospy.loginfo("Waiting for Topological map ...")

        rospy.loginfo("... Got Topological map")

        rospy.loginfo("Creating fremen server client.")
        self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
        self.FremenClient.wait_for_server()
        rospy.loginfo(" ...done")


        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(action_name, strands_navigation_msgs.msg.BuildTopPredictionAction, execute_cb = self.BuildCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        self.create_temporal_models()        
#        for i in self.models:
#            print i['model_id']
#            print i['time_model_id']

        self.predict_srv=rospy.Service('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState, self.predict_edge_cb)
        self.predict_srv=rospy.Service('/topological_prediction/edge_entropies', strands_navigation_msgs.srv.PredictEdgeState, self.edge_entropies_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()

    def create_temporal_models(self):
        self.edgid=[]
        self.eids = []
        self.unknowns = []

        self.get_list_of_edges()
        rospy.loginfo("Gathering Data")
        self.gather_stats()
        rospy.loginfo(" ...done")
        

    def get_predict(self, epoch):
        # print "requesting prediction for time %d" %epoch
        edges_ids=[]
        dur=[]
        
        eids = [x['edge_id'] for x in self.models]
        mods = [x['model_id'] for x in self.models]
        ords = [x['order'] for x in self.models]
        tids = [x['time_model_id'] for x in self.models]
        tords = [x['t_order'] for x in self.models]

        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'forecast'
        fremgoal.ids = mods
        fremgoal.times.append(epoch)
        
        #print i["order"]
        fremgoal.order = -1
        fremgoal.orders = ords#i["order"]
        
        self.FremenClient.send_goal(fremgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result(timeout=rospy.Duration(10.0))

        if self.FremenClient.get_state() == actionlib.GoalStatus.SUCCEEDED:
            
        
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()  # A FibonacciResult
    
            print ps
    
    
            prob = list(ps.probabilities)
    
            for j in range(len(mods)):
                if prob[j] < 0.01 :
                    prob[j] = 0.01
                i=get_model(mods[j], self.models)
                edges_ids.append(eids[j])
    
    
            fremgoal = fremenserver.msg.FremenGoal()
            fremgoal.operation = 'forecast'
            fremgoal.ids = tids
            fremgoal.times.append(epoch)
            #print i["order"]
            fremgoal.order = -1
            fremgoal.orders = tords#i["order"]
            
            self.FremenClient.send_goal(fremgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
        
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result(timeout=rospy.Duration(10.0))
        
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()  # A FibonacciResult
    
            print ps
    
            speeds = list(ps.probabilities)
    
            for j in range(len(mods)):
                if speeds[j]>0.01:
                    dur.append(rospy.Duration(self.models[j]["dist"]/speeds[j]))
                else:
                    dur.append(rospy.Duration(self.models[j]["dist"]/0.01))
    
           
            for i in self.unknowns:
                edges_ids.append(i["edge_id"])
                prob.append(0.5)
                est_dur = rospy.Duration(i["dist"]/0.1)
                speeds.append(0.1)
                dur.append(est_dur)
    
    
            for k in range(len(edges_ids)):
                print edges_ids[k], prob[k], dur[k].secs, speeds[k]
            return edges_ids, prob, dur
        elif not rospy.is_shutdown():
            rospy.logwarn("NO PREDICTIONS RECEIVED FROM FREMENSERVER WILL TRY AGAIN...")
            if not self.FremenClient.wait_for_server(rospy.Duration(10.0)):
                rospy.logerr("NO CONNECTION TO FREMENSERVER...")
                rospy.logwarn("WAITING FOR FREMENSERVER...")
                self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
                self.FremenClient.wait_for_server()
                rospy.loginfo(" ...done")
                self.create_temporal_models()
                rospy.logwarn("WILL TRY TO GET PREDICTIONS AGAIN...")
            return self.get_predict(epoch)
        

    def predict_edge_cb(self, req):
        with self.srv_lock:
            return self.get_predict(req.epoch.secs)
            


    def get_entropies(self, epoch):
        # print "requesting prediction for time %d" %epoch
        edges_ids=[]
        dur=[]
        
        eids = [x['edge_id'] for x in self.models]
        mods = [x['model_id'] for x in self.models]
        ords = [x['order'] for x in self.models]

        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'forecastEntropy'
        fremgoal.ids = mods
        fremgoal.times.append(epoch)
        #print i["order"]
        fremgoal.order = -1
        fremgoal.orders = ords#i["order"]
        
        self.FremenClient.send_goal(fremgoal)
        self.FremenClient.wait_for_result()
        ps = self.FremenClient.get_result()

        print ps

        prob = list(ps.entropies)

      
        for i in self.unknowns:
            edges_ids.append(i["edge_id"])
            prob.append(1.0)                # a priori probabilities (no stats)


        return edges_ids, prob, dur



    def edge_entropies_cb(self, req):
        # print req
        return self.get_entropies(req.epoch.secs)


    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self.map_received = True


    def get_list_of_edges(self):
        rospy.loginfo("Querying for list of edges")
        for i in self.lnodes.nodes :
            for j in i.edges:
                if j.edge_id not in self.edgid :
                    self.edgid.append(j.edge_id)
                    val={}
                    val["edge_id"]=j.edge_id
                    val["model_id"]=self.lnodes.name+'__'+j.edge_id
                    val["time_model_id"]=self.lnodes.name+'__'+j.edge_id+'_time'
                    val["ori"]=i.name
                    val["dest"]=j.node
                    ddn=get_node(self.lnodes, j.node)
                    val["dist"]= get_distance_to_node(i,ddn)                    
#                    if j.top_vel>= 0.1:
#                        val["dist"]= get_distance_to_node(i,ddn)/j.top_vel
#                    else :
#                        val["dist"]= get_distance_to_node(i,ddn)/0.1
                    self.eids.append(val)
        fdbmsg = 'Done. %d edges found' %len(self.edgid)
        rospy.loginfo(fdbmsg)


    """
     BuildCallBack
     
     This Functions is called when the Action Server is called to build the models again
    """
    def BuildCallback(self, goal):
        self.cancelled = False

        # print goal

        # set epoch ranges based on goal
        if goal.start_range.secs > 0:
            self.range[0] = goal.start_range.secs
        if goal.end_range.secs > 0:
            self.range[1] = goal.end_range.secs

        rospy.loginfo('Building model for epoch range: %s' % self.range)

        start_time = time.time()        
        #self.get_list_of_edges()
        #elapsed_time = time.time() - start_time
        #self._feedback.result = "%d edges found in %.3f seconds \nGathering stats ..." %(len(self.eids),elapsed_time)
        #self._as.publish_feedback(self._feedback)
        #self.gather_stats()        
        self.create_temporal_models()
        elapsed_time = time.time() - start_time
        self._feedback.result = "Finished after %.3f seconds" %elapsed_time 
        self._as.publish_feedback(self._feedback)       #Publish Feedback

        #rospy.loginfo("Finished after %.3f sechttp://9gag.com/gag/aGR3z60onds" %elapsed_time)
        
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

        to_add=[]
        for i in self.eids:            

            query = {"topological_map": self.lnodes.name, "edge_id":i["edge_id"]}                
            query_meta={}            
            
            if len(self.range) == 2:
                
                if self.range[1] < 1:
                    upperlim = rospy.Time.now().secs
                else:
                    upperlim = self.range[1]

                query_meta["epoch"] = {"$gte": self.range[0], "$lt" : upperlim}    

            #print query
            #print query_meta
            
            available = msg_store.query(NavStatistics._type, query, query_meta)
            # print len(available)
            edge_mod={}
            edge_mod["model_id"]= i["model_id"]#self.lnodes.name+'__'+i["edge_id"]
            edge_mod["time_model_id"]=i["time_model_id"]
            edge_mod["dist"]= i["dist"]#self.lnodes.name+'__'+i["edge_id"]
            edge_mod["models"]=[]
            edge_mod["order"]=-1
            edge_mod["t_order"]=-1
            edge_mod["edge_id"]=i["edge_id"]
            
            for j in available:                
                val = {}
                if j[0].status != 'fatal':
                    val["st"] = 1
                    val["speed"] = i["dist"]/j[0].operation_time
                    if val["speed"]>1:
                        val["speed"]=1.0
                else:
                    val["st"] = 0
                    val["speed"] = 0.0
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
            # print "-------CREATING MODEL----------"
            
            mid = i["model_id"]
            tmid = i["time_model_id"]
          
            print "Creating models:"
            print mid, tmid
            #print i["dist"]
            stimes=[]
            times=[]
            states=[]
            speeds=[]
            #print "adding %d meassurements" %len(i["models"])
            for j in i["models"]:
                times.append(j["epoch"])
                states.append(j["st"])
                #print j
                if j["speed"]>0:
                    speeds.append(j["speed"])
                    stimes.append(j["epoch"])                
            i["order"] = self.add_and_eval_models(mid,times,states)
            
            print "###########################################"
            print "Generating speed models: ", tmid
            print "epochs (%d): " %len(stimes)
            print stimes
            print "speeds (%d): " %len(speeds)
            print speeds
            
            i["t_order"] = self.add_and_eval_value_models(tmid,stimes,speeds)
#            i["t_order"] = self.add_and_eval_models(tmid,stimes,speeds)
            print "Done Model Order %d" %i["t_order"]
            
            #print times
            #print states


    def add_and_eval_models(self, model_id, epochs, states):
            fremgoal = fremenserver.msg.FremenGoal()
            fremgoal.operation = 'add'
            fremgoal.id = model_id
            fremgoal.times = epochs
            fremgoal.states = states
            
            # Sends the goal to the action server.
            self.FremenClient.send_goal(fremgoal)
            
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
            
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()
            #print ps
            
            # print "--- EVALUATE ---"
            frevgoal = fremenserver.msg.FremenGoal()
            frevgoal.operation = 'evaluate'
            frevgoal.id = model_id
            frevgoal.times = epochs
            frevgoal.states = states
            frevgoal.order = 5
            
            # Sends the goal to the action server.
            self.FremenClient.send_goal(frevgoal)
            
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
            
            # Prints out the result of executing the action
            pse = self.FremenClient.get_result()  # A FibonacciResult
            # print pse.errors
            # print "chosen order %d" %pse.errors.index(min(pse.errors))
            return pse.errors.index(min(pse.errors))


    def add_and_eval_value_models(self, model_id, epochs, states):
            fremgoal = fremenserver.msg.FremenGoal()
            fremgoal.operation = 'addvalues'
            fremgoal.id = model_id
            fremgoal.times = epochs
            #fremgoal.states = states
            fremgoal.values = states
            
            # Sends the goal to the action server.
            self.FremenClient.send_goal(fremgoal)
            
            print "Sending data to fremenserver"
            
            
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
            
            print "fremenserver done"
            
            # Prints out the result of executing the action
            ps = self.FremenClient.get_result()
            print "fremenserver result:"
            print ps
            
            # print "--- EVALUATE ---"
            frevgoal = fremenserver.msg.FremenGoal()
            frevgoal.operation = 'evaluate'
            frevgoal.id = model_id
            frevgoal.times = epochs
            frevgoal.states = states
            frevgoal.order = 5
            
            # Sends the goal to the action server.
            self.FremenClient.send_goal(frevgoal)
            
            # Waits for the server to finish performing the action.
            self.FremenClient.wait_for_result()
            
            # Prints out the result of executing the action
            pse = self.FremenClient.get_result()  # A FibonacciResult
            # print pse.errors
            # print "chosen order %d" %pse.errors.index(min(pse.errors))
            return pse.errors.index(min(pse.errors))
        

if __name__ == '__main__':
    rospy.init_node('topological_prediction')
    epochs=[]
    #if len(sys.argv) < 2:
    if '-h' in sys.argv or '--help' in sys.argv:
        usage()
        sys.exit(1)
    else:
        if '-range' in sys.argv:
            ind = sys.argv.index('-range')
            epochs.append(int(sys.argv[ind+1]))
            epochs.append(int(sys.argv[ind+2]))
            print epochs
        else:
            print "gathering all the stats"        
            epochs=[0, rospy.get_rostime().to_sec()]

    server = TopologicalNavPred(epochs)