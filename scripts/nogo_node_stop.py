#!/usr/bin/env python

import rospy
from threading import Timer
import rosservice
import actionlib
#import rosgraph.masterapi

from std_msgs.msg import String
import strands_navigation_msgs.srv
from strands_navigation_msgs.msg import TopologicalMap
import strands_emails.msg



class NoGoServer(object):

    def __init__(self, name) :
       
        rospy.on_shutdown(self._on_node_shutdown)
        self.current_node = "Unknown"
        self.nogo_pre_active = False
        self.nogo_active = False
        self.nogos=[]
        self.stop_services=["/enable_motors", "/emergency_stop", "/task_executor/set_execution_status"]
        self.activate_services=["/enable_motors", "/reset_motorstop", "/task_executor/set_execution_status"]


        

        #Waiting for Topological Map        
        self._map_received=False
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while not self._map_received :
            pass
        rospy.loginfo(" ...done")


        self.update_service_list()
        print self.av_stop_services, self.av_activate_services
        
    
        #Subscribing to Localisation Topics
        rospy.loginfo("Subscribing to Localisation Topics")
        rospy.Subscriber('/current_node', String, self.currentNodeCallback)
        rospy.loginfo(" ...done")

        self.nogos = self.get_no_go_nodes()
        print self.nogos
        self._killall_timers=False
        t = Timer(1.0, self.timer_callback)
        t.start()
        rospy.loginfo("All Done ...")
        rospy.spin()



    def update_service_list(self):
        self.av_stop_services=[]
        self.av_activate_services=[]        
        #print rosservice.get_service_list()
        self.service_names = rosservice.get_service_list()#[x[0] for x in rosgraph.masterapi.Master('/no_go_nodes_stop').getSystemState()[2]]
        #print service_names
        for i in self.stop_services:
            if i in self.service_names:
                self.av_stop_services.append(i)
        
        for i in self.activate_services :
            if i in self.service_names :
                self.av_activate_services.append(i)



    """
     Get No_Go_Nodes
     
     This function gets the list of No go nodes
    """
    def get_no_go_nodes(self):
        rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
        try:
            get_nogos = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = get_nogos('no_go')
            #print resp1
            return resp1.nodes
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return []



    """
     Update Map CallBack
     
     This Function updates the Topological Map everytime it is called
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self._map_received=True
        
        #print "NO GO NODES"

        #print self.nogos


    """
     Current Node CallBack
     
    """
    def currentNodeCallback(self, msg):
        self.current_node = msg.data
        if self.current_node in self.nogos:
            self.nogo_active=True
        else:
            self.nogo_active=False
 
 
    def timer_callback(self):
        if self.nogo_active:
            self.set_emergency_stop()
            if not self.nogo_pre_active :
                self.update_service_list()
                self.set_free_run(True)
                self.start_stop_scheduler(False)
                self.send_email()
            self.nogo_pre_active = True
        else:
            if self.nogo_pre_active :
                self.update_service_list()
                self.release_emergency_stop()
                self.set_free_run(False)
                self.start_stop_scheduler(True)            
            self.nogo_pre_active = False
            
        if not self._killall_timers :
            t = Timer(1.0, self.timer_callback)
            t.start()


    def set_free_run(self, val):
        if "/enable_motors" in self.av_stop_services:
            try:
                rospy.wait_for_service('/enable_motors', timeout=0.5)
                sfrun = rospy.ServiceProxy('/enable_motors', rosservice.get_service_class_by_name('/enable_motors'))
                sfrun(val)
                print "Free run %s" %val
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Scheduler Free Run service')

        
    def start_stop_scheduler(self, val):
        if "/task_executor/set_execution_status" in self.av_stop_services:
            try:
                rospy.wait_for_service('/task_executor/set_execution_status', timeout=0.5)
                schstop = rospy.ServiceProxy('/task_executor/set_execution_status', rosservice.get_service_class_by_name('/task_executor/set_execution_status'))
                schstop(val)
                print "Schedule Execute %s" %val
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Scheduler Start/Stop service')            

    def set_emergency_stop(self):
        if '/emergency_stop' in self.av_stop_services:
            try:
                rospy.wait_for_service('/emergency_stop', timeout=0.5)
                stop = rospy.ServiceProxy('/emergency_stop', rosservice.get_service_class_by_name('/emergency_stop'))
                stop()
                rospy.loginfo("NO GO node stop")
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Emergency Stop service')
    
    def release_emergency_stop(self):
        if '/reset_motorstop' in self.av_activate_services :
            try:
                rospy.wait_for_service('/reset_motorstop', timeout=0.5)
                reset = rospy.ServiceProxy('/reset_motorstop', rosservice.get_service_class_by_name('/reset_motorstop'))
                reset()
                print "reset"
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Release Emergency Stop service')

    def send_email(self):
        client = actionlib.SimpleActionClient('strands_emails', strands_emails.msg.SendEmailAction)
        client.wait_for_server()
        goal = strands_emails.msg.SendEmailGoal()
        
        rospy.loginfo(" ... Init done")
        goal.text = "Help!!!!!\n I just reached a no go area, please come rescue me!!!! \nthe recommended action:\npush me away from the Nogo area towards a safer zone NOT Down the stairs"
        goal.to_address = 'pulidofentanes@gmail.com, jpulidofentanes@lincoln.ac.uk'
        goal.subject = 'Robot needs help'
        
        # Sends the goal to the action server.
        client.send_goal(goal)
        
        # Waits for the server to finish performing the action.
        print "sedning email"
        #client.wait_for_result()
        
        # Prints out the result of executing the action
        return client.get_result()  # A FibonacciResult


    def _on_node_shutdown(self):
        self._killall_timers=True


if __name__ == '__main__':
    rospy.init_node('no_go_nodes_stop')
    server = NoGoServer(rospy.get_name())
