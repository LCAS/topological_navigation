#!/usr/bin/env python

import rospy
import rosservice
import actionlib
#import rosgraph.masterapi


from std_msgs.msg import String
import std_srvs.srv
import strands_navigation_msgs.srv
from strands_navigation_msgs.msg import TopologicalMap
import strands_emails.msg
import topological_navigation.msg


class SafetyServer(object):

    def __init__(self, name) :
        #rospy.on_shutdown(self._on_node_shutdown)
        self.closest_node = "Unknown"
        self.stop_services=["/enable_motors", "/emergency_stop", "/task_executor/set_execution_status"]
        self.activate_services=["/enable_motors", "/reset_motorstop", "/task_executor/set_execution_status"]  
        self.info=''
        

        #Waiting for Topological Map        
        self._map_received=False
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while not self._map_received :
            pass
        rospy.loginfo(" ...done")

        self.update_service_list()
        
        
        #Subscribing to Localisation Topics
        rospy.loginfo("Subscribing to Localisation Topics")
        rospy.Subscriber('/closest_node', String, self.cNodeCallback)
        rospy.loginfo(" ...done")

        #This service returns any given map
        self.get_map_srv=rospy.Service('/go_to_safety_point', std_srvs.srv.Empty, self.goto_safety_cb)


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
     Get Safety_Nodes
     
     This function gets the list of No go nodes
    """
    def goto_safety_cb(self, req):
        self.update_service_list()
        nodes=self.get_safety_nodes()
        self.info=''
        if nodes:
            print nodes[0]
            rospy.loginfo("Pause task executor")
            self.info= self.info + ' Pause task executor\n'
            self.start_stop_scheduler(False)
            if not self.go_to_node(nodes[0]):
                cinfo = "Navigation Failed stuck somewhere close to %s" %self.closest_node
                rospy.loginfo(cinfo)
                self.info= self.info + ' ' +cinfo + '\n'
            else:
                cinfo = "At %s" %self.closest_node
                rospy.loginfo(cinfo)
                self.info= self.info + ' ' +cinfo + '\n'
            self.info= self.info + ' Set Emergency Stop\n'
            rospy.loginfo("Set Emergency Stop")
            self.set_emergency_stop()
            self.info= self.info + ' Set Free Run\n'
            rospy.loginfo("Set Free Run")
            self.set_free_run(True)
        else :
            self.info= self.info + ' Can\'t Find any Safety Points no action taken\n'
            rospy.logerr("Can't Find any Safety Points no action taken")
        self.send_email()
        return []




    def get_safety_nodes(self):
        try:
            rospy.wait_for_service('/topological_localisation/get_nodes_with_tag')
            get_nodes = rospy.ServiceProxy('/topological_localisation/get_nodes_with_tag', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = get_nodes('Safety_Point')
            #print resp1
            return resp1.nodes
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return []



    def go_to_node(self, node):
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
           
        self.client.wait_for_server()
        doing = "Requesting Navigation to %s" %node
        self.info= self.info + ' ' +doing+'\n'
        rospy.loginfo(doing)
               
        navgoal = topological_navigation.msg.GotoNodeGoal()       
        #print "Requesting Navigation to %s" %node
        navgoal.target = node
        
        self.client.send_goal(navgoal)
        self.client.wait_for_result()
        
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        return ps.success


    """
     Update Map CallBack
     
     This Function updates the Topological Map everytime it is called
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self._map_received=True


    """
     Current Node CallBack
     
    """
    def cNodeCallback(self, msg):
        self.closest_node = msg.data
 


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
                print "stop"
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
        
        goal.text = "Hello,\nMy safety behaviour has been triggered I have taken the following actions:\n%s\nI recommend calling an expert to resume Normal operation" %self.info
        goal.to_address = 'pulidofentanes@gmail.com, jpulidofentanes@lincoln.ac.uk'
        goal.subject = 'Robot Safety Behaviour Triggered'
        
        client.send_goal(goal)
        print "sending email"
        
        # Prints out the result of executing the action
        #return client.get_result()  # A FibonacciResult



if __name__ == '__main__':
    rospy.init_node('go_to_safety')
    server = SafetyServer(rospy.get_name())
