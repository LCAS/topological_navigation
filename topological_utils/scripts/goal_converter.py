#!/usr/bin/env python
"""
Created on Thu Apr 29 10:04:30 2021

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import sys, os, traceback
import rospy, yaml, json

from topological_navigation.manager2 import map_manager_2

def load_data_from_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.load(f)


def convert_goal(root_dir, action, action_type, goal_path):
    
    goal = load_data_from_yaml(goal_path)
        
    tmap_files = []
    for root, dirs, files in os.walk(root_dir):
        for _file in files:
            if _file.endswith(".tmap2"):
                 tmap_files.append(os.path.join(root, _file))
    
    mm = map_manager_2(advertise_srvs=False)
    for f in tmap_files:
        print "\n"
        rospy.loginfo("CHANGING THE GOAL IN " + f)
        
        try:
            mm.init_map(filename=f)
            mm.update_action(action, action_type, json.dumps(goal), update=False, write_map=False)
            mm.write_topological_map(f)
        except Exception:
            rospy.logerr("Unable to convert goal in {}\n{}".format(f, traceback.format_exc()))
            sys.exit(1)
            
        rospy.sleep(1.0)
#########################################################################################################
             
             
#########################################################################################################    
if __name__ == '__main__' :
    
    if "-h" in sys.argv or "--help" in sys.argv or len(sys.argv) < 5:
        print "usage is goal_converter.py root_directory action action_type goal"
        sys.exit(1)
    else:
        root_dir = sys.argv[1]
        action = sys.argv[2]
        action_type = sys.argv[3]
        goal = sys.argv[4]

    print("Converting the goal type of {} action to {} for topological maps found in {}".format(action, action_type, root_dir))

    rospy.init_node("topological_goal_converter")
    convert_goal(root_dir, action, action_type, goal)
#########################################################################################################