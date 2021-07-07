#!/usr/bin/env python
"""
Created on Tue Jul  6 13:07:18 2021

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import sys, rospy, yaml
from topological_navigation.manager2 import map_manager_2


def load_yaml(filename):
    with open(filename,'r') as f:
        return yaml.load(f)
    
    
def edge_groups_to_tmap2(f_tmap2, groups, group_names):
    
    mm2 = map_manager_2()
    mm2.init_map(filename=f_tmap2)

    for name in group_names:
        rospy.loginfo("Adding parameters for edge group {}".format(name))
        group = groups["edge_nav_reconfig_groups"][name]
        
        for edge_id in group["edges"]:
            for param in group["parameters"]:
                if not isinstance(param["value"], str):
                    value = str(param["value"])
                    value_is_string = False
                else:
                    value_is_string = True
                    
                mm2.add_param_to_edge_config(edge_id, param["ns"], param["name"], value, value_is_string, write_map=False)
    
    mm2.write_topological_map(f_tmap2)            
#########################################################################################################


#########################################################################################################    
if __name__ == '__main__' :
    
    if "-h" in sys.argv or "--help" in sys.argv or len(sys.argv) < 4:
        print "usage is edge_reconf_groups_to_tmap2.py tmap.tmap2 edge_reconf_groups.yaml ['group_1', 'group_2',...,'group_n']"
        sys.exit(1)
    else:
        print sys.argv
        f_tmap2 = sys.argv[1]
        f_groups = sys.argv[2]
        group_names = sys.argv[3].strip('[]').split(',')
        
    print("Populating topological map 2 '{}' with parameters from edge reconfigure groups '{}'".format(f_tmap2, f_groups))
    
    rospy.init_node("edge_reconf_groups_to_tmap2")
    
    groups = load_yaml(f_groups)
    edge_groups_to_tmap2(f_tmap2, groups, group_names)
#########################################################################################################