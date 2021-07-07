#!/usr/bin/env python
"""
Created on Thu Apr 29 10:04:30 2021

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import sys, os, traceback
import rospy

from topological_navigation.manager import map_manager


def convert_tmaps(root_dir, ext_old, ext_new):
        
    tmap_files = []
    for root, dirs, files in os.walk(root_dir):
        for _file in files:
            if _file.endswith(ext_old):
                 tmap_files.append(os.path.join(root, _file))
    
    mm = map_manager()
    for f in tmap_files:
        print "\n"
        rospy.loginfo("CONVERTING " + f)
        
        try:
            mm.init_map(f, load_from_file=True)
            mm.manager2.write_topological_map(os.path.splitext(f)[0] + ext_new)
        except Exception:
            rospy.logerr("Unable to convert {}\n{}".format(f, traceback.format_exc()))
#########################################################################################################
             
             
#########################################################################################################    
if __name__ == '__main__' :
    
    if "-h" in sys.argv or "--help" in sys.argv or len(sys.argv) < 4:
        print "usage is map_converter.py root_directory old_topomap_ext new_topomap_ext"
        sys.exit(1)
    else:
        root_dir = sys.argv[1]
        ext_old = sys.argv[2]
        ext_new = sys.argv[3]

    print("Converting topological maps with ext {} found in {} to new format with ext {}".format(ext_old, root_dir, ext_new))

    rospy.init_node("topological_map_converter")
    convert_tmaps(root_dir, ext_old, ext_new)
#########################################################################################################