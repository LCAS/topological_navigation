#!/usr/bin/env python
"""
Created on Thu Apr 29 10:04:30 2021

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import sys, os
import rospy


class MapConverter(object):
    
    def __init__(self, root_dir, ext_old, ext_new):
        
        tmap_files = []
        for root, dirs, files in os.walk(root_dir):
            for _file in files:
                if _file.endswith(ext_old):
                     tmap_files.append(os.path.join(root, _file))
                     
        for f in tmap_files:
            print f
#########################################################################################################
             
             
#########################################################################################################    
if __name__ == '__main__' :
    
    if "-h" in sys.argv or "--help" in sys.argv or len(sys.argv) < 4:
        print "usage is map_converter.py, root_directory, old_topomap_ext, new_topomap_ext"
        sys.exit(1)
    else:
        root_dir = sys.argv[1]
        ext_old = sys.argv[2]
        ext_new = sys.argv[3]

    print "Converting all topological maps with ext {} found in {} to new format with ext {}".format(ext_old, root_dir, ext_new)

    rospy.init_node("topological_map_converter")
    mc = MapConverter(root_dir, ext_old, ext_new)
    rospy.spin()
#########################################################################################################