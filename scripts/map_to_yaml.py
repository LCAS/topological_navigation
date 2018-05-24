#!/usr/bin/env python

import sys
import rospy
import json
import sys
import yaml
#import pickle

from topological_navigation.topological_node import *
from actionlib_msgs.msg import *

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from strands_navigation_msgs.msg import TopologicalNode

from mongodb_store.message_store import MessageStoreProxy
import topological_navigation.msg




class MapExport(object):

    def __init__(self, dataset_name, filename) :
        #print "loading file from map %s" %filename
        self.lnodes = self.loadMap(dataset_name, filename)


    def loadMap(self, point_set, filename):

        point_set=str(sys.argv[1])
        #map_name=str(sys.argv[3])

        msg_store = MessageStoreProxy(collection='topological_maps')

        query_meta = {}
        query_meta["pointset"] = point_set

        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0

        print available

        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")

        else :
            query_meta = {}
            query_meta["pointset"] = point_set

            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)

            #node=TopologicalNode()

            top_map=[]
            for i in message_list:
                nodeinf = {}
                nodeinf["node"] = yaml.load(str(i[0]))
                if nodeinf["node"]["localise_by_topic"]:
                    nodeinf["node"]["localise_by_topic"] = json.dumps(nodeinf["node"]["localise_by_topic"])
                nodeinf["meta"] = i[1] #str(bson.json_util.dumps(i[1], indent=1))
                nodeinf["meta"].pop("last_updated_by", None)
                nodeinf["meta"].pop('inserted_at', None)
                nodeinf["meta"].pop('last_updated_at', None)
                nodeinf["meta"].pop('stored_type', None)
                nodeinf["meta"].pop('stored_class', None)
                nodeinf["meta"].pop('inserted_by', None)
                nodeinf["meta"].pop('_id', None)
                top_map.append(nodeinf)
                #val = bson.json_util.dumps(nodeinf["meta"], indent=1)



            top_map.sort(key=lambda x: x['node']['name'])
            yml = yaml.safe_dump(top_map, default_flow_style=False)
            #print yml
            #print s_output

            fh = open(filename, "w")
            #s_output = str(bson.json_util.dumps(nodeinf, indent=1))
            s_output = str(yml)
            #print s_output
            fh.write(s_output)
            fh.close

#            fh = open(filename, "w")
#            #s_output = str(bson.json_util.dumps(nodeinf, indent=1))
#            s_output = str(bson.json_util.dumps(top_map, indent=1, sort_keys=True) )
#            print s_output
#            fh.write(s_output)
#            fh.close


if __name__ == '__main__':
    if len(sys.argv) < 3 :
        print "usage: map_to_yaml dataset_name output_file.yaml"
        sys.exit(2)

    dataset_name=str(sys.argv[1])
    filename=str(sys.argv[2])
    rospy.init_node('topological_map_exporter')
    server = MapExport(dataset_name, filename)
