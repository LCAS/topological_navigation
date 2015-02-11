#!/usr/bin/env python

import json
import yaml
import sys

from strands_navigation_msgs.msg import TopologicalNode
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy


if __name__ == '__main__':
    if len(sys.argv) < 1 :
        print "usage: insert_map input_file.txt"
	sys.exit(2)

    filename=str(sys.argv[1])
    #dataset_name=str(sys.argv[2])
    #map_name=str(sys.argv[3])

    msg_store = MessageStoreProxy(collection='topological_maps')

    json_data=open(filename, 'rb').read()
    
    data = json.loads(json_data)
    
    for i in data:
        meta = i['meta']
        msgv = dc_util.dictionary_to_message(i['node'], TopologicalNode)
        msg_store.insert(msgv,meta)
        #mongodb_store.util.store_message(points_db,p,val)























