#!/usr/bin/env python

import yaml
import sys

from strands_navigation_msgs.msg import TopologicalNode
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy


if __name__ == '__main__':
    if len(sys.argv) < 2 :
        print "usage: insert_map input_file.txt"
	sys.exit(2)

    filename=str(sys.argv[1])

    msg_store = MessageStoreProxy(collection='topological_maps')

    print "loading %s"%filename
    yaml_data=open(filename, "r")
    
    data = yaml.load(yaml_data)
    
    print "printing vlaa"

    
    for i in data:
        meta = i['meta']
        print i['node']
        msgv = dc_util.dictionary_to_message(i['node'], TopologicalNode)
        #print msgv, meta
        msg_store.insert(msgv,meta)
        #mongodb_store.util.store_message(points_db,p,val)























