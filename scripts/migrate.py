#!/usr/bin/env python
import sys
import rospy
import pymongo
import json
import sys
import math


from strands_navigation_msgs.msg import TopologicalNode, Edge, Vertex
from geometry_msgs.msg import Pose
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy



def update_node(node, pointset):
    to_pop=['edges','verts','pose']
    n = TopologicalNode()

    n.yaw_goal_tolerance = 0.1
    n.xy_goal_tolerance = 0.3

    lsl = n.__slots__
    for j in lsl:
        if node.has_key(j) and not j in to_pop:
            setattr(n, j, node[j])

    ppos = node['pose']

    p = dc_util.dictionary_to_message(ppos, Pose)
    #print p
    #n.pose.append(p)
    n.pose = p
    n.pointset = pointset
    return n


def update_edge(edge, nna, nma):
    e = Edge()
    #e.action = ''
    e.top_vel =0.55
    e.map_2d = nma
#    e.use_default_recovery = True
#    e.use_default_nav_recovery = True
#    e.use_default_helpers = True
    lsl = e.__slots__
    for j in lsl:
        if edge.has_key(j):
            setattr(e, j, edge[j])
    e.edge_id = "%s_%s"%(nna,e.node)
    return e

def update_vert(vert):
    v = Vertex()

    lsl = v.__slots__
    for j in lsl:
        if vert.has_key(j):
            setattr(v, j, vert[j])

    return v

def update_meta(meta, pointset):
    cm=meta
    to_pop=['inserted_at','stored_type','stored_class','inserted_by']
    
    lsl = meta.keys()
    
    for j in lsl:
        if j in to_pop:
            cm.pop(j)
    
    cm['pointset']=pointset
    return cm

if __name__ == '__main__':

    rospy.init_node('strands_navigation_migration')
    host = rospy.get_param("mongodb_host")
    port = rospy.get_param("mongodb_port")
    client = pymongo.MongoClient(host, port)

    to_pop=['_id','_meta']
    to_update=[]

    b = TopologicalNode().__slots__
    d = Edge().__slots__    
    
    print '========= Current Topological node definition Slots ==========='
    print b
    

    #pointset='robolab'
    #pointset='robolab_b'

    db=client.message_store
    collection=db["topological_maps"]
    available = collection.find().distinct("_meta.pointset")

    #for every point_set in the db
    for pointset in available:
        #get one message
        search = {"_meta.pointset": pointset}
        aa =collection.find_one(search)
        a = aa.keys()
        #pop out meta form msg store from the dict keys
        for i in to_pop:
            if i in a:
                a.pop(a.index(i))
        #count the difference
        c = len(list(set(b).difference(set(a))))
        if c > 0:
            #add to update list
            to_update.append(pointset)
        else:
#            print "No differences at node level testing edges"
#            print 'Edge comparison for pointset %s' %pointset
            e = aa["edges"][0].keys()
            edef =len(list(set(d).difference(set(e))))
#            print edef
            if edef > 0:
                to_update.append(pointset)

    print '========= The following maps need to be updated ==========='
    print to_update

    msg_store = MessageStoreProxy(collection='topological_maps')

    del_ids = []
    for pointset in to_update:
        #pointsetb='%s_b'%pointset
        #print pointsetb
        search = {"_meta.pointset": pointset}
        av =collection.find(search)
        lnodes=[]
        for a in av:
            #print a
            bc = update_node(a, pointset)
    
            nna = a['name']
            nma = a['map']
            es = a['edges']
            vt = a['verts']
            for i in es:
                ed = update_edge(i, nna, nma)
                bc.edges.append(ed)
                
            for i in vt:
                v = update_vert(i)
                bc.verts.append(v)
            
            meta = update_meta(a['_meta'], pointset)
            
            #print bc
            #print meta
            del_ids.append(a['_id'])
            #lnodes.append(bc)
            msg_store.insert(bc,meta)
        #print lnodes
    
    print "Deleting updated nodes"
    for i in del_ids:
        msg_store.delete(str(i))
    print "done"
    