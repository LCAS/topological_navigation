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
import strands_navigation_msgs.msg


def get_edge_id(source, target, eids):
    test=0
    eid = '%s_%s' %(source, target)
    while eid in eids:
        eid = '%s_%s_%3d' %(source, target, test)
        test += 1
    return eid


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


def update_edge(edge, nna, nma, eids):
    e = Edge()
    e.top_vel =0.55
    e.map_2d = nma
    lsl = e.__slots__
    for j in lsl:
        if edge.has_key(j):
            setattr(e, j, edge[j])
    if e.edge_id == '':
        eid = get_edge_id(nna, e.node, eids)
        e.edge_id = eid
    eids.append(eid)
    return e, eids

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

def check_for_update(b, d, client):
    to_update=[]
    db=client.message_store
    collection=db["topological_maps"]
    available = collection.find().distinct("_meta.pointset")

    #for every point_set in the db
    for pointset in available:
        #get one message
        search = {"_meta.pointset": pointset, "name" : 'ChargingPoint'}
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
    return to_update#, available

def update_maps(to_update, client):
    db=client.message_store
    collection=db["topological_maps"]
#    available = collection.find().distinct("_meta.pointset")

    msg_store = MessageStoreProxy(collection='topological_maps')

    del_ids = []
    for pointset in to_update:
        #pointsetb='%s_b'%pointset
        #print pointsetb
        search = {"_meta.pointset": pointset}
        av =collection.find(search)
        #lnodes=[]
        eids=[] #list of known edge id's
        for a in av:
            #print a
            bc = update_node(a, pointset)
    
            nna = a['name']
            nma = a['map']
            vt = a['verts']
            if a['edges']:
                es = a['edges']
                for i in es:
                    ed, eids = update_edge(i, nna, nma, eids)
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


def check_sanity(client):
    msg_store = MessageStoreProxy(collection='topological_maps')
    db=client.message_store
    collection=db["topological_maps"]
    available = collection.find().distinct("_meta.pointset")
    print available
    
    for point_set in available:
        #get one message
        #search = {"_meta.pointset": pointset}
        query_meta = {}
        query_meta["pointset"] = point_set
              
        message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
        #points = strands_navigation_msgs.msg.TopologicalMap()
        #points.name = point_set
        #points.map = point_set
        #points.pointset = point_set
        #string last_updated
        eids = []
        for i in message_list:
            update = False
            print i[0].pointset, i[0].name, i[1]['_id']
            if i[0].edges:
                for j in i[0].edges :
                    if j.edge_id == '':
                        update = True
                        print 'needs edge id'
                        j.edge_id =  get_edge_id(i[0].name, j.node, eids)
                        print 'new edge_id %s' %j.edge_id
                    eids.append(j.edge_id)
                    if j.top_vel <= 0.1 :
                        update = True
                        print 'needs top vel'
                        j.top_vel = 0.55
                    if j.map_2d == '':
                        update = True
                        print 'needs map_2d'
                        j.map_2d = i[0].map
            if update:
                msg_store.update_id(i[1]['_id'], i[0], i[1], upsert = False)                

def add_localise_by_topic(tmap, node, json_str):
    #print req
    #data = json.loads(req.content)
    #print data

    msg_store = MessageStoreProxy(collection='topological_maps')
    query = {"name" : node, "pointset": tmap}
    query_meta = {}
    #query_meta["pointset"] = tmap
    #query_meta["map"] = self.nodes.map

    #print query, query_meta
    available = msg_store.query(strands_navigation_msgs.msg.TopologicalNode._type, query, query_meta)
    #print len(available)
    if len(available) != 1:
         #succeded = False
         print 'there are no nodes or more than 1 with that name'
    else:
        #succeded = True
        for i in available:
            if not i[0].localise_by_topic:
                msgid= i[1]['_id']
                i[0].localise_by_topic=json_str
                #print i[0]
                print "Updating %s--%s" %(i[0].pointset, i[0].name)
                msg_store.update_id(msgid, i[0], i[1], upsert = False)


if __name__ == '__main__':

    rospy.init_node('strands_navigation_migration')
    host = rospy.get_param("mongodb_host")
    port = rospy.get_param("mongodb_port")
    client = pymongo.MongoClient(host, port)

    to_pop=['_id','_meta']

    b = TopologicalNode().__slots__
    d = Edge().__slots__    
    
    print '========= Current Topological NODE definition Slots ==========='
    print b
    print '========= Current Topological EDGE definition Slots ==========='
    print d


    to_update = check_for_update(b, d, client)

    print '========= The following maps need to be updated ==========='
    print to_update
    
    update_maps(to_update, client)

    check_sanity(client)    
    #print available
    
    db=client.message_store
    collection=db["topological_maps"]
    available = collection.find().distinct("_meta.pointset")
    print available    
    
    for i in available:
        add_localise_by_topic(i, 'ChargingPoint', "{\"topic\":\"battery_state\",\"field\":\"charging\", \"val\": true}")

    