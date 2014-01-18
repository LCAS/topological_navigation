#!/usr/bin/env python


def findInList(name,List):
    found = name in List
    if found :
        pos = List.index(name)
    else :
        pos = -1
    return pos


def update_to_expand(to_expand, new_nodes, maptree, father) :
    for i in new_nodes :
        found_el=False
        for j in to_expand :
            if i == j.name :
                found_el=True
        if not found_el :
            for k in maptree :
                if i == k.name :
                    new_el=k
                    new_el._set_Father(father)
                    to_expand.append(new_el)
    return to_expand

def get_node(name, maptree) :
    for i in maptree :
        if i.name == name :
            return i
    return None


class topological_node(object):
    def __init__(self, name):
        self.name = name
        self.expanded=False
        self.father='none'

    def _insert_waypoint(self, waypoint):
        self.waypoint=waypoint

    def _insert_edges(self, edges):
        self.edges=edges
        
    def _get_Children(self) :
        self.expanded=True
        a=['none']
        for k in self.edges :
            a.append(k['node'])
        a.pop(0)
        return a
        
    def _get_action(self, node) :
        for k in self.edges :
            if k['node'] == node :
                return k['action']
    
    def _set_Father(self,father) :
        self.father=father