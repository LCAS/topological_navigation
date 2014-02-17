#!/usr/bin/env python
import math

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
        self.px=0.0
        self.py=0.0
        self.influence_radius=1.5

    def _insert_waypoint(self, waypoint):
        self.waypoint=waypoint
        self._get_coords()

    def _get_coords(self):
        self.px=float(self.waypoint[0])
        self.py=float(self.waypoint[1])

    def _get_distance(self, cx, cy):
        dist=math.hypot((cx-self.px),(cy-self.py))
        return dist

    def _insert_edges(self, edges):
        self.edges=edges

    def _insert_vertices(self, vertices):
        self.vertices=vertices
        a=0
        for i in self.vertices :
            dist=math.hypot(i[0],i[1])
            if dist > a :
                a=dist
        self.influence_radius=a
        
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