#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from strands_navigation_msgs.msg import TopologicalNode, TopologicalMap, Edge
from geometry_msgs.msg import Pose, Point


def create_cross_map(width, height, nodeSeparation):
    """ Creates a vertical cross with 5 nodes in each line, with the middle at 0,0 """
    xOrigin = 0
    yOrigin = 0
    nodeSeparation = 10

    nodes = dict()

    originNode = TopologicalNode(name='origin', pose=Pose(position=Point(xOrigin, yOrigin, 0)))
    nodes['origin'] = originNode



    # horizontal nodes
    prevNodeName = ''
    for x in range(-(width/2),(width/2)+1):
    	if x != 0:
            nodeName = 'h_%s'%x
            node = TopologicalNode(name=nodeName, pose=Pose(position=Point(x * nodeSeparation, yOrigin, 0)))
            nodes[nodeName] = node
        else:
            nodeName = 'origin'
            node = originNode

        if prevNodeName:
            # connect up this to previous
            e = Edge()
            e.node = prevNodeName
            e.action = 'move_base'
            node.edges.append(e)
            # previous to this
            node = nodes[prevNodeName]
            e = Edge()
            e.node = nodeName
            e.action = 'move_base'
            node.edges.append(e)

        prevNodeName = nodeName


    # vertical nodes
    prevNodeName = ''
    for y in range(-(height/2),(height/2)+1): 
        if y != 0: 
            nodeName = 'v_%s'%y
            node = TopologicalNode(name=nodeName, pose=Pose(position=Point(xOrigin, y * nodeSeparation, 0)))
            nodes[nodeName] = node
        else:
            nodeName = 'origin'
            node = originNode
            
        if prevNodeName:
            # connect up this to previous
            e = Edge()
            e.node = prevNodeName
            e.action = 'move_base'
            node.edges.append(e)
            # previous to this
            node = nodes[prevNodeName]
            e = Edge()
            e.node = nodeName
            e.action = 'move_base'
            node.edges.append(e)


        prevNodeName = nodeName

    return nodes

