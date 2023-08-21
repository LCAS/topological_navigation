#!/usr/bin/env python
import math
import rospy
import sys

import std_msgs.msg
from topological_navigation.topological_node import *
from topological_navigation_msgs.msg import TopologicalNode, TopologicalMap, Edge
from geometry_msgs.msg import Pose, Point


def create_cross_map(width, height, nodeSeparation):
    """ Creates a vertical cross with 5 nodes in each line, with the middle at 0,0 """
    xOrigin = 0
    yOrigin = 0
    nodeSeparation = 10

    nodes = dict()

    originNode = TopologicalNode(name='ChargingPoint', pose=Pose(position=Point(xOrigin, yOrigin, 0)))
    nodes['ChargingPoint'] = originNode



    # horizontal nodes
    prevNodeName = ''
    for x in range(-(width/2),(width/2)+1):
        if x != 0:
            nodeName = 'h_%s'%x
            node = TopologicalNode(name=nodeName, pose=Pose(position=Point(x * nodeSeparation, yOrigin, 0)))
            nodes[nodeName] = node
        else:
            nodeName = 'ChargingPoint'
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
            nodeName = 'ChargingPoint'
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


def create_line_map(width, nodeSeparation):
    """ Creates a line map of ChargingPoint, Station, then width - 2 waypoints """
    xOrigin = 0
    yOrigin = 0
    nodeSeparation = 10

    nodes = []

    nodes.append(TopologicalNode(name='ChargingPoint', pose=Pose(position=Point(xOrigin, yOrigin, 0))))
    nodes.append(TopologicalNode(name='Station', pose=Pose(position=Point(xOrigin + nodeSeparation, yOrigin, 0))))

    for i in range(width - 2):
        nodes.append(TopologicalNode(name='WayPoint%s'%i, pose=Pose(position=Point(xOrigin + nodeSeparation * i, yOrigin, 0))))

    for i in range(len(nodes) - 1):
        n1 = nodes[i]
        n2 = nodes[i+1]
        
        n1n2 = Edge()
        n1n2.node = n2.name
        n1n2.action = 'move_base'
        n1n2.edge_id = '%s_%s' % (n1.name, n2.name)
        n1.edges.append(n1n2)

        n2n1 = Edge()
        n2n1.node = n1.name
        n2n1.action = 'move_base'
        n2n1.edge_id = '%s_%s' % (n2.name, n1.name)
        n2.edges.append(n2n1)


    nodes = {node.name: node for node in nodes}

    # print nodes['Station']

    return nodes

