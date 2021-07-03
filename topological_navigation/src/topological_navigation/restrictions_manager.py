#!/usr/bin/env python
import rospy
import sympy
import copy
import json
import yaml
import sys
import inspect
from topological_navigation_msgs.srv import RestrictMap, RestrictMapRequest, RestrictMapResponse
from topological_navigation.manager2 import map_manager_2
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest
from std_msgs.msg import String
from sympy.parsing.sympy_parser import parse_expr

from restrictions_impl import *


class RestrictionsManager():
    def __init__(self, robots, tasks):
        self.robots = robots
        self.tasks = tasks

        # here are the conditions that can be evaluiated in the predicates
        self.conditions = {}

        # get the topological map
        self.topo_map = None
        # for each node contains the indexes of the nodes which have an edge ending in it
        self.back_edges_idx = {}

        # publishers for restricted tmap
        self.restricted_tmap_pub = rospy.Publisher("topological_map",
                         TopologicalMap, queue_size=10, latch=True)
        self.restricted_tmap2_pub = rospy.Publisher(
                         "topological_map_2", String, queue_size=10,
                         latch=True)
    
    def register_restriction(self, condition_obj):
        """ This method receive a condition implementing AbstractRestriction class that can be evaluated"""
        self.conditions.update({
            condition_obj.name: condition_obj
        })
        rospy.loginfo(
            "New condition `{}` registered".format(condition_obj.name))

    def register_connections(self):
        rospy.Subscriber("/topological_map_2",
                         String, self._topomap_cb)
        rospy.loginfo("Waiting for topological map...")
        while self.topo_map is None:
            rospy.sleep(0.5)
        rospy.loginfo("DONE")

        rospy.Service("restrictions_manager/restrict_planning_map",
                      RestrictMap, self.restrict_planning_map_handle)
        rospy.Service("restrictions_manager/restrict_runtime_map",
                      RestrictMap, self.restrict_runtime_map_handle)

    # Create a predicate from the restrictions string and returns the conditions to be checked
    def _predicate_from_string(self, restriction_str):
        predicate = True
        if restriction_str == "":
            restriction_str = "True"
        try:
            predicate = parse_expr(restriction_str)
        except Exception as e:
            rospy.logerr(
                "The restriction expression is malformed, error: {}".format(e))
            rospy.logerr("The expression can contain the following conditions: {}\n and the following boolean symbols: &, |, ~, (, ).".format(
                self.conditions.keys()))
        else:
            if isinstance(predicate, dict):
                rospy.logerr(
                    "The restriction expression is malformed: {}".format(predicate))
                predicate = True
            elif not isinstance(predicate, bool):
                conds = predicate.atoms()
                for cond in conds:
                    cond_els = cond.name.split("_")
                    if cond_els[0] not in self.conditions.keys():
                        rospy.logwarn("The restriction `{}` has not been implemented yet, setting `{} = False`.".format(
                            cond_els[0], cond.name))
                        # setting the restriction to false
                        predicate = predicate.subs({
                            cond.name: False
                        })
                predicate = predicate.simplify()
        return predicate

    def _evaluate_restrictions(self, restrictions, entity_name, robot_state, for_node=True):
        # print("evaluate restrictions `{}` for {} and robot_state {}".format(restrictions, entity_name, robot_state))

        # get the sympy boolean predicate removing the atoms that cannot be grounded
        restriction_predicate = self._predicate_from_string(restrictions)
        # print("\tsimplified predicate: {}".format(restriction_predicate))

        # evaluate each restriction of the predicate until it simplifies to a boolean
        while not isinstance(restriction_predicate, bool) and\
                not restriction_predicate is sympy.true and \
                not restriction_predicate is sympy.false:
            restriction = restriction_predicate.atoms().pop()
            restr_elements = restriction.name.split("_")
            condition = self.conditions[restr_elements[0]]
            if for_node:
                evaluation = condition.evaluate_node(
                    node = entity_name,
                    value = restr_elements[1] if len(
                        restr_elements) > 1 else None,
                    robot_state=robot_state,
                    tmap=self.topo_map
                )
            else:
                evaluation = condition.evaluate_edge(
                    edge = entity_name,
                    value = restr_elements[1] if len(
                        restr_elements) > 1 else None,
                    robot_state=robot_state,
                    tmap=self.topo_map
                )
            # print("\tsubstituting {} to {}".format(evaluation, restriction.name))

            # ground the restriction and simplify the predicate
            restriction_predicate = restriction_predicate.subs({
                restriction.name: evaluation
            }).simplify()

            # print("\t\t res: {}, {}".format(
            #     restriction_predicate, type(restriction_predicate)))

        # here we negate because if the restriction matches (True) it means the robot is allowed to pass (not restricted)
        return not restriction_predicate
        
    def restrict_planning_map_handle(self, request):
        response = RestrictMapResponse()
        response.success = True

        new_topo_map = self._restrict_map_handle(request, "restrictions_planning")

        response.restricted_tmap = json.dumps(new_topo_map)


        self._publish_updated_restricted_maps(new_topo_map)

        return response
    
    def restrict_runtime_map_handle(self, request):
        response = RestrictMapResponse()
        response.success = True

        new_topo_map = self._restrict_map_handle(request, "restrictions_runtime")

        response.restricted_tmap = json.dumps(new_topo_map)

        self._publish_updated_restricted_maps(new_topo_map)

        return response
   
    def _restrict_map_handle(self, request, restrictions_arg):
        new_topo_map = copy.deepcopy(self.topo_map)
        robot_state = {}
        try:
            robot_state = eval(request.state)
        except:
            rospy.logwarn("Robot state data conversion to dictionary not valid, skip message")
            # robot_state = None 
        finally:
            # tmp_topo_map = copy.deepcopy(self.topo_map)
            to_remove_edges = {}
            to_remove_nodes = set({})
            # for each node, check restrictions
            for i, node in enumerate(self.topo_map["nodes"]):
                node_restrictions = str(node["node"][restrictions_arg])
                
                is_restricted = self._evaluate_restrictions(node_restrictions, node["meta"]["node"], robot_state, for_node=True)

                # remove the node if the restriction evaluates as False
                if is_restricted:
                    print("{}\n\tremoving node {}".format(
                        node_restrictions, new_topo_map["nodes"][i]["meta"]["node"]))
                    to_remove_nodes.add(i)

                    # remove all the back edges
                    for (ni, ei) in self.back_edges_idx[node["meta"]["node"]].items():
                        print("\n\t removing edge {}".format(
                            new_topo_map["nodes"][ni]["node"]["edges"][ei]["edge_id"]))
                        if ni in to_remove_edges:
                            to_remove_edges[ni].add(ei)
                        else:
                            to_remove_edges.update({ni: set({ei})})
                else:
                    # here now check the restrictions for the node's edges
                    for ei, edge in enumerate(node["node"]["edges"]):
                        edge_restrictions = str(edge[restrictions_arg])

                        is_restricted = self._evaluate_restrictions(edge_restrictions, edge["edge_id"], robot_state, for_node=False)

                        if is_restricted:
                            print("{}\n\t removing edge {}".format(node_restrictions,
                                new_topo_map["nodes"][i]["node"]["edges"][ei]["edge_id"]))
                            if i in to_remove_edges:
                                to_remove_edges[i].add(ei)
                            else:
                                to_remove_edges.update({i: set({ei})})

            # now remove all the edges
            for ni, eis in to_remove_edges.items():
                for ei in sorted(eis, reverse=True):
                    del new_topo_map["nodes"][ni]["node"]["edges"][ei]
            # now remove all the nodes
            for ni in sorted(to_remove_nodes, reverse=True):
                del new_topo_map["nodes"][ni]

        return new_topo_map

    def _topomap_cb(self, msg):

        rospy.loginfo("Received updated topomap")
        self.topo_map = json.loads(msg.data)
        # print("got topomap", self.topo_map["nodes"][0])

        for ni, node in enumerate(self.topo_map["nodes"]):
            for ei, edge in enumerate(node["node"]["edges"]):
                if edge["node"] in self.back_edges_idx:
                    self.back_edges_idx[edge["node"]].update({
                        ni: ei
                    })
                else:
                    self.back_edges_idx.update({
                        edge["node"]: {ni: ei}
                    })

        rospy.loginfo("Creating restricted topomaps")
        restricted_tmap2 = self._restrict_map_handle({}, "restrictions_planning")

        self._publish_updated_restricted_maps(restricted_tmap2)

    def _publish_updated_restricted_maps(self, restricted_tmap2):
        self.restricted_tmap2_pub.publish(json.dumps(restricted_tmap2))

        old_restricted_tmap = map_manager_2.convert_tmap2_to_tmap(
            restricted_tmap2, restricted_tmap2["pointset"], restricted_tmap2["metric_map"])

        self.restricted_tmap_pub.publish(old_restricted_tmap)


def get_admissible_robots(config):
    robots = []
    if "admissible_robot_ids" in config:
        robots = config["admissible_robot_ids"]

    return robots

def get_admissible_tasks(config):
    tasks = []
    if "active_tasks" in config:
        tasks = config["active_tasks"]

    return tasks

if __name__ == '__main__':
    rospy.init_node("topological_restrictions_manager")

    config_file = sys.argv[1]

    _config = {}
    with open(config_file, "r") as f_handle:
        _config = yaml.load(f_handle)

    # which robots can stay in the field
    _robots = get_admissible_robots(_config)
    # which tasks we can executed in the field
    _tasks = get_admissible_tasks(_config)

    manager = RestrictionsManager(robots=_robots, tasks=_tasks)

    # automatically find and register all the classes of AbstractRestriction type
    clsmembers = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    for restriction in clsmembers:
        if issubclass(restriction[1], AbstractRestriction) and not inspect.isabstract(restriction[1]):
            cls_argnames = inspect.getargspec(restriction[1].__init__).args
            args = {}
            if "robots" in cls_argnames:
                args["robots"] = _robots
            if "tasks" in cls_argnames:
                args["tasks"] = _tasks
            manager.register_restriction(restriction[1](**args))

    manager.register_connections()

    rospy.spin()
