#!/usr/bin/env python
import rospy
import sympy
import copy
import yaml
import sys
import inspect
from topological_navigation_msgs.srv import RestrictMap, RestrictMapRequest, RestrictMapResponse
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest
from std_msgs.msg import String
from abc import ABCMeta, abstractmethod, abstractproperty
from sympy.parsing.sympy_parser import parse_expr

###############################
##
# Here extend the class AbstractRestriction to create a new restriction.
# Once its defined here it will be automatically imported by the RestrictionsManager.
##
class AbstractRestriction():
    """ Abstract definition of a restriction that can be evaluated in real-time """
    __metaclass__ = ABCMeta

    robot_state = {}

    @abstractmethod
    def evaluate(self, node=None, value=None, robot_state={}):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        raise NotImplementedError()

    @abstractproperty
    def name(self):
        """ The name must correspond to the restriction variable name in the map definition """
        raise NotImplementedError()

    @abstractmethod 
    def ground_to_robot(self, robot_namespace=None):
        """ Retrieves and saves internally the state of the robot necessary for evaluating the restriction  """
        raise NotImplementedError()

class RobotType(AbstractRestriction):
    name = "robot"

    def __init__(self):
        super(AbstractRestriction, self).__init__()

    def evaluate(self, node, value, robot_state={}):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        evaluation = True
        if "robot" in robot_state:
            evaluation = value.lower() == robot_state["robot"].lower()
        elif "robot" in self.robot_state:
            evaluation = value.lower() == self.robot_state["robot"].lower()
        
        return evaluation

    def ground_to_robot(self, robot_namespace=None):
        if robot_namespace is None:
            # robot namespace under which this script is running
            robot_namespace = rospy.get_namespace()
        try:
            topic_name = "/{}/type".format(robot_namespace)
            robot_type = rospy.wait_for_message(topic_name, String, timeout=2)
        except rospy.ROSException:
            rospy.logwarn("Grounding of restrcition {} failed, topic {} not published".format(self.name, topic_name))
        else:
            self.robot_state.update({
                "robot": robot_type["data"]
            })

class TaskName(AbstractRestriction):
    name = "task"

    def __init__(self):
        super(AbstractRestriction, self).__init__()

    def evaluate(self, node, value, robot_state={}):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        evaluation = True
        if "task" in robot_state:
            evaluation = value.lower() == robot_state["task"].lower()
        elif "task" in self.robot_state:
            evaluation = value.lower() == self.robot_state["task"].lower()

        return evaluation

    def ground_to_robot(self, robot_namespace=None):
        if robot_namespace is None:
            # robot namespace under which this script is running
            robot_namespace = rospy.get_namespace()
        try:
            topic_name = "/{}/task".format(robot_namespace)
            robot_type = rospy.wait_for_message(topic_name, String, timeout=2)
        except rospy.ROSException:
            rospy.logwarn("Grounding of restrcition {} failed, topic {} not published".format(
                self.name, topic_name))
        else:
            self.robot_state.update({
                "task": robot_type["data"]
            })
###############################


class RestrictionsManager():
    def __init__(self):
        # here are the conditions that can be evaluiated in the predicates
        self.conditions = {}

        # get the topological map
        self.topo_map = None
        # for each node contains the indexes of the nodes which have an edge ending in it
        self.back_edges_idx = {}

        rospy.Subscriber("topological_map_2",
                         String, self._topomap_cb)
        rospy.loginfo("Waiting for topological map...")
        while self.topo_map is None:
            rospy.sleep(0.5)
        rospy.loginfo("DONE")

        rospy.Service("/topological_restrictions_manager/restrict_planning_map", RestrictMap, self.restrict_planning_map_handle)
        rospy.Service("/topological_restrictions_manager/restrict_runtime_map", RestrictMap, self.restrict_runtime_map_handle)
    
    def register_restriction(self, condition_obj):
        """ This method receive a condition implementing AbstractRestriction class that can be evaluated"""
        self.conditions.update({
            condition_obj.name: condition_obj
        })
        rospy.loginfo(
            "New condition `{}` registered".format(condition_obj.name))

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

    def _evaluate_restrictions(self, restrictions, node_name, robot_state):
        # print("evaluate restrictions `{}` for {} and robot_state {}".format(restrictions, node_name, robot_state))

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
            evaluation = condition.evaluate(
                node = node_name,
                value = restr_elements[1] if len(
                    restr_elements) > 1 else None,
                **robot_state
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
        return self._restrict_map_handle(request, "restrictions_planning")
    
    def restrict_runtime_map_handle(self, request):
        return self._restrict_map_handle(request, "restrictions_runtime")
   
    def _restrict_map_handle(self, request, restrictions_arg):
        response = RestrictMapResponse()
        new_topo_map = copy.deepcopy(self.topo_map)
        try:
            robot_state = eval(request.state)
        except TypeError:
            rospy.logerr("Message data conversion to dictionary not valid, skip message")
            response.success = False    
        else:
            # tmp_topo_map = copy.deepcopy(self.topo_map)
            to_remove_edges = {}
            to_remove_nodes = set({})
            # for each node, check restrictions
            for i, node in enumerate(self.topo_map["nodes"]):
                node_restrictions = str(node["node"][restrictions_arg])
                
                is_restricted = self._evaluate_restrictions(node_restrictions, node["meta"]["node"], robot_state)

                # remove the node if the restriction evaluates as False
                if is_restricted:
                    # print("{}\n\tremoving node {}".format(
                    #     node_restrictions, new_topo_map["nodes"][i]))
                    to_remove_nodes.add(i)

                    # remove all the back edges
                    for (ni, ei) in self.back_edges_idx[node["meta"]["node"]].items():
                        # print("\n\t removing edge {}".format(
                        #     new_topo_map["nodes"][ni]["node"]["edges"][ei]))
                        if ni in to_remove_edges:
                            to_remove_edges[ni].add(ei)
                        else:
                            to_remove_edges.update({ni: set({ei})})
                else:
                    # here now check the restrictions for the node's edges
                    for ei, edge in enumerate(node["node"]["edges"]):
                        edge_restrictions = str(edge[restrictions_arg])

                        is_restricted = self._evaluate_restrictions(edge_restrictions, edge["edge_id"], robot_state)

                        if is_restricted:
                            # print("{}\n\t removing edge {}".format(node_restrictions,
                            #     new_topo_map["nodes"][i]["node"]["edges"][ei]))
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

            response.success = True

        response.restricted_tmap = yaml.dump(new_topo_map)
        return response

    def _topomap_cb(self, msg):
        self.topo_map = yaml.safe_load(msg.data)
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


if __name__ == '__main__':
    rospy.init_node("topological_restrictions_manager")

    manager = RestrictionsManager()

    # automatically find and register all the classes of AbstractRestriction type
    clsmembers = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    for restriction in clsmembers:
        if issubclass(restriction[1], AbstractRestriction) and not inspect.isabstract(restriction[1]):
            manager.register_restriction(restriction[1]())

    rospy.spin()
