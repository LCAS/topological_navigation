import rospy
import sympy
import copy
import yaml
from topological_navigation_msgs.srv import RestrictMap, RestrictMapRequest, RestrictMapResponse
from strands_navigation_msgs.srv import GetTaggedNodes, GetTaggedNodesRequest
from std_msgs.msg import String
from abc import ABCMeta, abstractmethod, abstractproperty
from sympy.parsing.sympy_parser import parse_expr


class AbstractRestriction():
    """ Abstract definition of a restriction that can be evaluated in real-time """
    __metaclass__ = ABCMeta

    @abstractmethod
    def get_value(self, node=None, value=None, **robot_state):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        raise NotImplementedError()

    @abstractproperty
    def name(self):
        """ The name must correspond to the restriction name in the map definition """
        raise NotImplementedError()


class Blacklisted(AbstractRestriction):
    
    name= "blacklisted"

    def __init__(self):
        super(AbstractRestriction, self).__init__()
        try:
            rospy.wait_for_service(
                'topological_map_manager/get_tagged_nodes', timeout=2)
        except ROSException as e:
            rospy.logwarn(
                "Service /topological_map_manager/get_tagged_nodes not available: %s" % e)
        else:
            self.taggedNodesSrv = rospy.ServiceProxy(
                'topological_map_manager/get_tagged_nodes', GetTaggedNodes)

    def get_value(self, node, value=None, **robot_state):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        request = GetTaggedNodesRequest()
        request.tag = "blacklisted"
        c_nodes = self.taggedNodesSrv(request).nodes
        
        if node in c_nodes:
            return True

        return False


class RobotType(AbstractRestriction):

    name = "robot"

    def __init__(self):
        super(AbstractRestriction, self).__init__()

    def get_value(self, node, value, **robot_state):
        """ Returns the value of the restriction associated with the given entity, must return a boolean value  """
        print(robot_state)
        return value == robot_state["robot"]



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

        rospy.Service("/restrictions_manager/restrict_map", RestrictMap, self.restrict_map_handle)
    
    def register_restriction(self, condition_obj):
        """ This method receive a condition implementing AbstractRestriction class that can be evaluated"""
        self.conditions.update({
            condition_obj.name: condition_obj
        })
        rospy.loginfo("New condition `{}` registered".format(condition_obj.name))

    def _evaluate_restrictions(self, restrictions, node_name, robot_state):
        # get the sympy boolean predicate removing the atoms that cannot be grounded
        restriction_predicate = self._predicate_from_string(restrictions)

        # evaluate each restriction of the predicate until it simplifies to a boolean
        while not isinstance(restriction_predicate, bool) and \
                not restriction_predicate.is_Boolean:
            restriction = restriction_predicate.atoms().pop()
            restr_elements = restriction.name.split("_")
            condition = self.conditions[restr_elements[0]]
            evaluation = condition.get_value(
                node = node_name,
                value = restr_elements[1] if len(
                    restr_elements) > 1 else None,
                **robot_state
            )
            # ground the restriction and simplify the predicate
            restriction_predicate = restriction_predicate.subs({
                restriction.name: evaluation
            }).simplify()

        # here we negate because if the restriction matches (True) it means the robot is allowed to pass (not restricted)
        return not restriction_predicate

    def restrict_map_handle(self, request):
        response = RestrictMapResponse()
        try:
            robot_state = eval(request.state)
        except TypeError:
            rospy.logerr("Message data conversion to dictionary not valid, skip message")
            response.success = False    
        else:
            new_topo_map = copy.deepcopy(self.topo_map)

            # for each node, check restrictions
            for i, node in enumerate(self.topo_map["nodes"]):
                node_restrictions = str(node["node"]["restrictions"])
                
                is_restricted = self._evaluate_restrictions(node_restrictions, node["meta"]["node"], robot_state)

                # remove the node if the restriction evaluates as False
                if is_restricted:
                    del new_topo_map[i]
                    # remove all the back edges
                    for (ni, ei) in self.back_edges_idx[node["meta"]["node"]].items():
                        del new_topo_map["nodes"][ni]["node"]["edges"][ei]
                else:
                    # here now check the restrictions for the node's edges
                    for ei, edge in enumerate(node["node"]["edges"]):
                        edge_restrictions = str(edge["restrictions"])

                        is_restricted = self._evaluate_restrictions(edge_restrictions, edge["edge_id"], robot_state)

                        if is_restricted:
                            del new_topo_map["nodes"][i]["node"]["edges"][ei]

            response.success = True
            response.restricted_tmap = yaml.dump(new_topo_map)
        
        return response

    # Create a predicate from the restrictions string and returns the conditions to be checked
    def _predicate_from_string(self, restriction_str):
        try:
            predicate = parse_expr(restriction_str)
        except Exception as e:
            rospy.logerr("The restriction expression is malformed, error: {}".format(e))
            rospy.logerr("The expression can contain the following conditions: {}\n and the following boolean symbols: &, |, ~, (, ).".format(self.conditions))
        else:
            if isinstance(predicate, dict):
                rospy.logerr("The restriction expression is malformed: {}".format(predicate))
                return True

            if not isinstance(predicate, bool):
                conds = predicate.atoms()
                for cond in conds:
                    cond_els = cond.name.split("_")
                    if cond_els[0] not in self.conditions.keys():
                        rospy.logwarn("The restriction {} has not been implemented yet, setting `{}==False`.".format(cond_els[0], cond.name))
                        # setting the restriction to false
                        predicate = predicate.subs({
                            cond.name: False
                        })
                predicate = predicate.simplify()
        return predicate

    def _topomap_cb(self, msg):
        self.topo_map = yaml.safe_load(msg.data)
        print("got topomap", self.topo_map["nodes"][0])

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
        # print(self.back_edges_idx)


if __name__ == '__main__':
    rospy.init_node("restrictions_manager")

    manager = RestrictionsManager()

    # TODO make this automatically find and register all the classes of AbstractRestriction type
    for restriction in [RobotType]:
        manager.register_restriction(restriction())

    rospy.spin()
