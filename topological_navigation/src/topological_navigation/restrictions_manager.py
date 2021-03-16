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
        return value == robot_state["robot"]



class RestrictionsManager():
    def __init__(self):
        # here are the conditions that can be evaluiated in the predicates
        self.conditions = {}

        # get the topological map
        self.topo_map = None

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
                node_restrictions = node["node"]["restrictions"]
                # get the sympy boolean predicate removing the atoms that cannot be grounded
                restriction_predicate = self._predicate_from_string(node_restrictions)

                # evaluate each restriction of the predicate
                while len(restriction_predicate.atoms()) > 0:
                    restriction = restriction_predicate.atoms[0]
                    restr_elements = restriction.split("_")
                    condition = self.conditions[restr_elements[0]]
                    evaluation = condition.get_value(
                        node = node["meta"]["node"],
                        value = restr_elements[1] if len(restr_elements) > 1 else None,
                        robot_state = robot_state
                    )
                    # ground the restriction and simplify the predicate
                    restriction_predicate = restriction_predicate.subs({
                        restriction: evaluation
                    }).simplify()

                if restriction_predicate:
                    del new_topo_map[i]

            response.success = True
            response.restricted_tmap = new_topo_map
        
        return response

    # Create a predicate from the restrictions string and returns the conditions to be checked
    def _predicate_from_string(self, restriction_str, entity_name):
        try:
            predicate = parse_expr(restriction_str)
        except Exception as e:
            rospy.logerr("The restriction expression for {} is malformed, error: {}".format(entity_name, e))
            rospy.logerr("The expression can contain the following conditions: {}\n and the following boolean symbols: &, |, ~, (, ).".format(self.conditions))
        else:
            conds = predicate.atoms()
            for cond in conds:
                cond_els = cond.split("_")
                if cond_els[0] not in self.condition.keys():
                    rospy.logwarn("The restriction {} has not been implemented yet, setting `{}==False`.".format(cond_els[0], cond))
                    # setting the restriction to false
                    predicate = predicate.subs({
                        cond: False
                    })
        return predicate.simplify()

    def _topomap_cb(self, msg):
        self.topo_map = yaml.safe_load(msg.data)
        print("got topomap", self.topo_map["nodes"][0])


if __name__ == '__main__':
    rospy.init_node("restrictions_manager")

    manager = RestrictionsManager()

    # TODO make this automatically find and register all the classes of AbstractRestriction type
    for restriction in [RobotType]:
        manager.register_restriction(restriction)

    rospy.spin()
