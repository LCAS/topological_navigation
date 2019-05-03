#!/usr/bin/env python
import rospy, dynamic_reconfigure.client
from strands_navigation_msgs.srv import ReconfAtEdges, ReconfAtEdgesResponse


class reconf_at_edges_server(object):


    def __init__(self):
        self.edge_groups = rospy.get_param("/edge_nav_reconfig_groups", {})
        rospy.Service('reconf_at_edges', ReconfAtEdges, self.handle_reconf_at_edges)


    def handle_reconf_at_edges(self, req):

        self.success = False

        for group in self.edge_groups:

            if req.edge_id in self.edge_groups[group]["edges"]:
                rospy.loginfo("\nSetting parameters for edge group {} ...".format(group))

                self.set_parameters(group)
                break

        return ReconfAtEdgesResponse(self.success)


    def set_parameters(self, group):

        try:
            for param in self.edge_groups[group]["parameters"]:
                print "Setting {} = {}".format("/".join((param["ns"], param["name"])), param["value"])

                rcnfclient = dynamic_reconfigure.client.Client(param["ns"], timeout=5.0)
                rcnfclient.update_configuration({param["name"]: param["value"]})

            self.success = True

        except rospy.ROSException as e:
            rospy.logerr(e)




if __name__ == "__main__":

    rospy.init_node('reconf_at_edges_server', anonymous=True)
    reconf_at_edges_server()
    rospy.spin()
#####################################################################################
