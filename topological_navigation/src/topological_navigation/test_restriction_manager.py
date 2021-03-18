#!/usr/bin/env python

import rospy
import yaml
from topological_navigation_msgs.srv import RestrictMap, RestrictMapRequest, RestrictMapResponse

rospy.init_node("test")

rospy.sleep(1)

##
srvpr = rospy.ServiceProxy("/topological_restrictions_manager/restrict_planning_map", RestrictMap)

req = RestrictMapRequest()
req.state = '{"robot": "short", "task": "uv"}'

res = srvpr.call(req)
print("planning restrictions", res.success, len(yaml.safe_load(res.restricted_tmap)["nodes"]))
rospy.sleep(1)

##
srvpr = rospy.ServiceProxy("/topological_restrictions_manager/restrict_runtime_map", RestrictMap)

req = RestrictMapRequest()
req.state = '{"robot": "tall", "task": "uv"}'

res = srvpr.call(req)

print("runtime restrictions", res.success, len(yaml.safe_load(res.restricted_tmap)["nodes"]))
