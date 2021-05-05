#!/usr/bin/env python

import rospy
import yaml
from topological_navigation_msgs.srv import RestrictMap, RestrictMapRequest, RestrictMapResponse

rospy.init_node("test")

rospy.sleep(1)

##
srvpr = rospy.ServiceProxy("/thorvald_001/restrictions_manager/restrict_planning_map", RestrictMap)

req = RestrictMapRequest()
# req.state = '{"robot": "short", "task": "uv"}'

res = srvpr.call(req)
print("thorvald_001 restrictions", res.success, len(
    yaml.safe_load(res.restricted_tmap)["nodes"]))
rospy.sleep(1)

##
srvpr = rospy.ServiceProxy("/thorvald_002/restrictions_manager/restrict_planning_map", RestrictMap)

req = RestrictMapRequest()
# req.state = '{"robot": "tall", "task": "uv"}'

res = srvpr.call(req)

print("thorvald_002 restrictions", res.success, len(
    yaml.safe_load(res.restricted_tmap)["nodes"]))
