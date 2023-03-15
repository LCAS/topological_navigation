cmake_minimum_required(VERSION 3.5)
project(topological_navigation_msgs)




############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define dependencies for messages
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
set(msg_deps
  builtin_interfaces
  std_msgs
  geometry_msgs
  actionlib_msgs
)




########################
#
#   FILE DECLARATION
#

# Declare the custom message files
set(msg_files
  "msg/ClosestEdges.msg"
  "msg/TopologicalMap.msg"
  "msg/TopologicalNode.msg"
  "msg/Vertex.msg"
  "msg/Edge.msg"
  "msg/NavRoute.msg"
  "msg/CurrentEdge.msg"
  "msg/NavStatistics.msg"
  "msg/TopologicalRoute.msg"
  "msg/AddEdgeReq.msg"
  "msg/AddNodeReq.msg"
  "msg/SetInfluenceZoneReq.msg"
  "msg/UpdateEdgeConfigReq.msg"
)


# Declare the custom service files
set(srv_files
  "srv/WriteTopologicalMap.srv"
  "srv/UpdateEdgeConfig.srv"
  "srv/UpdateRestrictions.srv"
  "srv/UpdateEdge.srv"
  "srv/UpdateAction.srv"
  "srv/RestrictMap.srv"
  "srv/EvaluateNode.srv"
  "srv/EvaluateEdge.srv"
  "srv/AddDatum.srv"
  "srv/GetTaggedNodes.srv"
  "srv/GetTags.srv"
  "srv/GetNodeTags.srv"
  "srv/GetEdgesBetweenNodes.srv"
  "srv/AddNode.srv"
  "srv/RmvNode.srv"
  "srv/AddEdge.srv"
  "srv/AddContent.srv"
  "srv/UpdateNodeName.srv"
  "srv/UpdateNodeTolerance.srv"
  "srv/ModifyTag.srv"
  "srv/AddTag.srv"
  "srv/GetTopologicalMap.srv"
  "srv/UpdateEdgeLegacy.srv"
  "srv/LocalisePose.srv"
  "srv/GetRouteTo.srv"
  "srv/GetRouteBetween.srv"
  "srv/EstimateTravelTime.srv"
  "srv/PredictEdgeState.srv"
  "srv/UpdateFailPolicy.srv"
  "srv/ReconfAtEdges.srv"
  "srv/LoadTopoNavTestScenario.srv"
  "srv/RunTopoNavTestScenario.srv"
  "srv/SetInfluenceZone.srv"
  "srv/AddEdgeArray.srv"
  "srv/AddNodeArray.srv"
  "srv/SetInfluenceZoneArray.srv"
  "srv/UpdateEdgeConfigArray.srv"
)



#Generate actions in the 'action' folder
# Declare the custom service files
set(action_files
    "action/GotoNode.action"
    "action/ExecutePolicyMode.action"
    "action/BuildTopPrediction.action"
)



#######################
#
#   FILE GENERATION
#

# Generate the files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES ${msg_deps}
)


###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()
