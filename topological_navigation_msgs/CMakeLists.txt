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
find_package(action_msgs REQUIRED)
set(msg_deps
  builtin_interfaces
  std_msgs
  geometry_msgs
  action_msgs
)




########################
#
#   FILE DECLARATION
#

# Declare the custom message files
set(msg_files
  "msg/ExecutePolicyModeGoal.msg"
  "msg/ClosestEdges.msg"
  "msg/TopologicalMap.msg"
  "msg/TopologicalNode.msg"
  "msg/Vertex.msg"
  "msg/Edge.msg"
  "msg/NavRoute.msg"
  "msg/CurrentEdge.msg"
  "msg/NavStatistics.msg"
  "msg/TopologicalRoute.msg"
  "msg/TopologicalOccupiedNode.msg"
  "msg/AddEdgeReq.msg"
  "msg/AddNodeReq.msg"
  "msg/SetInfluenceZoneReq.msg"
  "msg/UpdateEdgeConfigReq.msg"
  "msg/GotoNodeFeedback.msg"
  "msg/ExecutePolicyModeFeedback.msg"
)


# Declare the custom service files
set(srv_files
  "srv/WriteTopologicalMap.srv"
  "srv/EvaluateNode.srv"
  "srv/EvaluateEdge.srv"
  "srv/LocalisePose.srv"
  "srv/GetRouteTo.srv"
  "srv/GetRouteBetween.srv"
  "srv/LoadTopoNavTestScenario.srv"
  "srv/RunTopoNavTestScenario.srv"
  "srv/ReconfAtEdges.srv"
  "srv/LoadTmap.srv"
)



#Generate actions in the 'action' folder
# Declare the custom service files
set(action_files
    "action/GotoNode.action"
    "action/ExecutePolicyMode.action"
    "action/BuildTopPrediction.action"
    "action/RecordMap.action"
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
