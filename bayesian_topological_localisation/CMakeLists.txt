cmake_minimum_required(VERSION 3.5)
project(bayesian_topological_localisation)


############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


# Define dependencies for messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(msg_deps
  std_msgs
  geometry_msgs
)



########################
#
#   FILE DECLARATION
#

# Declare the custom message files
set(msg_files
  "msg/DistributionStamped.msg"
  "msg/PoseObservation.msg"
  "msg/LikelihoodObservation.msg"
  "msg/ParticlesState.msg"
)


# Declare the custom service files
set(srv_files
  "srv/LocaliseAgent.srv"
  "srv/StopLocalise.srv"
  "srv/UpdatePoseObservation.srv"
  "srv/UpdateLikelihoodObservation.srv"
  "srv/UpdatePriorLikelihoodObservation.srv"
  "srv/Predict.srv"
  "srv/SetFloat64.srv"
)



#######################
#
#   FILE GENERATION
#

# Generate the files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${msg_deps}
)





#########################
#
#   PYTHON COMPILATION
#

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/localisation_node.py
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install the launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)



###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()






