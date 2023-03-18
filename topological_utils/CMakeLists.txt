cmake_minimum_required(VERSION 3.5)
project(topological_utils)



############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define dependencies for services
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
set(srv_deps
  std_msgs
)





########################
#
#   FILE DECLARATION
#

# Declare the custom service files
set(srv_files
    "srv/NodeMetadata.srv"
)



#######################
#
#   FILE GENERATION
#

# Generate the files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES ${srv_deps}
)






#########################
#
#   PYTHON COMPILATION
#

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY support
  DESTINATION share/${PROJECT_NAME}/
)




###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()








