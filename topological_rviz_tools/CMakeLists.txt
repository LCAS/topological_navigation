cmake_minimum_required(VERSION 3.5)
project(topological_rviz_tools)


####################################
#
#   DEFINE COMPILER SPECIFICATION
#


# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()
#
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()


############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Define dependencies for messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(topological_navigation_msgs REQUIRED)

# Define dependencies for RViz Plugins
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)

# Group dependencies for easier referencing
set(srv_deps
  std_msgs
  geometry_msgs
  topological_navigation_msgs
)


########################
#
#   FILE DECLARATION
#

# Declare the custom service files
set(srv_files
  "srv/AddEdge.srv"
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
#   BUILD RVIZ2 PLUGIN
#

# Group dependencies for easier referencing
set(rviz_deps
  ament_cmake
  rclcpp
  rclcpp_components
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  topological_navigation_msgs
  pluginlib
)

## Specify the 'include' directories for the project
#include_directories(
#  include
#  ${rclcpp_INCLUDE_DIRS}
#  ${rviz_common_INCLUDE_DIRS}
#  ${rviz_rendering_INCLUDE_DIRS}
#  ${topological_navigation_msgs_INCLUDE_DIRS}
#)
#
#
#
## Construct the library and define its dependencies
## Declare files to be included under the plugin
#set(rviz_files
#  src/topological_edge_tool.cpp
#  src/topological_node_tool.cpp
#  src/topological_map_panel.cpp
#  src/node_controller.cpp
#  src/topmap_manager.cpp
#  src/node_property.cpp
#  src/edge_property.cpp
#  src/pose_property.cpp
#  src/edge_controller.cpp
#  src/tag_controller.cpp
#  src/tag_property.cpp
#)
#add_library(${PROJECT_NAME}_lib SHARED ${rviz_files})
#
#target_include_directories(${PROJECT_NAME}_lib
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
#  PRIVATE
#    ${rviz_common_INCLUDE_DIRS}
#    ${rviz_default_plugins_INCLUDE_DIRS}
#    ${rviz_rendering_INCLUDE_DIRS}
#)
#
#ament_target_dependencies(${PROJECT_NAME}_lib ${rviz_deps})
#
#
#
#
## Set compile-time definitions
#target_compile_definitions(${PROJECT_NAME}_lib
#  PRIVATE "TOPORVIZ_BUILDING_DLL"
#  PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY"
#)
#
#
#
## Install libraries and executables
#install(TARGETS
#  ${PROJECT_NAME}_lib
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)
#install(DIRECTORY include/
#  DESTINATION include
#)
#
#
## Install plugin description file
#install(FILES
#  plugin_description.xml
#  DESTINATION share/${PROJECT_NAME}
#)
#
#
## Register the plugin
#pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
#
#



#################################################
##
##   INSTALL PYTHON FILES AND OTHER DIRECTORIES
##
##   NO! this should be managed by the setup.py file
#
## Install scripts
#install(PROGRAMS
#  scripts/python_topmap_interface.py
#  DESTINATION lib/${PROJECT_NAME}
#)
#
## Install directories
#foreach (dir launch images conf)
#    install(DIRECTORY ${dir}/
#        DESTINATION share/${PROJECT_NAME}/${dir}
#    )
#endforeach(dir)
#


###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(${srv_deps})
ament_export_dependencies(${rviz_deps})
ament_export_include_directories(include)
ament_export_libraries(${target_install_binaries})


if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
    ament_package()
endif()
#ament_package()
