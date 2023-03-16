cmake_minimum_required(VERSION 3.5)
project(topological_rviz_tools)


####################################
#
#   DEFINE COMPILER SPECIFICATION
#


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# Define dependencies for messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(topological_navigation_msgs REQUIRED)
set(srv_deps
  std_msgs
  geometry_msgs
  topological_navigation_msgs
)

# Define dependencies for RViz Plugins
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
set(rviz_deps
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
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


# Declare files to be included under the plugin
set(rviz_files
  src/topological_edge_tool.cpp
  src/topological_node_tool.cpp
  src/topological_map_panel.cpp
  src/node_controller.cpp
  src/topmap_manager.cpp
  src/node_property.cpp
  src/edge_property.cpp
  src/pose_property.cpp
  src/edge_controller.cpp
  src/tag_controller.cpp
  src/tag_property.cpp
)


# Construct the library
add_library(topological_rviz_tools_lib SHARED ${rviz_files})
ament_target_dependencies(topological_rviz_tools_lib ${rviz_deps})
target_compile_definitions(topological_rviz_tools_lib
  PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY"
)
set(target_install_binaries topological_rviz_tools_lib)


# Register the plugin
pluginlib_export_plugin_description_file(rviz_default_plugins plugin_description.xml)

install(
  TARGETS ${target_install_binaries}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)



################################################
#
#   INSTALL PYTHON FILES AND OTHER DIRECTORIES
#

# Install scripts
install(PROGRAMS
  scripts/python_topmap_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
foreach (dir launch images conf)
    install(DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir}
    )
endforeach(dir)



###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_common)
ament_export_dependencies(rviz_default_plugins)
ament_export_dependencies(rviz_ogre_vendor)
ament_export_dependencies(rviz_rendering)
ament_export_dependencies(rviz_rendering_plugins)
ament_export_include_directories(include)
ament_export_libraries(${target_install_binaries})
ament_package()
