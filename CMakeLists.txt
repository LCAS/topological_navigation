cmake_minimum_required(VERSION 2.8.3)
project(topological_utils)

find_package(catkin REQUIRED rospy roscpp std_msgs geometry_msgs message_generation mongodb_store visualization_msgs)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_service_files(
    FILES
    NodeMetadata.srv
)
generate_messages(
DEPENDENCIES
    std_msgs 
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topological_utils
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(topological_utils
#   src/${PROJECT_NAME}/topological_utils.cpp
# )



## Declare a cpp executable
# add_executable(topological_utils_node src/topological_utils_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(topological_utils_node topological_utils_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(topological_utils_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/add_content.py
   scripts/add_edge.py
   scripts/add_node.py
   scripts/crop_map.py
   scripts/dummy_topological_navigation.py
   scripts/insert_map.py
   scripts/joy_add_node.py
   scripts/joy_add_waypoint.py
   scripts/list_maps
   scripts/load_yaml_map.py
   scripts/load_json_map.py
   #scripts/map_collection_change.py
   scripts/map_export.py
   scripts/map_to_json.py
   scripts/map_to_yaml.py
   scripts/migrate.py
   scripts/node_rm.py
   scripts/rm_map_from_db.py
   scripts/tmap_from_waypoints.py
   scripts/tmap_to_yaml.py
   scripts/topological_map_update.py
   scripts/waypoints_to_yaml_tmap.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY support
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_topological_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
