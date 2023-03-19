cmake_minimum_required(VERSION 3.5)
project(topological_navigation)

############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(topological_navigation_msgs REQUIRED)



#########################
#
#   PYTHON COMPILATION
#

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



#########################
#
#   ROS2 TESTING
#


#install(DIRECTORY
#  tests
#  DESTINATION bin/${PROKECT_NAME}
#  USE_SOURCE_PERMISSIONS
#)

#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  #add_rostest(tests/travel_time_estimator.test)
#  add_rostest(tests/map_manager.test)
#  #add_rostest(tests/navigation_scenarios.test)
#endif()




###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()
