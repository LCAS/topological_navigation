cmake_minimum_required(VERSION 3.5)
project(topological_rviz_tools)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wno-deprecated-declarations")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(topological_navigation_msgs REQUIRED)

# Define dependencies for RViz Plugins
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(pluginlib REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(topological_rviz_tools_headers_to_moc
  include/${PROJECT_NAME}/edge_controller.hpp  
  include/${PROJECT_NAME}/node_controller.hpp  
  include/${PROJECT_NAME}/node_property.hpp  
  include/${PROJECT_NAME}/pose_property.hpp  
  include/${PROJECT_NAME}/tag_controller.hpp  
  include/${PROJECT_NAME}/tag_property.hpp 
  include/${PROJECT_NAME}/topmap_manager.hpp 
  include/${PROJECT_NAME}/topological_edge_tool.hpp 
  include/${PROJECT_NAME}/topological_map_panel.hpp 
  include/${PROJECT_NAME}/topological_node_tool.hpp
)

foreach(header "${vision_msgs_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(topological_rviz_tools_headers_to_moc "${header}")
endforeach()

set(topological_rviz_tools_source_files
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

add_library(${PROJECT_NAME} SHARED ${topological_rviz_tools_source_files} 
                      ${topological_rviz_tools_headers_to_moc})

target_include_directories(${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
      ${Qt5Widgets_INCLUDE_DIRS} )

target_link_libraries(${PROJECT_NAME} PUBLIC
            rviz_ogre_vendor::OgreMain
            rviz_ogre_vendor::OgreOverlay
            rviz_common::rviz_common 
)

target_compile_definitions(${PROJECT_NAME}
                 PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")


ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        rclcpp
        rclcpp_components
        rviz_common
        rviz_default_plugins
        rviz_ogre_vendor
        rviz_rendering
        topological_navigation_msgs
        pluginlib)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories("include/${PROJECT_NAME}")


ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    rviz_common
    rviz_default_plugins
    rviz_ogre_vendor
    rviz_rendering
    topological_navigation_msgs
    pluginlib)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
    DESTINATION "share/${PROJECT_NAME}"
  )
  
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )


ament_package()