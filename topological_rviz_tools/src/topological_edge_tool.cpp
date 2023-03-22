#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rclcpp/logging.hpp>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "topological_edge_tool.h"

namespace topological_rviz_tools
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
TopmapEdgeTool::TopmapEdgeTool()
  : noClick_(true)
{
  shortcut_key_ = 'e';

  // Initialise the marker with some defaults
  edgeMarker_.header.frame_id = "map";
  edgeMarker_.ns = "edge_tool_markers";
  edgeMarker_.id = 0;
  edgeMarker_.type = visualization_msgs::Marker::ARROW;
  edgeMarker_.scale.x = 0.1;
  edgeMarker_.scale.y = 0.15;
  edgeMarker_.scale.z = 0.2;
  edgeMarker_.color.a = 1.0; // Don't forget to set the alpha!
  edgeMarker_.color.r = 0.0;
  edgeMarker_.color.g = 1.0;
  edgeMarker_.color.b = 0.0;
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
TopmapEdgeTool::~TopmapEdgeTool()
{
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void TopmapEdgeTool::onInitialize()
{
  ros::NodeHandle nh;
  ros::Rate r(10);
  while(!ros::service::exists("/topmap_interface/add_edge", true))
  {
    r.sleep();
    RCLCPP_INFO("Waiting for add_edge service\n");
  }
  addEdgeSrv_ = nh.serviceClient<topological_navigation_msgs::AddEdgeRviz>("/topmap_interface/add_edge", true);
  markerPub_ = nh.advertise<visualization_msgs::Marker>("edge_tool_marker", 0);
  update_map_ = nh.advertise<std_msgs::Time>("/update_map", 5);
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void TopmapEdgeTool::activate()
{
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
void TopmapEdgeTool::deactivate()
{
  noClick_ = true;
  edgeMarker_.action = visualization_msgs::Marker::DELETE;
  edgeMarker_.points.clear();
  edgeMarker_.header.stamp = ros::Time();
  markerPub_.publish(edgeMarker_);
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing.
int TopmapEdgeTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if(rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection))
  {
    bool left = event.leftDown();
    bool right = event.rightDown();

    geometry_msgs::Pose event_pose = geometry_msgs::Pose();
    event_pose.position.x = intersection.x;
    event_pose.position.y = intersection.y;
    event_pose.position.z = intersection.z;

    if (edgeMarker_.points.size() > 1) {
      // Remove the second pose in points vector, with the previous pose of the
      // mouse.
      edgeMarker_.points.pop_back();
    }

    if (!noClick_){
      // If the first click happened, then we update the endpoint of the arrow each time the mouse moves.
      edgeMarker_.points.push_back(event_pose.position);
      edgeMarker_.header.stamp = ros::Time();
      edgeMarker_.action = visualization_msgs::Marker::ADD;
      markerPub_.publish(edgeMarker_);
    }

    if (right || left){
      if (noClick_){
	firstClick_ = event_pose;
	edgeMarker_.points.push_back(event_pose.position);
	noClick_ = false;
      } else {
	// On the second click, send the edge to the service to be added to the
	// map, and then reset the poses.
	topological_navigation_msgs::AddEdgeRviz srv;
	srv.request.first = firstClick_;
	srv.request.second = event_pose;
	srv.request.max_distance = 5.0; // be relatively accurate with clicks
	// if left clicked, add bidirectional edge
	srv.request.bidirectional = right ? false : true;

	if (addEdgeSrv_.call(srv)){
	  if (srv.response.success) {
	    RCLCPP_INFO("Successfully added edge: %s", srv.response.message.c_str());
	    std_msgs::Time t;
	    t.data = ros::Time::now();
	    update_map_.publish(t);
	  } else {
	    RCLCPP_INFO("Failed to add edge: %s", srv.response.message.c_str());
	  }
	} else {
	  RCLCPP_WARN("Failed to add edge: %s", srv.response.message.c_str());
	}
	edgeMarker_.action = visualization_msgs::Marker::DELETE;
	edgeMarker_.points.clear();
	edgeMarker_.header.stamp = ros::Time();
	markerPub_.publish(edgeMarker_);
	noClick_ = true;
      }
      return Render;
    }
  }

  return Render;
}
} // end namespace topological_rviz_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(topological_rviz_tools::TopmapEdgeTool,rviz::Tool)
