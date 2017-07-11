#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "topological_node_tool.h"

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
TopmapNodeTool::TopmapNodeTool()
{
  shortcut_key_ = 'n';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
TopmapNodeTool::~TopmapNodeTool()
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
void TopmapNodeTool::onInitialize()
{
  
  ros::NodeHandle nh;
  ros::Rate r(10);
  while(!ros::service::exists("/topological_map_manager/add_topological_node", true))
  {
    r.sleep();
    ROS_INFO("Waiting for add_topological_node service\n");
  }

  addNodeSrv_ = nh.serviceClient<strands_navigation_msgs::AddNode>("/topological_map_manager/add_topological_node", true);
  update_map_ = nh.advertise<std_msgs::Time>("/update_map", 5);

}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void TopmapNodeTool::activate()
{
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
void TopmapNodeTool::deactivate()
{
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing.
int TopmapNodeTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if(rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection))
  {
    if (event.leftDown()){
      geometry_msgs::Pose clicked = geometry_msgs::Pose();
      clicked.position.x = intersection.x;
      clicked.position.y = intersection.y;
      clicked.position.z = intersection.z;
      // On the second click, send the edge to the service to be added to the
      // map, and then reset the poses.
      strands_navigation_msgs::AddNode srv;
      srv.request.pose = clicked;

      if (addNodeSrv_.call(srv)){
	if (srv.response.success) {
	  ROS_INFO("Successfully added node");
	  std_msgs::Time t;
	  t.data = ros::Time::now();
	  update_map_.publish(t);
	} else {
	  ROS_INFO("Failed to add node");
	}
      } else {
	ROS_WARN("Failed to connect to add node service");
      }
      return Render | Finished;
    }
    return Render;
  }
}

} // end namespace topological_rviz_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(topological_rviz_tools::TopmapNodeTool, rviz::Tool)
