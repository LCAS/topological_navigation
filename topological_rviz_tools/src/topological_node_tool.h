#ifndef TOPMAP_NODE_TOOL_H
#define TOPMAP_NODE_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Time.h"
#include "strands_navigation_msgs/AddNode.h"

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace topological_rviz_tools
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class TopmapNodeTool: public rviz::Tool
{
Q_OBJECT
public:
  TopmapNodeTool();
  ~TopmapNodeTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
private:
  ros::ServiceClient addNodeSrv_;
  ros::Publisher update_map_;
};
} // end namespace topological_rviz_tools

#endif // TOPMAP_NODE_TOOL_H
