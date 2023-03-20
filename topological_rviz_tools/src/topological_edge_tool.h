#ifndef TOPMAP_EDGE_TOOL_H
#define TOPMAP_EDGE_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include "topological_navigation_msgs/AddEdgeRviz.h"
#include "std_msgs/Time.h"

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
class TopmapEdgeTool: public rviz::Tool
{
Q_OBJECT
public:
  TopmapEdgeTool();
  ~TopmapEdgeTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
private:
  ros::Publisher markerPub_;
  ros::Publisher update_map_;
  ros::ServiceClient addEdgeSrv_;
  bool noClick_; // true if nothing clicked yet
  geometry_msgs::Pose firstClick_;
  visualization_msgs::Marker edgeMarker_;

};
} // end namespace topological_rviz_tools

#endif // TOPMAP_EDGE_TOOL_H
