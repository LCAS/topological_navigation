#ifndef NODE_PROPERTY_H
#define NODE_PROPERTY_H

#include "ros/ros.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/float_property.h"
#include "topological_navigation_msgs/TopologicalNode.h"
#include "topological_navigation_msgs/GetNodeTags.h"
#include "topological_navigation_msgs/UpdateNodeName.h"
#include "topological_navigation_msgs/UpdateNodeTolerance.h"
#include "pose_property.h"
#include "edge_controller.h"
#include "tag_controller.h"

namespace topological_rviz_tools
{

class TagController;

/** @brief Property specialized to provide getter for booleans. */
class NodeProperty: public rviz::Property
{
Q_OBJECT
public:
  NodeProperty(const QString& name = QString(),
               const topological_navigation_msgs::TopologicalNode& default_value = topological_navigation_msgs::TopologicalNode(),
               const QString& description = QString(),
               Property* parent = 0,
               const char *changed_slot = 0,
               QObject* receiver = 0);

  virtual ~NodeProperty();

  std::string getNodeName() { return name_; }
  TagController* getTagController() { return tag_controller_; }
public Q_SLOTS:
  void updateYawTolerance();
  void updateXYTolerance();
  void updateNodeName();
  void nodePropertyUpdated();

Q_SIGNALS:
void nodeModified(Property* node);

private:
  const topological_navigation_msgs::TopologicalNode& node_;
  
  ros::ServiceClient nameUpdate_;
  ros::ServiceClient toleranceUpdate_;

  rviz::StringProperty* node_name_;
  rviz::StringProperty* map_;
  rviz::StringProperty* pointset_;
  rviz::StringProperty* localise_;
  rviz::FloatProperty* yaw_tolerance_;
  rviz::FloatProperty* xy_tolerance_;
  // Store the name so that we can refer to it to change the node name in the
  // map - once it changes in the property we won't know its previous value
  // otherwise.
  std::string name_;
  // Also store the editable values, in case the service call fails. We then
  // reset the property value to its original value.
  float xy_tol_value_;
  float yaw_tol_value_;
  bool reset_value_;
  PoseProperty* pose_;
  EdgeController* edge_controller_;
  TagController* tag_controller_;
};

} // end namespace topological_rviz_tools

#endif // NODE_PROPERTY_H
