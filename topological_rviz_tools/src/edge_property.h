#ifndef EDGE_PROPERTY_H
#define EDGE_PROPERTY_H

#include "rclcpp/rclcpp.hpp"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/float_property.h"
#include "topological_navigation_msgs/Edge.h"
#include "topological_navigation_msgs/UpdateEdgeLegacy.h"

namespace topological_rviz_tools
{

/** @brief Property specialized to provide getter for booleans. */
class EdgeProperty: public rviz::Property
{
Q_OBJECT
public:
  EdgeProperty(const QString& name = QString(),
               const topological_navigation_msgs::Edge& default_value = topological_navigation_msgs::Edge(),
               const QString& description = QString(),
               Property* parent = 0,
               const char *changed_slot = 0,
               QObject* receiver = 0);

  virtual ~EdgeProperty();

  std::string getEdgeId() { return edge_id_->getString().toStdString(); }
public Q_SLOTS:
  void updateAction();
  void updateTopvel();

Q_SIGNALS:
  void edgeModified();

private:
  const topological_navigation_msgs::Edge& edge_;
  
  // keep track of changing values to ensure that they are redisplayed correctly
  // when we fail to update.
  std::string action_value_;
  float topvel_value_;

  bool reset_value_;
  ros::ServiceClient edgeUpdate_;

  rviz::StringProperty* edge_id_;
  rviz::StringProperty* node_;
  rviz::StringProperty* action_;
  rviz::StringProperty* map_2d_;
  rviz::FloatProperty* inflation_radius_;
  rviz::FloatProperty* top_vel_;
};

} // end namespace topological_rviz_tools

#endif // EDGE_PROPERTY_H
