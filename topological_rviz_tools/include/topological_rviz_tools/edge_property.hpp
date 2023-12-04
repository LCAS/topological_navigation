#ifndef EDGE_PROPERTY_H
#define EDGE_PROPERTY_H

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "topological_navigation_msgs/msg/edge.hpp"
#include "topological_navigation_msgs/srv/update_edge_legacy.hpp"

namespace topological_rviz_tools
{

/** @brief Property specialized to provide getter for booleans. */
class EdgeProperty: public rviz_common::properties::Property
{
Q_OBJECT
public:
  EdgeProperty(const QString& name = QString(),
               const topological_navigation_msgs::msg::Edge& default_value = topological_navigation_msgs::msg::Edge(),
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
  const topological_navigation_msgs::msg::Edge& edge_;
  
  // keep track of changing values to ensure that they are redisplayed correctly
  // when we fail to update.
  std::string action_value_;
  float topvel_value_;

  bool reset_value_;
  ros::ServiceClient edgeUpdate_;

  rviz_common::properties::StringProperty* edge_id_;
  rviz_common::properties::StringProperty* node_;
  rviz_common::properties::StringProperty* action_;
  rviz_common::properties::StringProperty* map_2d_;
  rviz_common::properties::FloatProperty* inflation_radius_;
  rviz_common::properties::FloatProperty* top_vel_;
  rclcpp::Logger logger_{rclcpp::get_logger("rviz2")};
};

} // end namespace topological_rviz_tools

#endif // EDGE_PROPERTY_H
