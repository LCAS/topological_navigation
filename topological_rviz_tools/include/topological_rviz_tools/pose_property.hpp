#ifndef POSE_PROPERTY_H
#define POSE_PROPERTY_H

#include "rclcpp/rclcpp.hpp"
#include "topological_navigation_msgs/srv/add_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace topological_rviz_tools
{

/** @brief Property specialized to provide getter for booleans. */
class PoseProperty: public rviz_common::properties::Property
{

public:
 PoseProperty(const QString& name = QString(),
	      const geometry_msgs::msg::Pose& default_value = geometry_msgs::msg::Pose(),
	      const QString& description = QString(),
	      rviz_common::properties::Property* parent = 0,
	      const char *changed_slot = 0,
	      QObject* receiver = 0);

  virtual ~PoseProperty();

public Q_SLOTS:
  void positionUpdated();

Q_SIGNALS:
  void poseModified();

private:
  const geometry_msgs::msg::Pose& pose_;
  rviz_common::properties::StringProperty* orientation_;
  rviz_common::properties::FloatProperty* orientation_w_;
  rviz_common::properties::FloatProperty* orientation_x_;
  rviz_common::properties::FloatProperty* orientation_y_;
  rviz_common::properties::FloatProperty* orientation_z_;
  rviz_common::properties::StringProperty* position_;
  rviz_common::properties::FloatProperty* position_x_;
  rviz_common::properties::FloatProperty* position_y_;
  rviz_common::properties::FloatProperty* position_z_;
  rclcpp::Logger logger_{rclcpp::get_logger("rviz2")};

  ros::ServiceClient poseUpdate_;
};

} // end namespace topological_rviz_tools

#endif // POSE_PROPERTY_H
