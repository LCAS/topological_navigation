#ifndef TAG_PROPERTY_H
#define TAG_PROPERTY_H

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "topological_navigation_msgs/srv/modify_tag.hpp"

namespace topological_rviz_tools
{

class TagProperty: public rviz_common::properties::StringProperty
{

public:
  TagProperty(const QString& name = QString(),
	      const QString& default_value = QString(),
	      const QString& description = QString(),
              const QString& node_name = QString(),
	      Property* parent = 0,
              const char *changed_slot = 0,
              QObject* receiver = 0);


  virtual ~TagProperty();
  void addTag(const QString& tag);

public Q_SLOTS:
  void updateTag();

Q_SIGNALS:
  void tagModified();
private:
  ros::ServiceClient tagUpdate_;
  std::string tag_value_; // keep value so it's not lost if we fail to update
  bool reset_value_;
  std::string node_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("rviz2")};
};

} // end namespace topological_rviz_tools

#endif // TAG_PROPERTY_H
