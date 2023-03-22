#ifndef TAG_PROPERTY_H
#define TAG_PROPERTY_H

#include "rclcpp/rclcpp.hpp"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "topological_navigation_msgs/ModifyTag.h"

namespace topological_rviz_tools
{

class TagProperty: public rviz::StringProperty
{
Q_OBJECT
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
};

} // end namespace topological_rviz_tools

#endif // TAG_PROPERTY_H
