#include "tag_property.h"

namespace topological_rviz_tools
{

TagProperty::TagProperty(const QString& name,
			 const QString& default_value,
			 const QString& description,
			 const QString& node_name,
			 Property* parent,
			 const char *changed_slot,
			 QObject* receiver)
  : rviz::StringProperty(name, default_value, description, parent, changed_slot, receiver)
  , tag_value_(default_value.toStdString())
  , reset_value_(false)
  , node_name_(node_name.toStdString())
{
  connect(this, SIGNAL(changed()), this, SLOT(updateTag()));
  ros::NodeHandle nh;
  tagUpdate_ = nh.serviceClient<topological_navigation_msgs::ModifyTag>("/topological_map_manager/modify_node_tags", true);
}

void TagProperty::updateTag(){
  if (reset_value_){ // this function gets called when we reset a value when the service call fails, so ignore that.
    reset_value_ = false;
    return;
  }

  topological_navigation_msgs::ModifyTag srv;
  srv.request.tag = tag_value_.c_str();
  srv.request.new_tag = getString().toStdString().c_str();
  srv.request.node.push_back(node_name_);
  
  if (tagUpdate_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Successfully updated tag %s to %s", srv.request.tag.c_str(), srv.request.new_tag.c_str());
      Q_EMIT tagModified();
      tag_value_ = getString().toStdString();
    } else {
      ROS_INFO("Failed to update tag %s: %s", srv.request.tag.c_str(), srv.response.meta.c_str());
      reset_value_ = true;
      setValue(QString::fromStdString(tag_value_));
    }
  } else {
    ROS_WARN("Failed to get response from service to update tag %s", srv.request.tag.c_str());
    reset_value_ = true;
    setValue(QString::fromStdString(tag_value_));
  }
}

TagProperty::~TagProperty()
{
}

} // end namespace topological_rviz_tools
