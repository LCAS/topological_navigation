#include "edge_property.h"

namespace topological_rviz_tools
{

EdgeProperty::EdgeProperty(const QString& name,
			   const topological_navigation_msgs::Edge& default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value.edge_id.c_str(), description, parent, changed_slot, receiver)
  , edge_(default_value)
  , action_value_(default_value.action)
  , topvel_value_(default_value.top_vel)
  , reset_value_(false)
{
  ros::NodeHandle nh;
  edgeUpdate_ = nh.serviceClient<topological_navigation_msgs::UpdateEdgeLegacy>("/topological_map_manager/update_edge", true);
  setReadOnly(true);
  edge_id_ = new rviz::StringProperty("Edge ID", edge_.edge_id.c_str(), "", this);
  edge_id_->setReadOnly(true);
  node_ = new rviz::StringProperty("Node", edge_.node.c_str(), "", this);
  node_->setReadOnly(true);
  action_ = new rviz::StringProperty("Action", edge_.action.c_str(), "", this, SLOT(updateAction()), this);
  map_2d_ = new rviz::StringProperty("Map 2D", edge_.map_2d.c_str(), "", this);
  map_2d_->setReadOnly(true);
  top_vel_ = new rviz::FloatProperty("Top vel", edge_.top_vel, "", this, SLOT(updateTopvel()), this);
  inflation_radius_ = new rviz::FloatProperty("Inflation radius", edge_.inflation_radius, "", this);
  inflation_radius_->setReadOnly(true);
}

void EdgeProperty::updateTopvel(){
  if (reset_value_){ // this function gets called when we reset a value when the service call fails, so ignore that.
    reset_value_ = false;
    return;
  }

  topological_navigation_msgs::UpdateEdgeLegacy srv;
  srv.request.edge_id = edge_id_->getStdString().c_str();
  srv.request.top_vel = top_vel_->getFloat();
  srv.request.action = action_->getStdString().c_str();
  
  if (edgeUpdate_.call(srv)) {
    if (srv.response.success) {
      RCLCPP_INFO("Successfully updated edge %s topvel to %f", edge_id_->getStdString().c_str(), srv.request.top_vel);
      Q_EMIT edgeModified();
      topvel_value_ = top_vel_->getFloat();
    } else {
      RCLCPP_INFO("Failed to update xy tolerance of %s: %s", edge_id_->getStdString().c_str(), srv.response.message.c_str());
      reset_value_ = true;
      top_vel_->setValue(topvel_value_);
    }
  } else {
    RCLCPP_WARN("Failed to get response from service to update xy tolerance for node %s", edge_id_->getStdString().c_str());
    reset_value_ = true;
    top_vel_->setValue(topvel_value_);
  }

}

void EdgeProperty::updateAction(){
  if (reset_value_){ // this function gets called when we reset a value when the service call fails, so ignore that.
    reset_value_ = false;
    return;
  }

  topological_navigation_msgs::UpdateEdgeLegacy srv;
  srv.request.edge_id = edge_id_->getStdString().c_str();
  srv.request.top_vel = top_vel_->getFloat();
  srv.request.action = action_->getStdString().c_str();
  
  if (edgeUpdate_.call(srv)) {
    if (srv.response.success) {
      RCLCPP_INFO("Successfully updated edge %s action to %s", edge_id_->getStdString().c_str(), srv.request.action.c_str());
      Q_EMIT edgeModified();
      action_value_ = action_->getStdString();
    } else {
      RCLCPP_INFO("Failed to update edge action of %s: %s", edge_id_->getStdString().c_str(), srv.response.message.c_str());
      reset_value_ = true;
      action_->setValue(QString::fromStdString(action_value_));
    }
  } else {
    RCLCPP_WARN("Failed to get response from service to update action for edge %s", edge_id_->getStdString().c_str());
    reset_value_ = true;
    action_->setValue(QString::fromStdString(action_value_));
  }
}
  
EdgeProperty::~EdgeProperty()
{
  delete edge_id_;
  delete node_;
  delete action_;
  delete map_2d_;
  delete top_vel_;
  delete inflation_radius_;
}

} // end namespace topological_rviz_tools
