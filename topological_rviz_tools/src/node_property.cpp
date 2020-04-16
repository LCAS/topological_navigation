#include "node_property.h"

namespace topological_rviz_tools
{

NodeProperty::NodeProperty(const QString& name,
			   const strands_navigation_msgs::TopologicalNode& default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value.name.c_str(), description, parent, changed_slot, this)
  , node_(default_value)
  , name_(default_value.name)
  , xy_tol_value_(default_value.xy_goal_tolerance)
  , yaw_tol_value_(default_value.yaw_goal_tolerance)
  , reset_value_(false)
{
  // manually connect the signals instead of using the constructor to do it.
  // Can't seem to get the connection to work if passing in the slot in the
  // constructor.
  connect(this, SIGNAL(changed()), this, SLOT(updateNodeName()));

  ros::NodeHandle nh;
  nameUpdate_ = nh.serviceClient<strands_navigation_msgs::UpdateNodeName>("/topological_map_manager/update_node_name", true);
  toleranceUpdate_ = nh.serviceClient<strands_navigation_msgs::UpdateNodeTolerance>("/topological_map_manager/update_node_tolerance", true);

  map_ = new rviz::StringProperty("Map", node_.map.c_str(), "", this);
  map_->setReadOnly(true);

  pointset_ = new rviz::StringProperty("Pointset", node_.pointset.c_str(), "", this);
  pointset_->setReadOnly(true);

  localise_ = new rviz::StringProperty("Localise by topic", node_.localise_by_topic.c_str(), "", this);
  localise_->setReadOnly(true);

  yaw_tolerance_ = new rviz::FloatProperty("Yaw Tolerance", node_.yaw_goal_tolerance,
					   "The robot is facing the right direction if the"
					   " difference between the current yaw and the node's"
					   " orientation is less than this value.",
					   this, SLOT(updateYawTolerance()), this);
  xy_tolerance_ = new rviz::FloatProperty("XY Tolerance", node_.xy_goal_tolerance,
					  "The robot is at the goal if the difference"
					  " between its current position and the node's"
					  " position is less than this value.",
					  this, SLOT(updateXYTolerance()), this);

  ros::ServiceClient tagService_ = nh.serviceClient<strands_navigation_msgs::GetNodeTags>("/topological_map_manager/get_node_tags", true);
  strands_navigation_msgs::GetNodeTags srv;
  srv.request.node_name = name_.c_str();
  std::vector<std::string> node_tags;
  if (tagService_.call(srv)) {
    if (srv.response.success) {
      node_tags = srv.response.tags;
    } else {
      ROS_WARN("Failed to get tags for node %s", name_.c_str());
    }
  } else {
    ROS_WARN("Failed to get response from service to get tags for node %s", name_.c_str());
  }
  tag_controller_ = new TagController("Tags", node_tags, "", this);
  if (node_tags.size() == 0) {
    tag_controller_->setHidden(true);
  }

  pose_ = new PoseProperty("Pose", node_.pose, "", this);
  edge_controller_ = new EdgeController("Edges", node_.edges, "", this);
}

NodeProperty::~NodeProperty()
{
  delete map_;
  delete pointset_;
  delete yaw_tolerance_;
  delete xy_tolerance_;
  delete pose_;
  delete edge_controller_;
}

void NodeProperty::updateYawTolerance(){
  if (reset_value_){ // this function gets called when we reset a value when the service call fails, so ignore that.
    reset_value_ = false;
    return;
  }

  strands_navigation_msgs::UpdateNodeTolerance srv;
  srv.request.node_name = name_;
  srv.request.yaw_tolerance = yaw_tolerance_->getFloat();
  srv.request.xy_tolerance = xy_tolerance_->getFloat();
  
  if (toleranceUpdate_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Successfully updated yaw tolerance for node %s to %f", name_.c_str(), srv.request.yaw_tolerance);
      Q_EMIT nodeModified(this);
      yaw_tol_value_ = yaw_tolerance_->getFloat();
    } else {
      ROS_INFO("Failed to update yaw tolerance for node %s: %s", name_.c_str(), srv.response.message.c_str());
      reset_value_ = true;
      yaw_tolerance_->setValue(yaw_tol_value_);
    }
  } else {
    ROS_WARN("Failed to get response from service to update yaw tolerance for node %s", name_.c_str());
    reset_value_ = true;
    yaw_tolerance_->setValue(yaw_tol_value_);
  }
}

void NodeProperty::updateXYTolerance(){
  if (reset_value_){ // this function gets called when we reset a value when the service call fails, so ignore that.
    reset_value_ = false;
    return;
  }

  strands_navigation_msgs::UpdateNodeTolerance srv;
  srv.request.node_name = name_;
  srv.request.yaw_tolerance = yaw_tolerance_->getFloat();
  srv.request.xy_tolerance = xy_tolerance_->getFloat();
  
  if (toleranceUpdate_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Successfully updated tolerance for node %s to %f", name_.c_str(), srv.request.xy_tolerance);
      Q_EMIT nodeModified(this);
      xy_tol_value_ = xy_tolerance_->getFloat();
    } else {
      ROS_INFO("Failed to update xy tolerance of %s: %s", name_.c_str(), srv.response.message.c_str());
      reset_value_ = true;
      xy_tolerance_->setValue(xy_tol_value_);
    }
  } else {
    ROS_WARN("Failed to get response from service to update xy tolerance for node %s", name_.c_str());
    reset_value_ = true;
    xy_tolerance_->setValue(xy_tol_value_);
  }
}

void NodeProperty::updateNodeName(){
  if (reset_value_){ // this function gets called when we reset a value when the service call fails, so ignore that.
    reset_value_ = false;
    return;
  }

  strands_navigation_msgs::UpdateNodeName srv;
  srv.request.node_name = name_;
  srv.request.new_name = this->getValue().toString().toStdString().c_str();
  
  if (nameUpdate_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Successfully updated node name %s to %s", name_.c_str(), srv.request.new_name.c_str());
      Q_EMIT nodeModified(this);
      name_ = getValue().toString().toStdString();
    } else {
      ROS_INFO("Failed to update node name of %s to %s: %s", name_.c_str(), srv.request.new_name.c_str(), srv.response.message.c_str());
      reset_value_ = true;
      setValue(QString::fromStdString(name_));
    }
  } else {
    ROS_WARN("Failed to get response from service to update node %s", name_.c_str());
    reset_value_ = true;
    setValue(QString::fromStdString(name_));
  }
  name_ = this->getValue().toString().toStdString();
}

void NodeProperty::nodePropertyUpdated(){
  Q_EMIT nodeModified(this);
}

} // end namespace topological_rviz_tools
