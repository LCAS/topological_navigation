#include "edge_controller.h"


namespace topological_rviz_tools
{
EdgeController::EdgeController(const QString& name,
			       const std::vector<topological_navigation_msgs::Edge>& default_values,
			       const QString& description,
			       rviz::Property* parent,
			       const char *changed_slot,
			       QObject* receiver)
  : rviz::Property(name, "", description, parent, changed_slot, receiver)
{
  for (int i = 0; i < default_values.size(); i++) {
    // RCLCPP_INFO("ADDING EDGE %s", default_values[i].edge_id.c_str());
    EdgeProperty* newEdge = new EdgeProperty("Edge", default_values[i], "");
    addChild(newEdge);
    connect(newEdge, SIGNAL(edgeModified()), parent, SLOT(nodePropertyUpdated()));
  }
}

void EdgeController::initialize()
{

  std::stringstream ss;
  static int count = 0;
  ss << "EdgeControllerCamera" << count++;

  // Do subclass initialization.
  onInitialize();
}

EdgeController::~EdgeController()
{
  for (;numChildren() != 0;) {
    delete takeChildAt(0);
  }
}

QString EdgeController::formatClassId(const QString& class_id)
{
  QStringList id_parts = class_id.split("/");
  if(id_parts.size() != 2)
  {
    // Should never happen with pluginlib class ids, which are
    // formatted like "package_name/class_name".  Not worth crashing
    // over though.
    return class_id;
  }
  else
  {
    return id_parts[ 1 ] + " (" + id_parts[ 0 ] + ")";
  }
}

void EdgeController::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void EdgeController::load(const rviz::Config& config)
{
  // // Load the name by hand.
  // QString name;
  // if(config.mapGetString("Name", &name))
  // {
  //   setName(name);
  // }
  // // Load all sub-properties the same way the base class does.
  // rviz::Property::load(config);
}

void EdgeController::save(rviz::Config config) const
{
  // config.mapSetValue("Class", getClassId());
  // config.mapSetValue("Name", getName());

  // rviz::Property::save(config);
}

} // end namespace topological_rviz_tools
