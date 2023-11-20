#include "topological_rviz_tools/tag_controller.hpp"

namespace topological_rviz_tools
{
TagController::TagController(const QString& name,
			     const std::vector<std::string>& default_values,
			     const QString& description,
			     NodeProperty* parent,
			     const char *changed_slot,
			     QObject* receiver)
  : rviz_common::properties::Property(name, "", description, parent, changed_slot, receiver)
{
  for (int i = 0; i < default_values.size(); i++) {
    TagProperty* newTag = new TagProperty("Tag", QString(QString::fromStdString(default_values[i])), "", QString::fromStdString(parent->getNodeName()));
    addChild(newTag);
    connect(newTag, SIGNAL(tagModified()), parent, SLOT(nodePropertyUpdated()));
  }
}

void TagController::initialize()
{

  std::stringstream ss;
  static int count = 0;
  ss << "TagController" << count++;
}

TagController::~TagController()
{
  for (;numChildren() != 0;) {
    delete takeChildAt(0);
  }
}

QString TagController::formatClassId(const QString& class_id)
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

void TagController::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void TagController::load(const rviz_common::Config& config)
{
  // // Load the name by hand.
  // QString name;
  // if(config.mapGetString("Name", &name))
  // {
  //   setName(name);
  // }
  // // Load all sub-properties the same way the base class does.
  // rviz_common::properties::Property::load(config);
}

void TagController::save(rviz_common::Config config) const
{
  // config.mapSetValue("Class", getClassId());
  // config.mapSetValue("Name", getName());

  // rviz_common::properties::Property::save(config);
}

} // end namespace topological_rviz_tools
