#ifndef TOPMAP_NODE_CONTROLLER_H
#define TOPMAP_NODE_CONTROLLER_H

#include <algorithm>
#include <string>
#include <utility>

#include <QCursor>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include "ros/ros.h"

#include "rviz/config.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/load_resource.h"
#include "rviz/ogre_helpers/render_system.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/window_manager_interface.h"

#include "topological_navigation_msgs/TopologicalMap.h"
#include "topological_navigation_msgs/TopologicalNode.h"

#include "node_property.h"

class QKeyEvent;

namespace topological_rviz_tools {
class NodeController: public rviz::Property
{
Q_OBJECT
public:
  NodeController();
  virtual ~NodeController();

  /** @brief Do all setup that can't be done in the constructor.
   *
   *
   * Calls onInitialize() just before returning. */
  void initialize();

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  // required by something that initialises this class
  QString formatClassId(const QString& class_id);

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const { return class_id_; }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id ) { class_id_ = class_id; }

Q_SIGNALS:
  void configChanged();
  void childModified();

private Q_SLOTS:
  void updateModifiedNode(Property* node);

protected:
  /** @brief Do subclass-specific initialization.  Called by
   * NodeController::initialize after context_ and camera_ are set.
   * Default implementation does nothing. */
  virtual void onInitialize() {}

  void addModifiedChild(rviz::Property* modifiedChild){ modifiedChildren_.push_back(modifiedChild); }

private:
  void topmapCallback(const topological_navigation_msgs::TopologicalMap::ConstPtr& msg);

  QString class_id_;
  ros::Subscriber top_sub_;
  std::vector<rviz::Property*> modifiedChildren_;

  /* bool sortNodes(topological_navigation_msgs::TopologicalNode a, topological_navigation_msgs::TopologicalNode b) { return a.name.compare(b.name) < 0; } */

  struct NodeSorter {

    bool operator() (topological_navigation_msgs::TopologicalNode a,
                     topological_navigation_msgs::TopologicalNode b) {
      std::string an = a.name;
      std::string bn = b.name;
      std::transform(an.begin(), an.end(), an.begin(), ::tolower);
      std::transform(bn.begin(), bn.end(), bn.begin(), ::tolower);
      return an.compare(bn) < 0;
    }
  } nodeSort;
};

} // end namespace topological_rviz_tools

#endif // TOPMAP_NODE_CONTROLLER_H
