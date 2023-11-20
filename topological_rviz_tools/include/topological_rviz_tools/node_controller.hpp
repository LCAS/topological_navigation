#ifndef TOPMAP_NODE_CONTROLLER_H
#define TOPMAP_NODE_CONTROLLER_H

#include <algorithm>
#include <string>
#include <utility>

#include <QCursor>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_rendering/render_system.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/window_manager_interface.hpp"

#include "topological_navigation_msgs/msg/topological_map.hpp"
#include "topological_navigation_msgs/msg/topological_node.hpp"

#include "node_property.hpp"

class QKeyEvent;

namespace topological_rviz_tools {
class NodeController: public rviz_common::properties::Property
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

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

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

  void addModifiedChild(rviz_common::properties::Property* modifiedChild){ modifiedChildren_.push_back(modifiedChild); }

private:
  void topmapCallback(const topological_navigation_msgs::TopologicalMap::ConstPtr& msg);

  QString class_id_;
  ros::Subscriber top_sub_;
  std::vector<rviz_common::properties::Property*> modifiedChildren_;
  rclcpp::Logger logger_{rclcpp::get_logger("rviz2")};

  /* bool sortNodes(topological_navigation_msgs::msg::TopologicalNode a, topological_navigation_msgs::msg::TopologicalNode b) { return a.name.compare(b.name) < 0; } */

  struct NodeSorter {

    bool operator() (topological_navigation_msgs::msg::TopologicalNode a,
                     topological_navigation_msgs::msg::TopologicalNode b) {
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
