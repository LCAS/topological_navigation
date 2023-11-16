#ifndef TOPMAP_EDGE_CONTROLLER_H
#define TOPMAP_EDGE_CONTROLLER_H

#include <string>

#include <QCursor>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

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
#include "topological_navigation_msgs/msg/edge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "edge_property.hpp"

class QKeyEvent;

namespace topological_rviz_tools {
class EdgeController: public rviz::Property
{
Q_OBJECT
public:
  EdgeController(const QString& name = QString(),
		 const std::vector<topological_navigation_msgs::Edge>& default_values = std::vector<topological_navigation_msgs::Edge>(),
		 const QString& description = QString(),
		 rviz::Property* parent = 0,
		 const char *changed_slot = 0,
		 QObject* receiver = 0);
  virtual ~EdgeController();

  void initialize();

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  // required by something that initialises this class
  QString formatClassId(const QString& class_id);

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const { return class_id_; }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id ) { class_id_ = class_id; }

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  bool addEdge(const topological_navigation_msgs::Edge& edge);
Q_SIGNALS:
  void configChanged();

protected:
  /** @brief Do subclass-specific initialization.  Called by
   * EdgeController::initialize after context_ and camera_ are set.
   * Default implementation does nothing. */
  virtual void onInitialize() {}
private:
  QString class_id_;
  std::vector<EdgeProperty*> edges_;
};

} // end namespace topological_rviz_tools

#endif // TOPMAP_EDGE_CONTROLLER_H
