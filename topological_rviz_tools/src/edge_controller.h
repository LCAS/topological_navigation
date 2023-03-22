#ifndef TOPMAP_EDGE_CONTROLLER_H
#define TOPMAP_EDGE_CONTROLLER_H

#include <string>

#include <QCursor>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include "rviz_common/config.hpp"
#include "rviz_common/display_context.h"
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

#include "topological_navigation_msgs/Edge.h"
#include "geometry_msgs/Pose.h"

#include "edge_property.h"

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
