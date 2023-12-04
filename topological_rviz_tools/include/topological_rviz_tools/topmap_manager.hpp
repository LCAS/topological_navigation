#ifndef TOPMAP_MANAGER_H
#define TOPMAP_MANAGER_H

#include "node_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <sstream>

#include <QList>
#include <QObject>
#include <QStringList>

#include "rviz_common/display_context.hpp"
#include "rviz_common/factory/pluginlib_factory.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/render_panel.hpp"

namespace topological_rviz_tools
{

class TopmapManager: public QObject
{

public:
  TopmapManager(rviz_common::DisplayContext* context);
  ~TopmapManager();

  void initialize();

  void update(float wall_dt, float ros_dt);

  /** @brief Return the current NodeController in use for the main
   * RenderWindow. */
  NodeController* getController() const;

  NodeProperty* getCurrent() const;

  NodeController* create(const QString& type);

  int getNumViews() const;

  NodeController* getViewAt(int index) const;

  /** @brief Remove the given NodeController from the list and return
   * it.  If it is not in the list, NULL is returned and nothing
   * changes. */
  NodeController* take(NodeController* view);

  /** @brief Remove the NodeController at the given index from the
   * list and return it.  If the index is not valid, NULL is returned
   * and nothing changes. */
  NodeController* takeAt(int index);

  rviz_common::properties::PropertyTreeModel* getPropertyModel() { return property_model_; }

  void load(const rviz_common::Config& config);
  void save(rviz_common::Config config) const;

  /** @brief Make a copy of @a view_to_copy and install that as the new current NodeController. */
  void setCurrentFrom(NodeProperty* view_to_copy);

  /** @brief Return a copy of source, made by saving source to
   * a Config and instantiating and loading a new one from that. */
  NodeProperty* copy(NodeProperty* source);

  rviz_common::PluginlibFactory<NodeController>* getFactory() const { return factory_; }

  /** @brief Set the 3D view widget whose view will be controlled by
   * NodeController instances from by this TopmapManager. */
  void setRenderPanel(rviz_common::RenderPanel* render_panel);

  /** @brief Return the 3D view widget managed by this TopmapManager. */
  rviz_common::RenderPanel* getRenderPanel() const { return render_panel_; }

public Q_SLOTS:

  /** @brief Make a copy of the current NodeController and add it to the end of the list of saved views. */
  void copyCurrentToList();

  /** @brief Create a new view controller of the given type and set it
   * up to mimic and replace the previous current view. */
  void setCurrentNodeControllerType(const QString& new_class_id);

Q_SIGNALS:
  void configChanged();

  /** @brief Emitted just after the current view controller changes. */
  void currentChanged();

private Q_SLOTS:
  void onCurrentDestroyed(QObject* obj);

private:
  /** @brief Set @a new_current as current.
   * @param mimic_view If true, call new_current->mimic(previous), if false call new_current->transitionFrom(previous).
   *
   * This calls mimic() or transitionFrom() on the new controller,
   * deletes the previous controller (if one existed), and tells the
   * RenderPanel about the new controller. */
  void setCurrent(NodeProperty* new_current, bool mimic_view);

  rviz_common::DisplayContext* context_;
  NodeController* root_property_;
  rviz_common::properties::PropertyTreeModel* property_model_;
  rviz_common::PluginlibFactory<NodeController>* factory_;
  NodeProperty* current_;
  rviz_common::RenderPanel* render_panel_;
  rclcpp::Logger logger_{rclcpp::get_logger("rviz2")};
};

}
#endif
