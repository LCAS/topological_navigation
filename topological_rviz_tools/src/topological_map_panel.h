#ifndef TOPMAP_PANEL_H
#define TOPMAP_PANEL_H

#include <cstdio>

#include "rviz/panel.h"
#include "topmap_manager.h"
#include "tag_property.h"
#include "edge_property.h"
#include "node_property.h"
#include "ros/ros.h"

#include "rviz/properties/property_tree_widget.h"
#include "std_msgs/Time.h"

#include "topological_navigation_msgs/AddTag.h"
#include "topological_navigation_msgs/AddEdge.h"
#include "topological_navigation_msgs/RmvNode.h"

class QComboBox;
class QMessageBox;
class QModelIndex;
class QPushButton;
class QInputDialog;

namespace topological_rviz_tools {
/**
 * @brief Panel for choosing the view controller and saving and restoring
 * viewpoints.
 */
class TopologicalMapPanel: public rviz::Panel
{
Q_OBJECT
public:
  TopologicalMapPanel(QWidget* parent = 0);
  virtual ~TopologicalMapPanel() {}

  /** @brief Overridden from TopologicalMapPanel.  Just calls setMan() with vis_manager_->getTopmapManager(). */
  virtual void onInitialize();

  /** @brief Set the TopmapManager which this panel should display and edit.
   *
   * If this TopologicalMapPanel is to be used with a TopmapManager other than
   * the one in the TopmapManager sent in through
   * TopologicalMapPanel::initialize(), either TopologicalMapPanelel::initialize() must not be
   * called or setTopmapManager() must be called after
   * TopologicalMapPanel::initialize(). */
  void setTopmapManager(TopmapManager* topmap_man);

  /** @brief Returns the current TopmapManager. */
  TopmapManager* getTopmapManager() const { return topmap_man_; }

  /** @brief Load configuration data, specifically the PropertyTreeWidget view settings. */
  virtual void load(const rviz::Config& config);

  /** @brief Save configuration data, specifically the PropertyTreeWidget view settings. */
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void onDeleteClicked();
  void onAddTagClicked();
  void renameSelected();
  void onCurrentChanged();
  void updateTopMap();
private:
  ros::ServiceClient delNodeSrv_;
  ros::ServiceClient delTagSrv_;
  ros::ServiceClient delEdgeSrv_;
  ros::ServiceClient addTagSrv_;
  ros::Publisher update_map_;

  TopmapManager* topmap_man_;
  rviz::PropertyTreeWidget* properties_view_;
};

} // namespace topological_rviz_tools

#endif // TOPMAP_PANEL_H
