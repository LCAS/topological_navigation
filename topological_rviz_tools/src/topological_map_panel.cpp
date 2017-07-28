#include "topological_map_panel.h"

#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QComboBox>

namespace topological_rviz_tools
{
TopologicalMapPanel::TopologicalMapPanel(QWidget* parent)
  : rviz::Panel(parent)
  , topmap_man_(NULL)
{
  properties_view_ = new rviz::PropertyTreeWidget();

  ros::NodeHandle nh;
  ros::Rate r(10);
  while(!ros::service::exists("/topological_map_manager/remove_topological_node", true))
  {
    r.sleep();
    ROS_INFO("Waiting for remove_topological_node service\n");
  }
  while(!ros::service::exists("/topological_map_manager/add_tag_to_node", true))
  {
    r.sleep();
    ROS_INFO("Waiting for add_tag_to_node service\n");
  }

  while(!ros::service::exists("/topological_map_manager/rm_tag_from_node", true))
  {
    r.sleep();
    ROS_INFO("Waiting for rm_tag_from_node service\n");
  }

  while(!ros::service::exists("/topological_map_manager/remove_edge", true))
  {
    r.sleep();
    ROS_INFO("Waiting for remove_edge service\n");
  }
  delNodeSrv_ = nh.serviceClient<strands_navigation_msgs::RmvNode>("/topological_map_manager/remove_topological_node", true);
  addTagSrv_ = nh.serviceClient<strands_navigation_msgs::AddTag>("/topological_map_manager/add_tag_to_node", true);
  delTagSrv_ = nh.serviceClient<strands_navigation_msgs::AddTag>("/topological_map_manager/rm_tag_from_node", true);
  delEdgeSrv_ = nh.serviceClient<strands_navigation_msgs::AddEdge>("/topological_map_manager/remove_edge", true);
  update_map_ = nh.advertise<std_msgs::Time>("/update_map", 5);

  QPushButton* add_tag_button = new QPushButton("Add tag");
  QPushButton* remove_button = new QPushButton("Remove");

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget(add_tag_button);
  button_layout->addWidget(remove_button);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0,0,0,0);
  main_layout->addWidget(properties_view_);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);

  connect(remove_button, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
  connect(add_tag_button, SIGNAL(clicked()), this, SLOT(onAddTagClicked()));
  // connect(properties_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(setCurrentViewFromIndex(const QModelIndex&)));
  // connect(properties_view_, SIGNAL(activated(const QModelIndex&)), this, SLOT(setCurrentViewFromIndex(const QModelIndex&)));
}

void TopologicalMapPanel::onInitialize()
{
  ROS_INFO("Topmapmanel::OnInitialise");
  setTopmapManager(new TopmapManager(NULL));
  // connect topological map update caller to the nodecontroller so we can
  // update on changes to the nodes
  connect(topmap_man_->getController(), SIGNAL(childModified()), this, SLOT(updateTopMap()));
}

void TopologicalMapPanel::setTopmapManager(TopmapManager* topmap_man)
{
  ROS_INFO("Setting model");
  properties_view_->setModel(topmap_man->getPropertyModel());
  topmap_man_ = topmap_man;

  // connect(camera_type_selector_, SIGNAL(activated(int)), this, SLOT(onTypeSelectorChanged(int)));
  // connect(topmap_man_, SIGNAL(currentChanged()), this, SLOT(onCurrentChanged()));
  // onCurrentChanged();
}

void TopologicalMapPanel::onDeleteClicked()
{
  QList<NodeProperty*> nodes_to_delete = properties_view_->getSelectedObjects<NodeProperty>();
  QList<TagProperty*> tags_to_delete = properties_view_->getSelectedObjects<TagProperty>();
  QList<EdgeProperty*> edges_to_delete = properties_view_->getSelectedObjects<EdgeProperty>();

  if (nodes_to_delete.size() + tags_to_delete.size() + edges_to_delete.size() == 0){
    return;
  }

  NodeController* controller = topmap_man_->getController();

  QMessageBox box;
  char* buf = new char[100];
  sprintf(buf, "Delete %d nodes, %d edges and %d tags?", nodes_to_delete.size(), edges_to_delete.size(), tags_to_delete.size());
  std::string info_msg = buf;
  delete buf;
  
  box.setText("You requested removal of objects.");
  box.setInformativeText(QString::fromStdString(info_msg));
  box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  box.setDefaultButton(QMessageBox::Cancel);
  int ret = box.exec();

  if (ret == QMessageBox::Cancel){
    return;
  }
  
  for(int i = 0; i < nodes_to_delete.size(); i++) {
    strands_navigation_msgs::RmvNode srv;
    srv.request.name = nodes_to_delete[i]->getValue().toString().toStdString().c_str();
    
    if (delNodeSrv_.call(srv)) {
      if (srv.response.success) {
	ROS_INFO("Successfully removed node %s", srv.request.name.c_str());
      } else {
	ROS_INFO("Failed to remove node %s", srv.request.name.c_str());
      }
    } else {
      ROS_INFO("Failed to get response from server to remove node %s", srv.request.name.c_str());
    }
  }

  for(int i = 0; i < tags_to_delete.size(); i++) {
    strands_navigation_msgs::AddTag srv;
    srv.request.tag = tags_to_delete[i]->getString().toStdString().c_str();
    // go over all nodes in the controller and see which one this tag is from so
    // we can extract the node name
    std::string node_name;
    for (int nd; nd < controller->numChildren(); nd++) {
      if (controller->childAt(nd)->isAncestorOf(tags_to_delete[i])){
	srv.request.node.push_back(controller->childAt(nd)->getValue().toString().toStdString());
	break;
      }
    }

    if (delTagSrv_.call(srv)) {
      if (srv.response.success) {
	ROS_INFO("Successfully removed tag %s from node %s", srv.request.tag.c_str(), srv.request.node[0].c_str());
      } else {
	ROS_INFO("Failed to remove tag %s from node %s", srv.request.tag.c_str(), srv.request.node[0].c_str());
      }
    } else {
      ROS_INFO("Failed to get response from server to remove tag %s from node %s", srv.request.tag.c_str(), srv.request.node[0].c_str());
    }
  }

  for(int i = 0; i < edges_to_delete.size(); i++) {
    strands_navigation_msgs::AddEdge srv;
    srv.request.edge_id = edges_to_delete[i]->getEdgeId().c_str();

    if (delEdgeSrv_.call(srv)) {
      if (srv.response.success) {
	ROS_INFO("Successfully removed edge %s", srv.request.edge_id.c_str());

      } else {
	ROS_INFO("Failed to remove edge %s", srv.request.edge_id.c_str());
      }
    } else {
      ROS_INFO("Failed to get response from server to remove edge %s", srv.request.edge_id.c_str());
    }
  }

  // Update topological map only once after we remove all the stuff, to prevent
  // update spam.
  updateTopMap();

}

void TopologicalMapPanel::onAddTagClicked()
{
  QList<NodeProperty*> nodes = properties_view_->getSelectedObjects<NodeProperty>();

  QString tag = QInputDialog::getText(this, tr("Add tag"),
				      tr("Tag to add:"));
  
  if (tag.toStdString().empty()){
    return;
  }

  strands_navigation_msgs::AddTag srv;
  srv.request.tag = tag.toStdString().c_str();

  for(int i = 0; i < nodes.size(); i++)
  {
    srv.request.node.push_back(nodes[i]->getValue().toString().toStdString().c_str());
    if (nodes[i]->getTagController()->getHidden()){
      nodes[i]->getTagController()->setHidden(false);
    }
  }

  if (addTagSrv_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Successfully added tag \"%s\" to %d nodes", srv.request.tag.c_str(), nodes.size());
      updateTopMap();
    } else {
      ROS_INFO("Failed to add tag \"%s\" to %d nodes: %s", srv.request.tag.c_str(), nodes.size(), srv.response.meta.c_str());
    }
  } else {
    ROS_WARN("Failed to get response from service to add tag \"%s\" to %d nodes", srv.request.tag.c_str(), nodes.size());
  }
}
  

void TopologicalMapPanel::renameSelected()
{
  // QList<Node*> views_to_rename = properties_view_->getSelectedObjects<NodeController>();
  // if(views_to_rename.size() == 1)
  // {
  //   NodeProperty* view = views_to_rename[ 0 ];

  //   // TODO: should eventually move to a scheme where the CURRENT view
  //   // is not in the same list as the saved views, at which point this
  //   // check can go away.
  //   if(view == topmap_man_->getCurrent())
  //   {
  //     return;
  //   }

  //   QString old_name = view->getName();
  //   QString new_name = QInputDialog::getText(this, "Rename View", "New Name?", QLineEdit::Normal, old_name);

  //   if(new_name.isEmpty() || new_name == old_name)
  //   {
  //     return;
  //   }

  //   view->setName(new_name);
  // }
}

void TopologicalMapPanel::onCurrentChanged()
{
  //QString formatted_class_id = NodeController::formatClassId(topmap_man_->getCurrent()->getClassId());
  // ROS_INFO("CUrrent changed");
  // properties_view_->setAnimated(false);
  // topmap_man_->getCurrent()->expand();
  // properties_view_->setAnimated(true);
}

void TopologicalMapPanel::updateTopMap(){
  ROS_INFO("updating topmap");
  std_msgs::Time t;
  t.data = ros::Time::now();
  update_map_.publish(t);
}

void TopologicalMapPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  properties_view_->save(config);
}

void TopologicalMapPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  properties_view_->load(config);
}

} // namespace topological_rviz_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(topological_rviz_tools::TopologicalMapPanel, rviz::Panel)
