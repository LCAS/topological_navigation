#include "topmap_manager.h"

namespace topological_rviz_tools
{
TopmapManager::TopmapManager(rviz::DisplayContext* context)
  : context_(context)
  , root_property_(new NodeController)
  , property_model_(new rviz::PropertyTreeModel(root_property_))
  , factory_(new rviz::PluginlibFactory<NodeController>("topological_rviz_tools", "topological_rviz_tools::NodeController"))
  , current_(NULL)
  , render_panel_(NULL)
{
  RCLCPP_INFO("Initialising node manager");
  property_model_->setDragDropClass("node-controller");
  // connect(property_model_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));
  // add(new NodeController, -1);
}

TopmapManager::~TopmapManager()
{
  delete property_model_;
  delete factory_;
  delete root_property_;
}

void TopmapManager::initialize()
{
}

void TopmapManager::update(float wall_dt, float ros_dt)
{
}

NodeController* TopmapManager::create(const QString& class_id)
{
  // QString error;
  // NodeController* view = factory_->make(class_id, &error);
  // if(!view)
  // {
  //   view = new FailedNodeController(class_id, error);
  // }
  // view->initialize();

  // return view;
}

NodeController* TopmapManager::getController() const
{
  return root_property_;
}

NodeProperty* TopmapManager::getCurrent() const
{
  return current_;
}

void TopmapManager::setCurrentFrom(NodeProperty* source_view)
{
  // if(source_view == NULL)
  // {
  //   return;
  // }

  // NodeController* previous = getCurrent();
  // if(source_view != previous)
  // {
  //   NodeController* new_current = copy(source_view);

  //   setCurrent(new_current, false);
  //   Q_EMIT configChanged();
  // }
}

void TopmapManager::onCurrentDestroyed(QObject* obj)
{
  if(obj == current_)
  {
    current_ = NULL;
  }
}

void TopmapManager::setCurrent(NodeProperty* new_current, bool mimic_view)
{
  NodeProperty* previous = getCurrent();
  if(previous)
  {
    disconnect(previous, SIGNAL(destroyed(QObject*)), this, SLOT(onCurrentDestroyed(QObject*)));
  }
  new_current->setName("Current View");
  connect(new_current, SIGNAL(destroyed(QObject*)), this, SLOT(onCurrentDestroyed(QObject*)));
  current_ = new_current;
  // root_property_->addChildToFront(new_current);
  delete previous;

  // if(render_panel_)
  // {
  //   This setNodeController() can indirectly call
  //   TopmapManager::update(), so make sure getCurrent() will return the
  //   new one by this point.
  //   render_panel_->setViewController(new_current);
  // }
  Q_EMIT currentChanged();
}

void TopmapManager::setCurrentNodeControllerType(const QString& new_class_id)
{
  // setCurrent(create(new_class_id), true);
}

void TopmapManager::copyCurrentToList()
{
  // NodeController* current = getCurrent();
  // if(current)
  // {
  //   NodeController* new_copy = copy(current);
  //   new_copy->setName(factory_->getClassName(new_copy->getClassId()));
  //   root_property_->addChild(new_copy);
  // }
}

NodeController* TopmapManager::getViewAt(int index) const
{
  // if(index < 0)
  // {
  //   index = 0;
  // }
  // return qobject_cast<NodeController*>(root_property_->childAt(index + 1));
}

int TopmapManager::getNumViews() const
{
  // int count = root_property_->numChildren();
  // if(count <= 0)
  // {
  //   return 0;
  // }
  // else
  // {
  //   return count-1;
  // }
}

// void TopmapManager::add(NodeController* view, int index)
// {
//   // if(index < 0)
//   // {
//   //   index = root_property_->numChildren();
//   // }
//   // else
//   // {
//   //   index++;
//   // }
//   // property_model_->getRoot()->addChild(view, index);
// }

NodeController* TopmapManager::take(NodeController* view)
{
  // for(int i = 0; i < getNumViews(); i++)
  // {
  //   if(getViewAt(i) == view)
  //   {
  //     return qobject_cast<NodeController*>(root_property_->takeChildAt(i + 1));
  //   }
  // }
  // return NULL;
}

NodeController* TopmapManager::takeAt(int index)
{
  // if(index < 0)
  // {
  //   return NULL;
  // }
  // return qobject_cast<NodeController*>(root_property_->takeChildAt(index + 1));
}

void TopmapManager::load(const rviz::Config& config)
{
  // rviz::Config current_config = config.mapGetChild("Current");
  // QString class_id;
  // if(current_config.mapGetString("Class", &class_id))
  // {
  //   NodeController* new_current = create(class_id);
  //   new_current->load(current_config);
  //   setCurrent(new_current, false);
  // }

  // rviz::Config saved_views_config = config.mapGetChild("Saved");
  // root_property_->removeChildren(1);
  // int num_saved = saved_views_config.listLength();
  // for(int i = 0; i < num_saved; i++)
  // {
  //   rviz::Config view_config = saved_views_config.listChildAt(i);
    
  //   if(view_config.mapGetString("Class", &class_id))
  //   {
  //     NodeController* view = create(class_id);
  //     view->load(view_config);
  //     add(view);
  //   }
  // }
}

void TopmapManager::save(rviz::Config config) const
{
  // getCurrent()->save(config.mapMakeChild("Current"));

  // rviz::Config saved_views_config = config.mapMakeChild("Saved");
  // for(int i = 0; i < getNumViews(); i++)
  // {
  //   getViewAt(i)->save(saved_views_config.listAppendNew());
  // }
}

NodeProperty* TopmapManager::copy(NodeProperty* source)
{
  // rviz::Config config;
  // source->save(config);

  // NodeController* copy_of_source = create(source->getClassId());
  // copy_of_source->load(config);

  // return copy_of_source;
  return NULL;
}

}
