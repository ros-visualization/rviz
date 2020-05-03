/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <sstream>

#include <rviz/display_context.h>
#include <rviz/failed_view_controller.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>

#include <rviz/view_manager.h>

namespace rviz
{
ViewManager::ViewManager(DisplayContext* context)
  : context_(context)
  , root_property_(new ViewControllerContainer)
  , property_model_(new PropertyTreeModel(root_property_))
  , factory_(new PluginlibFactory<ViewController>("rviz", "rviz::ViewController"))
  , current_(nullptr)
  , render_panel_(nullptr)
{
  property_model_->setDragDropClass("view-controller");
  connect(property_model_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));
  connect(this, SIGNAL(currentChanged()), this, SIGNAL(configChanged()));
}

ViewManager::~ViewManager()
{
  delete property_model_;
  delete factory_;
}

void ViewManager::initialize()
{
  setCurrent(create("rviz/Orbit"), false);
}

void ViewManager::update(float wall_dt, float ros_dt)
{
  if (getCurrent())
  {
    getCurrent()->update(wall_dt, ros_dt);
  }
}

ViewController* ViewManager::create(const QString& class_id)
{
  QString error;
  ViewController* view = factory_->make(class_id, &error);
  if (!view)
  {
    view = new FailedViewController(class_id, error);
  }
  view->initialize(context_);

  return view;
}

ViewController* ViewManager::getCurrent() const
{
  return current_;
}

void ViewManager::setCurrentFrom(ViewController* source_view)
{
  if (source_view == nullptr)
  {
    return;
  }

  ViewController* previous = getCurrent();
  if (source_view != previous)
  {
    ViewController* new_current = copy(source_view);

    setCurrent(new_current, false);
    Q_EMIT configChanged();
  }
}

void ViewManager::onCurrentDestroyed(QObject* obj)
{
  if (obj == current_)
  {
    current_ = nullptr;
  }
}

void ViewManager::setCurrent(ViewController* new_current, bool mimic_view)
{
  ViewController* previous = getCurrent();
  if (previous)
  {
    if (mimic_view)
    {
      new_current->mimic(previous);
    }
    else
    {
      new_current->transitionFrom(previous);
    }
    disconnect(previous, SIGNAL(destroyed(QObject*)), this, SLOT(onCurrentDestroyed(QObject*)));
  }
  new_current->setName("Current View");
  connect(new_current, SIGNAL(destroyed(QObject*)), this, SLOT(onCurrentDestroyed(QObject*)));
  current_ = new_current;
  root_property_->addChildToFront(new_current);
  delete previous;

  if (render_panel_)
  {
    // This setViewController() can indirectly call
    // ViewManager::update(), so make sure getCurrent() will return the
    // new one by this point.
    render_panel_->setViewController(new_current);
  }
  if (current_ != previous)
    Q_EMIT currentChanged();
}

void ViewManager::setCurrentViewControllerType(const QString& new_class_id)
{
  setCurrent(create(new_class_id), true);
}

void ViewManager::copyCurrentToList()
{
  ViewController* current = getCurrent();
  if (current)
  {
    ViewController* new_copy = copy(current);
    new_copy->setName(factory_->getClassName(new_copy->getClassId()));
    root_property_->addChild(new_copy);
  }
}

ViewController* ViewManager::getViewAt(int index) const
{
  if (index < 0)
  {
    index = 0;
  }
  return qobject_cast<ViewController*>(root_property_->childAt(index + 1));
}

int ViewManager::getNumViews() const
{
  int count = root_property_->numChildren();
  if (count <= 0)
  {
    return 0;
  }
  else
  {
    return count - 1;
  }
}

void ViewManager::add(ViewController* view, int index)
{
  if (index < 0)
  {
    index = root_property_->numChildren();
  }
  else
  {
    index++;
  }
  property_model_->getRoot()->addChild(view, index);
}

ViewController* ViewManager::take(ViewController* view)
{
  for (int i = 0; i < getNumViews(); i++)
  {
    if (getViewAt(i) == view)
    {
      return qobject_cast<ViewController*>(root_property_->takeChildAt(i + 1));
    }
  }
  return nullptr;
}

ViewController* ViewManager::takeAt(int index)
{
  if (index < 0)
  {
    return nullptr;
  }
  return qobject_cast<ViewController*>(root_property_->takeChildAt(index + 1));
}

void ViewManager::load(const Config& config)
{
  Config current_config = config.mapGetChild("Current");
  QString class_id;
  if (current_config.mapGetString("Class", &class_id))
  {
    ViewController* new_current = create(class_id);
    new_current->load(current_config);
    setCurrent(new_current, false);
  }

  Config saved_views_config = config.mapGetChild("Saved");
  root_property_->removeChildren(1);
  int num_saved = saved_views_config.listLength();
  for (int i = 0; i < num_saved; i++)
  {
    Config view_config = saved_views_config.listChildAt(i);

    if (view_config.mapGetString("Class", &class_id))
    {
      ViewController* view = create(class_id);
      view->load(view_config);
      add(view);
    }
  }
}

void ViewManager::save(Config config) const
{
  getCurrent()->save(config.mapMakeChild("Current"));

  Config saved_views_config = config.mapMakeChild("Saved");
  for (int i = 0; i < getNumViews(); i++)
  {
    getViewAt(i)->save(saved_views_config.listAppendNew());
  }
}

ViewController* ViewManager::copy(ViewController* source)
{
  Config config;
  source->save(config);

  ViewController* copy_of_source = create(source->getClassId());
  copy_of_source->load(config);

  return copy_of_source;
}

void ViewManager::setRenderPanel(RenderPanel* render_panel)
{
  render_panel_ = render_panel;
}

Qt::ItemFlags ViewControllerContainer::getViewFlags(int column) const
{
  return Property::getViewFlags(column) | Qt::ItemIsDropEnabled;
}

void ViewControllerContainer::addChild(Property* child, int index)
{
  if (index == 0)
  {
    index = 1;
  }
  Property::addChild(child, index);
}

void ViewControllerContainer::addChildToFront(Property* child)
{
  Property::addChild(child, 0);
}

} // end namespace rviz
