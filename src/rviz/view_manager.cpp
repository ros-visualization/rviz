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

#include "rviz/display_context.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/render_panel.h"
#include "rviz/view_controller.h"
#include "rviz/view_controllers/fixed_orientation_ortho_view_controller.h"
#include "rviz/view_controllers/fps_view_controller.h"
#include "rviz/view_controllers/orbit_view_controller.h"
#include "rviz/view_controllers/xy_orbit_view_controller.h"

#include "rviz/view_manager.h"

namespace rviz
{

ViewManager::ViewManager( DisplayContext* context )
  : context_( context )
  , current_view_( NULL )
  , property_model_( new PropertyTreeModel( new Property ))
{
}

ViewManager::~ViewManager()
{
}

void ViewManager::initialize( Ogre::SceneNode* target_scene_node )
{
  target_scene_node_ = target_scene_node;

  addViewController(XYOrbitViewController::getClassNameStatic(), "XYOrbit");
  addViewController(OrbitViewController::getClassNameStatic(), "Orbit");
  addViewController(FPSViewController::getClassNameStatic(), "FPS");
  addViewController(FixedOrientationOrthoViewController::getClassNameStatic(), "TopDownOrtho");
  setCurrentViewControllerType(OrbitViewController::getClassNameStatic());
}

void ViewManager::update( float wall_dt, float ros_dt )
{
  if( current_view_ )
  {
    current_view_->update( wall_dt, ros_dt );
  }
}

void ViewManager::addViewController(const std::string& class_name, const std::string& name)
{
  Q_EMIT viewControllerTypeAdded( class_name, name );
  types_.append( QString::fromStdString( name ));
}

bool ViewManager::setCurrentViewControllerType(const std::string& type)
{
  if(current_view_ && (current_view_->getClassName() == type || current_view_->getName().toStdString() == type))
  {
    return true;
  }

  bool found = true;
  // hack hack hack hack until this becomes truly plugin based
  if(type == "rviz::OrbitViewController" || type == "Orbit")
  {
    current_view_ = new OrbitViewController(context_, "Orbit",target_scene_node_);
  }
  else if(type == "rviz::XYOrbitViewController" || type == "XYOrbit" ||
           type == "rviz::SimpleOrbitViewController" || type == "SimpleOrbit" /* the old class name */) 
  {
    current_view_ = new XYOrbitViewController(context_, "XYOrbit",target_scene_node_);
  }
  else if(type == "rviz::FPSViewController" || type == "FPS")
  {
    current_view_ = new FPSViewController(context_, "FPS",target_scene_node_);
  }
  else if(type == "rviz::FixedOrientationOrthoViewController" || type == "TopDownOrtho" || type == "Top-down Orthographic")
  {
    current_view_ = new FixedOrientationOrthoViewController(context_, "TopDownOrtho",target_scene_node_);
  }
  else if(!current_view_)
  {
    current_view_ = new OrbitViewController(context_, "Orbit",target_scene_node_);
  }
  else
  {
    found = false;
  }

  if(found)
  {
    property_model_->getRoot()->addChild( current_view_ );

    // RenderPanel::setViewController() deletes the old
    // ViewController, so don't do it here or it will crash!
    context_->getRenderPanel()->setViewController(current_view_);
    current_view_->setTargetFrame( context_->getTargetFrame().toStdString() );
    connect( current_view_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));
    Q_EMIT viewControllerChanged( current_view_ );
    Q_EMIT configChanged();
  }

  return found;
}

std::string ViewManager::getCurrentViewControllerType()
{
  return current_view_->getClassName();
}

} // end namespace rviz
