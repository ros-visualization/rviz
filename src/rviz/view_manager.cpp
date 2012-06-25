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

bool ViewManager::setCurrentViewControllerType( const std::string& type, bool delete_old )
{
  if(delete_old && current_view_ && (current_view_->getClassName() == type || current_view_->getName().toStdString() == type))
  {
    return true;
  }

  ViewController* view = create( type );
  if( !view && !current_view_ )
  {
    view = new OrbitViewController(context_, "Orbit",target_scene_node_);
  }

  if( view )
  {
    property_model_->getRoot()->addChild( view );

    ViewController* old_view = current_view_;
    if( setCurrent( view ) && delete_old )
    {
      delete old_view;
    }
    return true;
  }

  return false;
}

ViewController* ViewManager::create( const std::string& type )
{
  // hack hack hack hack until this becomes truly plugin based
  if(type == "rviz::OrbitViewController" || type == "Orbit")
  {
    return new OrbitViewController(context_, "Orbit",target_scene_node_);
  }
  else if(type == "rviz::XYOrbitViewController" || type == "XYOrbit" ||
           type == "rviz::SimpleOrbitViewController" || type == "SimpleOrbit" /* the old class name */) 
  {
    return new XYOrbitViewController(context_, "XYOrbit",target_scene_node_);
  }
  else if(type == "rviz::FPSViewController" || type == "FPS")
  {
    return new FPSViewController(context_, "FPS",target_scene_node_);
  }
  else if(type == "rviz::FixedOrientationOrthoViewController" || type == "TopDownOrtho" || type == "Top-down Orthographic")
  {
    return new FixedOrientationOrthoViewController(context_, "TopDownOrtho",target_scene_node_);
  }
  return NULL;
}

void ViewManager::copyCurrent()
{
  setCurrentViewControllerType( getCurrent()->getClassName(), false );
}

bool ViewManager::setCurrent( ViewController* view )
{
  if( view != current_view_ )
  {
    context_->getRenderPanel()->setViewController( view );
    view->setTargetFrame( context_->getTargetFrame().toStdString() );
    current_view_ = view;
    Q_EMIT currentChanged( current_view_ );
    Q_EMIT configChanged();
    return true;
  }
  return false;
}

} // end namespace rviz
