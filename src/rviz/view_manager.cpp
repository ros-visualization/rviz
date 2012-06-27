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
#include "rviz/properties/drop_enabled_property.h"
#include "rviz/properties/enum_property.h"
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
  , root_property_( new DropEnabledProperty )
  , property_model_( new PropertyTreeModel( root_property_ ))
{
  property_model_->setDragDropClass( "view-controller" );

  class_ids_.append( "rviz/Orbit" );
  class_ids_.append( "rviz/XYOrbit" );
  class_ids_.append( "rviz/FPS" );
  class_ids_.append( "rviz/TopDownOrtho" );
}

ViewManager::~ViewManager()
{
}

void ViewManager::initialize( Ogre::SceneNode* target_scene_node )
{
  target_scene_node_ = target_scene_node;

  setCurrentViewControllerType( "Orbit" );
}

void ViewManager::update( float wall_dt, float ros_dt )
{
  if( current_view_ )
  {
    current_view_->update( wall_dt, ros_dt );
  }
}

bool ViewManager::setCurrentViewControllerType( const std::string& type )
{
  if( current_view_ && (current_view_->getClassName() == type || current_view_->getName().toStdString() == type))
  {
    return true;
  }

  ViewController* view = create( type );
  if( !view && !current_view_ )
  {
    view = create( "Orbit" );
  }

  if( view )
  {
    add( view );

    ViewController* old_view = current_view_;
    if( old_view )
    {
      view->initializeFrom( old_view );
    }
    setCurrent( view );

    delete old_view;
    return true;
  }

  return false;
}

ViewController* ViewManager::create( const std::string& type )
{
  ViewController* view = NULL;

  // hack hack hack hack until this becomes truly plugin based
  if( type == "rviz/Orbit" || type == "Orbit")
  {
    view = new OrbitViewController(context_, "Orbit",target_scene_node_);
    view->setClassId( "rviz/Orbit" );
  }
  else if(type == "rviz/XYOrbit" || type == "XYOrbit" ) 
  {
    view = new XYOrbitViewController(context_, "XYOrbit",target_scene_node_);
    view->setClassId( "rviz/XYOrbit" );
  }
  else if(type == "rviz/FPS" || type == "FPS")
  {
    view = new FPSViewController(context_, "FPS",target_scene_node_);
    view->setClassId( "rviz/FPS" );
  }
  else if(type == "rviz/TopDownOrtho" || type == "TopDownOrtho")
  {
    view = new FixedOrientationOrthoViewController(context_, "TopDownOrtho",target_scene_node_);
    view->setClassId( "rviz/TopDownOrtho" );
  }

  if( view )
  {
    // When plugin is implemented, this should all be automated based on info from the factory.
    view->addTypeSelector( class_ids_ );
  }

  return view;
}

void ViewManager::copyCurrent()
{
  ViewController* new_view = getCurrent()->copy();
  new_view->setName( "Copy of " + getCurrent()->getName() );
  new_view->setClassId( getCurrent()->getClassId() ); // copy() does not copy class_id_.  Maybe should fix that.
  new_view->addTypeSelector( class_ids_ ); // copy() does not copy the type selector.  Maybe should fix that.
  add( new_view, getCurrent()->rowNumberInParent() + 1 );
  setCurrent( new_view );
}

bool ViewManager::setCurrent( ViewController* view, bool deactivate_previous )
{
  if( view != current_view_ )
  {
    if( deactivate_previous && current_view_ )
    {
      disconnect( current_view_, SIGNAL( destroyed( QObject* )), this, SLOT( onViewDeleted( QObject* )));
    }
    connect( view, SIGNAL( destroyed( QObject* )), this, SLOT( onViewDeleted( QObject* )));

    context_->getRenderPanel()->setViewController( view, deactivate_previous );
    view->setTargetFrame( context_->getTargetFrame().toStdString() );
    current_view_ = view;
    Q_EMIT currentChanged( current_view_ );
    Q_EMIT configChanged();
    return true;
  }
  return false;
}

void ViewManager::onViewDeleted( QObject* deleted_object )
{
  if( current_view_ == deleted_object )
  {
    ViewController* view;
    if( getNumViews() == 0 || (getNumViews() == 1 && getViewAt( 0 ) == current_view_))
    {
      view = create( "Orbit" );
      add( view );
    }
    else
    {
      view = getViewAt( 0 );
      if( view == current_view_ )
      {
        view = getViewAt( 1 );
      }
    }

    setCurrent( view, false );
  }
}

ViewController* ViewManager::getViewAt( int index ) const
{
  return qobject_cast<ViewController*>( root_property_->childAt( index ));
}

int ViewManager::getNumViews() const
{
  return root_property_->numChildren();
}

void ViewManager::add( ViewController* view, int index )
{
  property_model_->getRoot()->addChild( view, index );
}

} // end namespace rviz
