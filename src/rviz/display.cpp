/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "display.h"
#include "visualization_manager.h"
#include "properties/property_manager.h"
#include "properties/property.h"

namespace ogre_vis
{

Display::Display( const std::string& name, VisualizationManager* manager )
: vis_manager_( manager )
, scene_manager_( manager->getSceneManager() )
, name_( name )
, enabled_( false )
, target_frame_( "base" )
, ros_node_( manager->getROSNode() )
, tf_( manager->getTFClient() )
, property_prefix_( name_ + "." )
, property_manager_( NULL )
, parent_category_( NULL )
{
}

Display::~Display()
{
  if ( property_manager_ )
  {
    property_manager_->deleteByUserData( this );
  }
}

void Display::enable( bool force )
{
  if ( enabled_ && !force )
  {
    return;
  }

  enabled_ = true;

  onEnable();
}

void Display::disable( bool force )
{
  if ( !enabled_ && !force )
  {
    return;
  }

  enabled_ = false;

  onDisable();
}

void Display::setRenderCallback( boost::function<void ()> func )
{
  render_callback_ = func;
}

void Display::setLockRenderCallback( boost::function<void ()> func )
{
  render_lock_ = func;
}

void Display::setUnlockRenderCallback( boost::function<void ()> func )
{
  render_unlock_ = func;
}


void Display::causeRender()
{
  if ( render_callback_ )
  {
    render_callback_();
  }
}

void Display::lockRender()
{
  if ( render_lock_ )
  {
    render_lock_();
  }
}

void Display::unlockRender()
{
  if ( render_unlock_ )
  {
    render_unlock_();
  }
}

void Display::setTargetFrame( const std::string& frame )
{
  target_frame_ = frame;

  targetFrameChanged();
}

void Display::setFixedFrame( const std::string& frame )
{
  fixed_frame_ = frame;

  fixedFrameChanged();
}

void Display::setPropertyManager( PropertyManager* manager, CategoryProperty* parent )
{
  property_manager_ = manager;
  parent_category_ = parent;

  property_manager_->createProperty<BoolProperty>( "Enabled", property_prefix_, boost::bind( &Display::isEnabled, this ),
                                                   boost::bind( &VisualizationManager::setDisplayEnabled, vis_manager_, this, _1 ), parent, this );

  createProperties();
}

} // namespace ogre_vis
