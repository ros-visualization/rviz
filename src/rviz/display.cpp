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

namespace rviz
{

Display::Display()
  : vis_manager_( 0 )
  , scene_manager_( 0 )
  , enabled_( false )
  , status_( status_levels::Ok )
  , property_manager_( NULL )
{
}

Display::~Display()
{
  if ( property_manager_ )
  {
    property_manager_->deleteByUserData( this );
  }
}

void Display::initialize( const std::string& name, VisualizationManager* manager )
{
  setName( name );
  vis_manager_ = manager;
  scene_manager_ = manager->getSceneManager();
  update_nh_.setCallbackQueue(manager->getUpdateQueue());
  threaded_nh_.setCallbackQueue(manager->getThreadedQueue());

  // Do subclass initialization, if implemented.
  onInitialize();
}

void Display::setName(const std::string& name)
{
  name_ = name;
  property_prefix_ = name + ".";
}

void Display::enable( bool force )
{
  if ( enabled_ && !force )
  {
    return;
  }

  enabled_ = true;

  if (StatusPropertyPtr status = status_property_.lock())
  {
    status->enable();
  }

  onEnable();

  Q_EMIT stateChanged( this );
}

void Display::disable( bool force )
{
  if ( !enabled_ && !force )
  {
    return;
  }

  enabled_ = false;

  onDisable();

  if (StatusPropertyPtr status = status_property_.lock())
  {
    status->disable();
  }

  Q_EMIT stateChanged( this );
}

void Display::setEnabled(bool en, bool force)
{
  if (en)
  {
    enable(force);
  }
  else
  {
    disable(force);
  }
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

void Display::setFixedFrame( const std::string& frame )
{
  fixed_frame_ = frame;

  fixedFrameChanged();
}

StatusLevel Display::getStatus()
{
  return status_;
}

void Display::setStatus(StatusLevel level, const std::string& name, const std::string& text)
{
  if (StatusPropertyPtr status = status_property_.lock())
  {
    status->setStatus(level, name, text);

    StatusLevel new_status = status->getTopLevelStatus();
    if (new_status != status_)
    {
      status_ = new_status;
      Q_EMIT stateChanged( this );
    }
  }
}

void Display::deleteStatus(const std::string& name)
{
  if (StatusPropertyPtr status = status_property_.lock())
  {
    status->deleteStatus(name);

    StatusLevel new_status = status->getTopLevelStatus();
    if (new_status != status_)
    {
      status_ = new_status;
      Q_EMIT stateChanged( this );
    }
  }
}

void Display::clearStatuses()
{
  if (StatusPropertyPtr status = status_property_.lock())
  {
    status->clear();

    StatusLevel new_status = status->getTopLevelStatus();
    if (new_status != status_)
    {
      status_ = new_status;
      Q_EMIT stateChanged( this );
    }
  }
}

void Display::setPropertyManager( PropertyManager* manager, const CategoryPropertyWPtr& parent )
{
  ROS_ASSERT(!property_manager_);

  property_manager_ = manager;

  parent_category_ = parent;
  status_property_ = property_manager_->createStatus("Status", property_prefix_, parent_category_, this);

  createProperties();
}

void Display::reset()
{
  clearStatuses();
}

} // namespace rviz
