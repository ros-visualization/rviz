/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include "interactive_marker.h"

#include "rviz/frame_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/default_plugin/interactive_marker_display.h"
#include "rviz/render_panel.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMath.h>

#include <boost/make_shared.hpp>
#include <wx/menu.h>

namespace rviz
{

InteractiveMarker::InteractiveMarker( InteractiveMarkerDisplay *owner, VisualizationManager *vis_manager ) :
  owner_(owner)
, vis_manager_(vis_manager)
, dragging_(false)
, pose_update_requested_(false)
, axes_( vis_manager->getSceneManager(), 0, 1, 0.05 )
, menu_(0)
, heart_beat_t_(0)
{
}

InteractiveMarker::~InteractiveMarker()
{
  delete menu_;
}

void InteractiveMarker::reset()
{
  controls_.clear();
}

bool InteractiveMarker::processMessage( visualization_msgs::InteractiveMarkerConstPtr message )
{
  reset();

  visualization_msgs::InteractiveMarker auto_message = *message;

  name_ = auto_message.name;

  if ( auto_message.controls.size() == 0 )
  {
    owner_->setStatus( status_levels::Ok, name_, "Marker empty.");
    return true;
  }

  frame_locked_ = auto_message.frame_locked;

  size_ = auto_message.size;
  axes_.set( size_ * 0.5, size_*0.025 );

  reference_frame_ = auto_message.header.frame_id;

  Ogre::Vector3 reference_position;
  Ogre::Quaternion reference_orientation;

  // get parent pose
  if (!FrameManager::instance()->getTransform( auto_message.header, reference_position, reference_orientation ))
  {
    std::string error;
    FrameManager::instance()->transformHasProblems(auto_message.header.frame_id, auto_message.header.stamp, error);
    owner_->setStatus( status_levels::Error, name_, error);
    return false;
  }

  reference_position_ = reference_position;
  reference_orientation_ = reference_orientation;

  Ogre::Vector3 position( auto_message.pose.position.x, auto_message.pose.position.y, auto_message.pose.position.z );
  Ogre::Quaternion orientation( auto_message.pose.orientation.w, auto_message.pose.orientation.x,
      auto_message.pose.orientation.y, auto_message.pose.orientation.z );

  position_ = position;
  orientation_ = orientation;

  setReferencePose( reference_position, reference_orientation );
  setPose( reference_position + reference_orientation*position, reference_orientation*orientation );

  for ( unsigned i=0; i<auto_message.controls.size(); i++ )
  {
    controls_.push_back( boost::make_shared<InteractiveMarkerControl>( vis_manager_, auto_message.controls[i], this ) );
  }

  owner_->setStatus( status_levels::Ok, name_, "OK");

  if ( message->menu.size() > 0 )
  {
    menu_ = new wxMenu();

    for ( unsigned i=0; i<message->menu.size(); i++ )
    {
      wxString title = wxString::FromAscii(message->menu[i].title.c_str());
      if ( message->menu[i].entries.empty() )
      {
        menu_->Append( i, title );
      }
      else
      {
        wxMenu* sub_menu = new wxMenu;
        for ( unsigned j=0; j<message->menu[i].entries.size(); j++ )
        {
          wxString entry = wxString::FromAscii( message->menu[i].entries[j].c_str());
          sub_menu->Append( j, entry );
        }
        menu_->AppendSubMenu( sub_menu, title );
      }
    }
  }

  return true;
}

void InteractiveMarker::update(float wall_dt)
{
  Ogre::Vector3 reference_position;
  Ogre::Quaternion reference_orientation;

  // get parent pose
  if (!FrameManager::instance()->getTransform( reference_frame_, ros::Time(0), reference_position, reference_orientation ))
  {
    std::string error;
    FrameManager::instance()->transformHasProblems(reference_frame_, ros::Time(0), error);
    owner_->setStatus( status_levels::Error, name_, error);
    return;
  }

  setReferencePose( reference_position, reference_orientation );
/*
  heart_beat_t_ += wall_dt * 2.0;
  if ( heart_beat_t_ > 1.0 ) heart_beat_t_ -= 1.0;

  float heart_beat = 1.0/(heart_beat_t_+1.0) * 0.15 + 0.1;

  std::list<InteractiveMarkerControlPtr>::iterator it;
  for ( it = controls_.begin(); it != controls_.end(); it++ )
  {
    (*it)->update( heart_beat );
  }
  */
}

void InteractiveMarker::requestPoseUpdate( Ogre::Vector3 position, Ogre::Quaternion orientation )
{
  if ( dragging_ )
  {
    pose_update_requested_ = true;
    requested_position_ = position;
    requested_orientation_ = orientation;
  }
  else
  {
    setPose( position, orientation );
  }
}

void InteractiveMarker::setReferencePose( Ogre::Vector3 position, Ogre::Quaternion orientation )
{
  if ( frame_locked_ )
  {
    Ogre::Quaternion delta_orientation = orientation * reference_orientation_.Inverse();
    Ogre::Vector3 delta_position = position - reference_position_;

    setPose( position_ + delta_position, delta_orientation * orientation_ );
  }

  reference_position_ = position;
  reference_orientation_ = orientation;
}

void InteractiveMarker::setPose( Ogre::Vector3 position, Ogre::Quaternion orientation )
{
  position_ = position;
  orientation_ = orientation;

  axes_.setPosition(position_);
  axes_.setOrientation(orientation_);

  std::list<InteractiveMarkerControlPtr>::iterator it;
  for ( it = controls_.begin(); it != controls_.end(); it++ )
  {
    (*it)->interactiveMarkerPoseChanged( position_, orientation_ );
  }
}

void InteractiveMarker::translate( Ogre::Vector3 delta_position )
{
  setPose( position_+delta_position, orientation_ );
}

void InteractiveMarker::rotate( Ogre::Quaternion delta_orientation )
{
  setPose( position_, delta_orientation * orientation_ );
}

void InteractiveMarker::startDragging()
{
  dragging_ = true;
}

void InteractiveMarker::stopDragging()
{
  if ( pose_update_requested_ )
  {
    position_ = requested_position_;
    orientation_ = requested_orientation_;
  }
  pose_update_requested_ = false;
  dragging_ = false;
}

bool InteractiveMarker::handleMouseEvent(ViewportMouseEvent& event)
{
  if ( !menu_ )
  {
    return false;
  }

  if ( event.event.RightDown() || event.event.RightIsDown() )
  {
    return true;
  }

  if ( event.event.RightUp() )
  {
    event.panel->PopupMenu( menu_, event.event.GetX(), event.event.GetY() );
    return true;
  }

  return false;
}

}
