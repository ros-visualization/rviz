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

#include "pose_tool.h"
#include "visualization_manager.h"
#include "viewport_mouse_event.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_helpers/camera_base.h"
#include "ogre_helpers/arrow.h"
#include "ogre_helpers/qt_ogre_render_window.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <OGRE/OgreRay.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include <tf/transform_listener.h>

namespace rviz
{

PoseTool::PoseTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
: Tool( name, shortcut_key, manager )
{
  arrow_ = new Arrow( scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f );
  arrow_->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
  arrow_->getSceneNode()->setVisible( false );
}

PoseTool::~PoseTool()
{
  delete arrow_;
}

void PoseTool::activate()
{
  state_ = Position;
}

void PoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible( false );
}

Ogre::Vector3 PoseTool::getPositionFromMouseXY( Ogre::Viewport* viewport, int mouse_x, int mouse_y )
{
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay( (float)mouse_x / (float)width, (float)mouse_y / (float)height );
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects( ground_plane );
  if ( !intersection.first )
  {
    return Ogre::Vector3::ZERO;
  }

  return mouse_ray.getPoint( intersection.second );
}

int PoseTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  if( event.leftDown() )
  {
    ROS_ASSERT( state_ == Position );

    pos_ = getPositionFromMouseXY( event.viewport, event.x, event.y );
    arrow_->setPosition( pos_ );

    state_ = Orientation;
    flags |= Render;
  }
  else if( event.type == QEvent::MouseMove && event.left() )
  {
    if( state_ == Orientation )
    {
      //compute angle in x-y plane
      Ogre::Vector3 cur_pos = getPositionFromMouseXY( event.viewport, event.x, event.y );
      double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );

      arrow_->getSceneNode()->setVisible( true );

      //we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
      Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );

      arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );

      flags |= Render;
    }
  }
  else if( event.leftUp() )
  {
    if( state_ == Orientation )
    {
      //compute angle in x-y plane
      Ogre::Vector3 cur_pos = getPositionFromMouseXY( event.viewport, event.x, event.y );
      double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );

      onPoseSet(pos_.x, pos_.y, angle);

      flags |= (Finished|Render);
    }
  }

  return flags;
}

}

