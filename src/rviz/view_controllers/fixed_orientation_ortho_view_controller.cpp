/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "fixed_orientation_ortho_view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreViewport.h>

#include <ogre_tools/shape.h>
#include <ogre_tools/orthographic.h>

#include <stdint.h>
#include <sstream>

namespace rviz
{

FixedOrientationOrthoViewController::FixedOrientationOrthoViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node)
: ViewController(manager, name, target_scene_node)
, scale_(10.0f)
, orientation_(Ogre::Quaternion::IDENTITY)
{
}

FixedOrientationOrthoViewController::~FixedOrientationOrthoViewController()
{
}

void FixedOrientationOrthoViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  bool moved = false;

  if ( event.event.Dragging() )
  {
    int32_t diff_x = event.event.GetX() - event.last_x;
    int32_t diff_y = event.event.GetY() - event.last_y;

    if ( event.event.LeftIsDown() && !event.event.ShiftDown() )
    {
      camera_->roll( Ogre::Radian( -diff_x * 0.005 ) );
      orientation_ = camera_->getOrientation();
    }
    else if ( event.event.MiddleIsDown() || 
	      ( event.event.ShiftDown() && event.event.LeftIsDown() ))
    {
      move( -diff_x / scale_, diff_y / scale_, 0.0f );
    }
    else if ( event.event.RightIsDown() )
    {
      scale_ *= 1.0 - diff_y * 0.01;
    }

    moved = true;
  }

  if ( event.event.GetWheelRotation() != 0 )
  {
    int diff = event.event.GetWheelRotation();
    scale_ *= 1.0 - (-diff) * 0.001;

    moved = true;
  }

  if (moved)
  {
    manager_->queueRender();
  }
}

void FixedOrientationOrthoViewController::setOrientation(const Ogre::Quaternion& orientation)
{
  orientation_ = orientation;
}

void FixedOrientationOrthoViewController::onActivate()
{
  camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  camera_->setFixedYawAxis(false);
  camera_->setDirection(target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_X);
}

void FixedOrientationOrthoViewController::onDeactivate()
{
  camera_->setCustomProjectionMatrix(false);
}

void FixedOrientationOrthoViewController::onUpdate(float dt, float ros_dt)
{
  updateCamera();
}

void FixedOrientationOrthoViewController::lookAt( const Ogre::Vector3& point )
{
  Ogre::Vector3 reference_point = target_scene_node_->getPosition() - point;
  Ogre::Vector3 current_pos = camera_->getPosition();
  current_pos.x = reference_point.x;
  current_pos.z = reference_point.z;

  camera_->setPosition(current_pos);
}

void FixedOrientationOrthoViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  lookAt(target_scene_node_->getPosition());
}

void FixedOrientationOrthoViewController::updateCamera()
{
  camera_->setOrientation(orientation_);

  float width = camera_->getViewport()->getActualWidth();
  float height = camera_->getViewport()->getActualHeight();

  Ogre::Matrix4 proj;
  ogre_tools::buildScaledOrthoMatrix( proj, -width / scale_ / 2, width / scale_ / 2, -height / scale_ / 2, height / scale_ / 2,
                                      camera_->getNearClipDistance(), camera_->getFarClipDistance() );
  camera_->setCustomProjectionMatrix(true, proj);
}

void FixedOrientationOrthoViewController::move( float x, float y, float z )
{
  camera_->moveRelative( Ogre::Vector3( x, y, z ) );
}

void FixedOrientationOrthoViewController::fromString(const std::string& str)
{
  std::istringstream iss(str);

  iss >> scale_;
  iss.ignore();

  Ogre::Vector3 vec;
  iss >> vec.x;
  iss.ignore();
  iss >> vec.y;
  iss.ignore();
  iss >> vec.z;
  iss.ignore();
  camera_->setPosition(vec);

  Ogre::Quaternion quat;
  iss >> quat.x;
  iss.ignore();
  iss >> quat.y;
  iss.ignore();
  iss >> quat.z;
  iss.ignore();
  iss >> quat.w;
  iss.ignore();
  orientation_ = quat;
}

std::string FixedOrientationOrthoViewController::toString()
{
  std::ostringstream oss;
  oss << scale_ << " " << camera_->getPosition().x << " " << camera_->getPosition().y << " " << camera_->getPosition().z
      << " " << camera_->getOrientation().x << " " << camera_->getOrientation().y << " " << camera_->getOrientation().z << " " << camera_->getOrientation().w;

  return oss.str();
}


}
