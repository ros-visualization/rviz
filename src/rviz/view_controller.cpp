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

#include "view_controller.h"
#include "viewport_mouse_event.h"
#include "visualization_manager.h"
#include "frame_manager.h"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

ViewController::ViewController(VisualizationManager* manager, const std::string& name)
: manager_(manager)
, camera_(0)
, name_(name)
{
  reference_node_ = manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();

  //rotate the reference node to account for the different coordinate conventions
  //Ogre: x right   y up   z backwards
  //ROS : x forward y left z up
  Ogre::Matrix3 g_ogre_to_robot_matrix;
  g_ogre_to_robot_matrix.FromEulerAnglesYXZ( Ogre::Degree( -90 ), Ogre::Degree( 0 ), Ogre::Degree( -90 ) );
  global_orientation_.FromRotationMatrix( g_ogre_to_robot_matrix );
}

ViewController::~ViewController()
{
  manager_->getSceneManager()->destroySceneNode(reference_node_);
}

void ViewController::activate(Ogre::Camera* camera, const std::string& reference_frame)
{
  camera_ = camera;
  reference_frame_ = reference_frame;
  updateReferenceNode();
  reference_node_->attachObject(camera_);

  onActivate();
}

void ViewController::deactivate()
{
  onDeactivate();

  reference_node_->detachObject(camera_);
  camera_ = 0;
}

void ViewController::update(float dt, float ros_dt)
{
  updateReferenceNode();
  onUpdate(dt, ros_dt);
}

void ViewController::setReferenceFrame(const std::string& reference_frame)
{
  Ogre::Vector3 old_pos = reference_node_->getPosition();
  Ogre::Quaternion old_orient = reference_node_->getOrientation();
  reference_frame_ = reference_frame;
  updateReferenceNode();

  onReferenceFrameChanged(old_pos, old_orient);
}

void ViewController::updateReferenceNode()
{
  Ogre::Vector3 old_position = reference_node_->getPosition();
  Ogre::Quaternion old_orientation = reference_node_->getOrientation();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  FrameManager::instance()->getTransform(reference_frame_, ros::Time(), position, orientation);

  reference_node_->setPosition(position);
  reference_node_->setOrientation(orientation * global_orientation_);

  if (!old_position.positionEquals(position, 0.01) ||
      !old_orientation.equals(orientation, Ogre::Radian(0.05)))
  {
    manager_->queueRender();
  }
}

}
