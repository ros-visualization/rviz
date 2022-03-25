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

#include "marker_base.h"
#include <rviz/default_plugin/marker_display.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include "marker_selection_handler.h"
#include <rviz/frame_manager.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreSharedPtr.h>


#include <utility>


namespace rviz
{
MarkerBase::MarkerBase(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
  : owner_(owner), context_(context), scene_node_(parent_node->createChildSceneNode())
{
}

MarkerBase::~MarkerBase()
{
  context_->getSceneManager()->destroySceneNode(scene_node_);
}

void MarkerBase::setMessage(const Marker& message)
{
  // copy and save to shared pointer
  MarkerConstPtr message_ptr(new Marker(message));
  setMessage(message_ptr);
}

void MarkerBase::setMessage(const MarkerConstPtr& message)
{
  MarkerConstPtr old = message_;
  message_ = message;

  expiration_ = ros::Time::now() + message->lifetime;

  onNewMessage(old, message);
}

void MarkerBase::updateFrameLocked()
{
  ROS_ASSERT(message_ && message_->frame_locked);
  onNewMessage(message_, message_);
}

bool MarkerBase::expired()
{
  return ros::Time::now() >= expiration_;
}

bool MarkerBase::transform(const MarkerConstPtr& message,
                           Ogre::Vector3& pos,
                           Ogre::Quaternion& orient,
                           Ogre::Vector3& scale)
{
  ros::Time stamp = message->header.stamp;
  if (message->frame_locked)
  {
    stamp = ros::Time();
  }

  if (!context_->getFrameManager()->transform(message->header.frame_id, stamp, message->pose, pos,
                                              orient))
  {
    std::string error;
    context_->getFrameManager()->transformHasProblems(message->header.frame_id, message->header.stamp,
                                                      error);
    if (owner_)
    {
      owner_->setMarkerStatus(getID(), StatusProperty::Error, error);
    }
    return false;
  }

  scale = Ogre::Vector3(message->scale.x, message->scale.y, message->scale.z);

  return true;
}

void MarkerBase::setInteractiveObject(InteractiveObjectWPtr control)
{
  if (handler_)
  {
    handler_->setInteractiveObject(std::move(control));
  }
}

void MarkerBase::setPosition(const Ogre::Vector3& position)
{
  scene_node_->setPosition(position);
}

void MarkerBase::setOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_->setOrientation(orientation);
}

const Ogre::Vector3& MarkerBase::getPosition() const
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& MarkerBase::getOrientation() const
{
  return scene_node_->getOrientation();
}

void MarkerBase::extractMaterials(Ogre::Entity* entity, S_MaterialPtr& materials)
{
  uint32_t num_sub_entities = entity->getNumSubEntities();
  for (uint32_t i = 0; i < num_sub_entities; ++i)
  {
    Ogre::SubEntity* sub = entity->getSubEntity(i);
    const Ogre::MaterialPtr& material = sub->getMaterial();
    materials.insert(material);
  }
}


} // namespace rviz
