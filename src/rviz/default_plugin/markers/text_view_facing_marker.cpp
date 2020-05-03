/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/assert.h>

#include "rviz/default_plugin/markers/marker_selection_handler.h"
#include "rviz/display_context.h"
#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/default_plugin/markers/text_view_facing_marker.h"

namespace rviz
{
TextViewFacingMarker::TextViewFacingMarker(MarkerDisplay* owner,
                                           DisplayContext* context,
                                           Ogre::SceneNode* parent_node)
  : MarkerBase(owner, context, parent_node), text_(nullptr)
{
}

TextViewFacingMarker::~TextViewFacingMarker()
{
  delete text_;
}

void TextViewFacingMarker::onNewMessage(const MarkerConstPtr& /*old_message*/,
                                        const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::TEXT_VIEW_FACING);

  if (!text_)
  {
    text_ = new MovableText(new_message->text);
    text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_CENTER);
    scene_node_->attachObject(text_);

    handler_.reset(
        new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id), context_));
    handler_->addTrackedObject(text_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  setPosition(pos);
  text_->setCharacterHeight(new_message->scale.z);
  text_->setColor(Ogre::ColourValue(new_message->color.r, new_message->color.g, new_message->color.b,
                                    new_message->color.a));
  text_->setCaption(new_message->text);
}

S_MaterialPtr TextViewFacingMarker::getMaterials()
{
  S_MaterialPtr materials;
  if (text_ && text_->getMaterial().get())
  {
    materials.insert(text_->getMaterial());
  }
  return materials;
}

} // namespace rviz
