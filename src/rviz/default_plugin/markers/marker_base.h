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

#ifndef RVIZ_MARKER_BASE_H
#define RVIZ_MARKER_BASE_H

#include "rviz/selection/forwards.h"
#include "rviz/interactive_object.h"

#include <visualization_msgs/Marker.h>

#include <ros/time.h>

#include <boost/shared_ptr.hpp>

namespace Ogre
{
class SceneNode;
class Vector3;
class Quaternion;
class Entity;
} // namespace Ogre

namespace rviz
{
class DisplayContext;
class MarkerDisplay;
class MarkerSelectionHandler;

typedef std::pair<std::string, int32_t> MarkerID;
typedef std::set<Ogre::MaterialPtr> S_MaterialPtr;

class MarkerBase
{
public:
  typedef visualization_msgs::Marker Marker;
  typedef visualization_msgs::Marker::ConstPtr MarkerConstPtr;

  MarkerBase(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node);

  virtual ~MarkerBase();

  void setMessage(const Marker& message);
  void setMessage(const MarkerConstPtr& message);
  bool expired();

  void updateFrameLocked();

  const MarkerConstPtr& getMessage() const
  {
    return message_;
  }

  MarkerID getID()
  {
    return MarkerID(message_->ns, message_->id);
  }
  std::string getStringID()
  {
    std::stringstream ss;
    ss << message_->ns << "/" << message_->id;
    return ss.str();
  }

  /** @brief Associate an InteractiveObject with this MarkerBase. */
  void setInteractiveObject(InteractiveObjectWPtr object);

  virtual void setPosition(const Ogre::Vector3& position);
  virtual void setOrientation(const Ogre::Quaternion& orientation);
  const Ogre::Vector3& getPosition();
  const Ogre::Quaternion& getOrientation();

  virtual S_MaterialPtr getMaterials()
  {
    return S_MaterialPtr();
  }

protected:
  bool transform(const MarkerConstPtr& message,
                 Ogre::Vector3& pos,
                 Ogre::Quaternion& orient,
                 Ogre::Vector3& scale);
  virtual void onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message) = 0;

  void extractMaterials(Ogre::Entity* entity, S_MaterialPtr& materials);

  MarkerDisplay* owner_;
  DisplayContext* context_;

  Ogre::SceneNode* scene_node_;

  MarkerConstPtr message_;

  ros::Time expiration_;

  boost::shared_ptr<MarkerSelectionHandler> handler_;
};
typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;

} // namespace rviz

#endif
