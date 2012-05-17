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


#ifndef RANGE_DISPLAY_H
#define RANGE_DISPLAY_H

#include <sensor_msgs/Range.h>

#include <message_filters/subscriber.h>

#include <tf/message_filter.h>

#include "rviz/display.h"

namespace rviz
{
class Shape;
}


namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;

/**
 * \class RangeDisplay
 * \brief Displays a sensor_msgs::Range message as a cone.
 */
class RangeDisplay: public rviz::Display
{
Q_OBJECT
public:
  RangeDisplay();
  virtual ~RangeDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage( const sensor_msgs::Range::ConstPtr& msg );

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void updateTopic();
  void updateBufferLength();
  void updateColorAndAlpha();

private:
  uint32_t messages_received_;

  Ogre::SceneNode* scene_node_;
  std::vector<Shape* > cones_;      ///< Handles actually drawing the cone

  message_filters::Subscriber<sensor_msgs::Range> sub_;
  tf::MessageFilter<sensor_msgs::Range>* tf_filter_;
  sensor_msgs::Range::ConstPtr current_message_;

  ColorProperty* color_property_;
  RosTopicProperty* topic_property_;
  FloatProperty* alpha_property_;
  IntProperty* buffer_length_property_;
};

} // namespace range_plugin

#endif /* RANGE_DISPLAY_H */

