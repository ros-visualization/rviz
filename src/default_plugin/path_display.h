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


#ifndef RVIZ_PATH_DISPLAY_H
#define RVIZ_PATH_DISPLAY_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <boost/shared_ptr.hpp>

namespace ogre_tools
{
class PointCloud;
}

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace rviz
{

/**
 * \class PathDisplay
 * \brief Displays a nav_msgs::Path message
 */
class PathDisplay : public Display
{
public:
  PathDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~PathDisplay();

  void setTopic( const std::string& topic );
  const std::string& getTopic() { return topic_; }

  void setColor( const Color& color );
  const Color& getColor() { return color_; }

  void setAlpha( float alpha );
  float getAlpha() { return alpha_; }

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  static const char* getTypeStatic() { return "Path"; }
  virtual const char* getType() const { return getTypeStatic(); }
  static const char* getDescription();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const nav_msgs::Path::ConstPtr& msg);
  void processMessage(const nav_msgs::Path::ConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  Color color_;
  float alpha_;

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;

  message_filters::Subscriber<nav_msgs::Path> sub_;
  tf::MessageFilter<nav_msgs::Path> tf_filter_;
  nav_msgs::Path::ConstPtr current_message_;

  ColorPropertyWPtr color_property_;
  ROSTopicStringPropertyWPtr topic_property_;
  FloatPropertyWPtr alpha_property_;
};

} // namespace rviz

#endif /* RVIZ_PATH_DISPLAY_H */

