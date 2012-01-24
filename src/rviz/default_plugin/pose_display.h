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


#ifndef RVIZ_POSE_DISPLAY_H_
#define RVIZ_POSE_DISPLAY_H_

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"
#include "rviz/selection/forwards.h"

#include <geometry_msgs/PoseStamped.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace rviz
{
class Arrow;
class Axes;
class Shape;
}

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class PoseDisplaySelectionHandler;
typedef boost::shared_ptr<PoseDisplaySelectionHandler> PoseDisplaySelectionHandlerPtr;

/**
 * \class PoseDisplay
 * \brief Accumulates and displays the pose from a geometry_msgs::PoseStamped message
 */
class PoseDisplay : public Display
{
public:
  enum Shape
  {
    Arrow,
    Axes,
  };

  PoseDisplay();
  virtual ~PoseDisplay();

  virtual void onInitialize();

  void setTopic( const std::string& topic );
  const std::string& getTopic() { return topic_; }

  void setColor( const Color& color );
  const Color& getColor() { return color_; }

  void setShape(int shape);
  int getShape() { return current_shape_; }

  void setAlpha(float a);
  float getAlpha() { return alpha_; }

  // arrow-specific setters/getters
  void setHeadRadius(float r);
  float getHeadRadius() { return head_radius_; }
  void setHeadLength(float l);
  float getHeadLength() { return head_length_; }
  void setShaftRadius(float r);
  float getShaftRadius() { return shaft_radius_; }
  void setShaftLength(float l);
  float getShaftLength() { return shaft_length_; }

  // axes-specific setters/getters
  void setAxesRadius(float r);
  float getAxesRadius() { return axes_radius_; }
  void setAxesLength(float l);
  float getAxesLength() { return axes_length_; }

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

protected:
  void subscribe();
  void unsubscribe();
  void clear();

  void createShapeProperties();
  void setVisibility();

  void incomingMessage( const geometry_msgs::PoseStamped::ConstPtr& message );

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  Color color_;
  float alpha_;
  Shape current_shape_;

  // arrow-specific values
  float head_radius_;
  float head_length_;
  float shaft_radius_;
  float shaft_length_;

  // axes-specific values
  float axes_length_;
  float axes_radius_;

  rviz::Arrow* arrow_;
  rviz::Axes* axes_;
  CollObjectHandle coll_;
  PoseDisplaySelectionHandlerPtr coll_handler_;

  uint32_t messages_received_;

  Ogre::SceneNode* scene_node_;

  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_;
  tf::MessageFilter<geometry_msgs::PoseStamped>* tf_filter_;
  geometry_msgs::PoseStampedConstPtr latest_message_;

  ROSTopicStringPropertyWPtr topic_property_;
  EnumPropertyWPtr shape_property_;
  CategoryPropertyWPtr shape_category_;

  ColorPropertyWPtr color_property_;
  FloatPropertyWPtr alpha_property_;

  FloatPropertyWPtr head_radius_property_;
  FloatPropertyWPtr head_length_property_;
  FloatPropertyWPtr shaft_radius_property_;
  FloatPropertyWPtr shaft_length_property_;

  FloatPropertyWPtr axes_length_property_;
  FloatPropertyWPtr axes_radius_property_;
};

} // namespace rviz

#endif /* RVIZ_POSE_DISPLAY_H_ */
