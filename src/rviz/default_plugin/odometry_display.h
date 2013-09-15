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


#ifndef RVIZ_ODOMETRY_DISPLAY_H_
#define RVIZ_ODOMETRY_DISPLAY_H_

#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include <nav_msgs/Odometry.h>

#include "rviz/display.h"

namespace rviz
{

class Arrow;
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;

/**
 * \class OdometryDisplay
 * \brief Accumulates and displays the pose from a nav_msgs::Odometry message
 */
class OdometryDisplay: public Display
{
Q_OBJECT
public:
  OdometryDisplay();
  virtual ~OdometryDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  virtual void setTopic( const QString &topic, const QString &datatype );

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void updateColor();
  void updateTopic();
  void updateLength();

private:
  void subscribe();
  void unsubscribe();
  void clear();

  void incomingMessage( const nav_msgs::Odometry::ConstPtr& message );
  void transformArrow( const nav_msgs::Odometry::ConstPtr& message, Arrow* arrow );

  typedef std::deque<Arrow*> D_Arrow;
  D_Arrow arrows_;

  uint32_t messages_received_;

  nav_msgs::Odometry::ConstPtr last_used_message_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_;
  tf::MessageFilter<nav_msgs::Odometry>* tf_filter_;

  ColorProperty* color_property_;
  RosTopicProperty* topic_property_;
  FloatProperty* position_tolerance_property_;
  FloatProperty* angle_tolerance_property_;
  IntProperty* keep_property_;
  FloatProperty* length_property_;
};

} // namespace rviz

#endif /* RVIZ_ODOMETRY_DISPLAY_H_ */
