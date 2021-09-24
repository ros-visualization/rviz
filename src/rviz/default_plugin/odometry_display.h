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
#endif

#include <rviz/message_filter_display.h>
#include <nav_msgs/Odometry.h>

namespace rviz
{
class Arrow;
class Axes;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;

class CovarianceProperty;

/**
 * \class OdometryDisplay
 * \brief Accumulates and displays the pose from a nav_msgs::Odometry message
 */
class OdometryDisplay : public rviz::MessageFilterDisplay<nav_msgs::Odometry>
{
  Q_OBJECT
public:
  enum Shape
  {
    ArrowShape,
    AxesShape,
  };

  OdometryDisplay();
  ~OdometryDisplay() override;

  // Overides of MessageFilterDisplay
  void onInitialize() override;
  void reset() override;
  // Overides of Display
  void update(float wall_dt, float ros_dt) override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  void onEnable() override;

private Q_SLOTS:
  void updateShapeChoice();
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateArrowsGeometry();
  void updateAxisGeometry();

private:
  void updateGeometry(rviz::Arrow* arrow);
  void updateGeometry(rviz::Axes* axes);
  void clear();

  void processMessage(const nav_msgs::Odometry::ConstPtr& message) override;

  typedef std::deque<rviz::Arrow*> D_Arrow;
  typedef std::deque<rviz::Axes*> D_Axes;

  D_Arrow arrows_;
  D_Axes axes_;

  nav_msgs::Odometry::ConstPtr last_used_message_;

  rviz::EnumProperty* shape_property_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* position_tolerance_property_;
  rviz::FloatProperty* angle_tolerance_property_;
  rviz::IntProperty* keep_property_;

  rviz::FloatProperty* head_radius_property_;
  rviz::FloatProperty* head_length_property_;
  rviz::FloatProperty* shaft_radius_property_;
  rviz::FloatProperty* shaft_length_property_;

  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;

  CovarianceProperty* covariance_property_;
};

} // namespace rviz

#endif /* RVIZ_ODOMETRY_DISPLAY_H_ */
