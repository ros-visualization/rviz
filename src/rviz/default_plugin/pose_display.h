/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/message_filter_display.h"
#include "rviz/selection/forwards.h"

namespace rviz
{
class Arrow;
class Axes;
class ColorProperty;
class EnumProperty;
class FloatProperty;
class Shape;

class PoseDisplaySelectionHandler;
typedef boost::shared_ptr<PoseDisplaySelectionHandler> PoseDisplaySelectionHandlerPtr;

/** @brief Accumulates and displays the pose from a geometry_msgs::PoseStamped message. */
class PoseDisplay : public MessageFilterDisplay<geometry_msgs::PoseStamped>
{
  Q_OBJECT
public:
  enum Shape
  {
    Arrow,
    Axes,
  };

  PoseDisplay();
  ~PoseDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get arrow/axes visibility correct. */
  void onEnable() override;

private Q_SLOTS:
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateShapeChoice();
  void updateAxisGeometry();
  void updateArrowGeometry();

private:
  void clear();

  void processMessage(const geometry_msgs::PoseStamped::ConstPtr& message) override;

  rviz::Arrow* arrow_;
  rviz::Axes* axes_;
  bool pose_valid_;
  PoseDisplaySelectionHandlerPtr coll_handler_;

  EnumProperty* shape_property_;

  ColorProperty* color_property_;
  FloatProperty* alpha_property_;

  FloatProperty* head_radius_property_;
  FloatProperty* head_length_property_;
  FloatProperty* shaft_radius_property_;
  FloatProperty* shaft_length_property_;

  FloatProperty* axes_length_property_;
  FloatProperty* axes_radius_property_;

  friend class PoseDisplaySelectionHandler;
};

} // namespace rviz

#endif /* RVIZ_POSE_DISPLAY_H_ */
