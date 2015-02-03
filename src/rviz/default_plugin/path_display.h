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

#include <nav_msgs/Path.h>

#include "rviz/message_filter_display.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;


/**
 * \class PathDisplay
 * \brief Displays a nav_msgs::Path message
 */
class PathDisplay: public MessageFilterDisplay<nav_msgs::Path>
{
Q_OBJECT
public:
  PathDisplay();
  virtual ~PathDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage( const nav_msgs::Path::ConstPtr& msg );

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();

private:
  void destroyObjects();

  std::vector<Ogre::ManualObject*> manual_objects_;
  std::vector<rviz::BillboardLine*> billboard_lines_;

  EnumProperty* style_property_;
  ColorProperty* color_property_;
  FloatProperty* alpha_property_;
  FloatProperty* line_width_property_;
  IntProperty* buffer_length_property_;
  VectorProperty* offset_property_;

  enum LineStyle {
    LINES,
    BILLBOARDS
  };

};

} // namespace rviz

#endif /* RVIZ_PATH_DISPLAY_H */

