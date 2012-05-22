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


#ifndef RVIZ_POLYGON_DISPLAY_H
#define RVIZ_POLYGON_DISPLAY_H

#include <geometry_msgs/PolygonStamped.h>

#include "rviz/message_filter_display.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;

/**
 * \class PolygonDisplay
 * \brief Displays a geometry_msgs::PolygonStamped message
 */
class PolygonDisplay: public MessageFilterDisplay<geometry_msgs::PolygonStamped>
{
Q_OBJECT
public:
  PolygonDisplay();
  virtual ~PolygonDisplay();

  /** @brief Overridden from MessageFilterDisplay. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  virtual void reset();

protected:
  /** @brief Overridden from MessageFilterDisplay. */
  virtual void processMessage( const geometry_msgs::PolygonStamped::ConstPtr& msg );

  Ogre::ManualObject* manual_object_;

  ColorProperty* color_property_;
  FloatProperty* alpha_property_;
};

} // namespace rviz

#endif /* RVIZ_POLYGON_DISPLAY_H */

