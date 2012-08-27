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

#ifndef RVIZ_DISPLAY_VISIBILITY_MANAGER_H_
#define RVIZ_DISPLAY_VISIBILITY_MANAGER_H_

#include <stdint.h>

#include <map>
#include <string>

#include <QObject>
#include <QString>

namespace rviz
{

class DisplayContext;
class Property;
class Display;
class BoolProperty;
class DisplayVisibilityContext;

class DisplayVisibilityManager
{
public:
  DisplayVisibilityManager( Display* parent, DisplayContext* context, Property* root_property );
  virtual ~DisplayVisibilityManager();

  uint32_t getVisBit() { return vis_bit_; }
private:
  DisplayContext* context_;
  uint32_t vis_bit_;
  Property* root_property_;
  std::map<rviz::Display*, DisplayVisibilityContext*> contexts_;
};

class DisplayVisibilityContext : public QObject
{
Q_OBJECT
public:
  DisplayVisibilityContext( uint32_t vis_bit, Display* display, Property* root_property );
  virtual ~DisplayVisibilityContext();

public Q_SLOTS:

  /** @brief Update whether the display is visible or not. */
  void visibilityChanged();

private:
  uint32_t vis_bit_;
  Display* display_;
  BoolProperty* enabled_prop_;
};

}

#endif /* DISPLAY_VISIBILITY_MANAGER_H_ */
