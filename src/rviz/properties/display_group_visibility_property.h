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

#ifndef RVIZ_DISPLAY_GROUP_VISIBILITY_PROPERTY_H_
#define RVIZ_DISPLAY_GROUP_VISIBILITY_PROPERTY_H_

#include "display_visibility_property.h"

#include <stdint.h>

#include <map>
#include <string>

#include <QObject>
#include <QString>

namespace rviz
{

class DisplayContext;
class Property;
class BoolProperty;
class DisplayGroup;
class BoolProperty;
class DisplayVisibilityProperty;


/*
 * Manages the visibility of all displays in a display group
 * by switching one bit in Ogre's visibility mask.
 */
class DisplayGroupVisibilityProperty: public DisplayVisibilityProperty
{
  Q_OBJECT
public:

  /* @param parent Parent display (will not get a visibility property)
   * @param context The display context
   * @param root_property Where to attach new properties to
   */
  DisplayGroupVisibilityProperty(
      uint32_t vis_bit,
      DisplayGroup* display_group,
      Display* parent_display,
      const QString& name = QString(),
      bool default_value = false,
      const QString& description = QString(),
      Property* parent = 0,
      const char *changed_slot = 0,
      QObject* receiver = 0 );
  virtual ~DisplayGroupVisibilityProperty();

  // @brief Update visibility masks of all objects in the Ogre scene
  virtual void update();

public Q_SLOTS:

  void onDisplayAdded( rviz::Display* display );
  void onDisplayRemoved( rviz::Display* display );

private:

  // sort the properties in the same way as in the
  // root display group
  void sortDisplayList();

  DisplayGroup* display_group_;
  std::map<rviz::Display*, DisplayVisibilityProperty*> disp_vis_props_;
  Display* parent_display_;
};


}

#endif /* DISPLAY_VISIBILITY_MANAGER_H_ */
