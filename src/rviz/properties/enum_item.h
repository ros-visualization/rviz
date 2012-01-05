/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef RVIZ_ENUM_ITEM_H
#define RVIZ_ENUM_ITEM_H

#include <QObject>

#include "rviz/properties/property_widget_item.h"
#include "rviz/properties/property.h"

namespace rviz
{

class EnumItem: public QObject, public PropertyWidgetItem
{
Q_OBJECT
public:
  EnumItem( EnumProperty* property );

  // Overrides from PropertyWidgetItem
  virtual QWidget* createEditor( QWidget* parent, const QStyleOptionViewItem & option );

  virtual bool setEditorData( QWidget* editor );

  virtual bool setModelData( QWidget* editor );

  // Enum-specific functions

  /** Set the list of choice names and values. */
  void setChoices( Choices choices );

  /** Set the value of the current choice.  This finds the
   * corresponding choice name and makes it show up in the tree. */
  void setChoiceValue( int choice_value );

  int getChoiceValue() { return choice_value_; }

private Q_SLOTS:
  /** Set a new choice by index into choices_.  Called from user GUI
   * events, so emits data-changed signal. */
  void setChoiceIndex( int new_choice_index );

private:
  std::string currentChoiceName();

  Choices choices_;
  int choice_value_;
  bool signal_changes_;
};

} // end namespace rviz

#endif // RVIZ_ENUM_ITEM_H
