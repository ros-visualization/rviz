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

#include <QPainter>
#include <QStyleOptionViewItem>
#include <QComboBox>

#include "rviz/properties/enum_item.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_widget.h"

namespace rviz
{

EnumItem::EnumItem( EnumProperty* property )
  : PropertyWidgetItem( property, property->getName(), property->hasSetter() )
  , choice_value_( -93993328 ) // some very unlikely value
  , signal_changes_( true )
{
}

QWidget* EnumItem::createEditor( QWidget* parent, const QStyleOptionViewItem & option )
{
  QComboBox* editor = new QComboBox( parent );
  editor->setFrame( false );
  QObject::connect( editor, SIGNAL( currentIndexChanged( int )), this, SLOT( setChoiceIndex( int )));
  return editor;
}

bool EnumItem::setEditorData( QWidget* editor )
{
  if( QComboBox* enum_editor = qobject_cast<QComboBox*>( editor ))
  {
    signal_changes_ = false;
    enum_editor->clear();
    int index = 0;
    int chosen_index = -1;
    for( Choices::iterator ci = choices_.begin(); ci != choices_.end(); ci++ )
    {
      enum_editor->addItem( QString::fromStdString( (*ci).first ), (*ci).second );
      if( (*ci).second == choice_value_ )
      {
        chosen_index = index;
      }
      index++;
    }
    enum_editor->setCurrentIndex( chosen_index );
    signal_changes_ = true;
    return true;
  }
  return false;
}

bool EnumItem::setModelData( QWidget* editor )
{
  if( QComboBox* enum_editor = qobject_cast<QComboBox*>( editor ))
  {
    setChoiceIndex( enum_editor->currentIndex() );
    return true;
  }
  return false;
}

void EnumItem::setChoiceIndex( int index )
{
  if( signal_changes_ && index >= 0 && index < (int) choices_.size() )
  {
    int new_choice = choices_[ index ].second;
    if( new_choice != choice_value_ )
    {
      setChoiceValue( new_choice );
      emitDataChanged();
    }
  }
}

std::string EnumItem::currentChoiceName()
{
  for( Choices::iterator ci = choices_.begin(); ci != choices_.end(); ci++ )
  {
    if( (*ci).second == choice_value_ )
    {
      return (*ci).first;
    }
  }
  return "";
}

void EnumItem::setChoices( Choices choices )
{
  if( choices_ != choices )
  {
    choices_ = choices;
    setRightText( currentChoiceName() );
  }
}

void EnumItem::setChoiceValue( int choice_value )
{
  if( choice_value_ != choice_value )
  {
    choice_value_ = choice_value;
    setRightText( currentChoiceName() );
  }
}

} // end namespace rviz
