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
#include <QLineEdit>

#include "rviz/properties/edit_enum_item.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_widget.h"

namespace rviz
{

EditEnumItem::EditEnumItem( EditEnumProperty* property )
  : PropertyWidgetItem( property, property->getName(), property->hasSetter() )
  , signal_changes_( true )
{
}

QWidget* EditEnumItem::createEditor( QWidget* parent, const QStyleOptionViewItem & option )
{
  QComboBox* editor = new QComboBox( parent );
  editor->setEditable( true );
  editor->setFrame( false );
  QObject::connect( editor, SIGNAL( currentIndexChanged( int )), this, SLOT( setChoiceIndex( int )));
  return editor;
}

bool EditEnumItem::setEditorData( QWidget* editor )
{
  if( QComboBox* enum_editor = qobject_cast<QComboBox*>( editor ))
  {
    signal_changes_ = false;
    enum_editor->clear();
    if( option_cb_ )
    {
      choices_.clear();
      option_cb_( choices_ );
    }
    int index = 0;
    int chosen_index = -1;
    for( V_string::iterator ci = choices_.begin(); ci != choices_.end(); ci++ )
    {
      enum_editor->addItem( QString::fromStdString( *ci ));
      if( *ci == choice_ )
      {
        chosen_index = index;
      }
      index++;
    }
    if( chosen_index == -1 )
    {
      enum_editor->setEditText( QString::fromStdString( choice_ ));
    }
    else
    {
      enum_editor->setCurrentIndex( chosen_index );
    }
    signal_changes_ = true;
    return true;
  }
  return false;
}

bool EditEnumItem::setModelData( QWidget* editor )
{
  if( QComboBox* enum_editor = qobject_cast<QComboBox*>( editor ))
  {
    std::string new_choice = enum_editor->currentText().toStdString();
    if( choice_ != new_choice )
    {
      choice_ = new_choice;
      // Set the text and emit data-changed.
      setText( 1, QString::fromStdString( choice_ ));
    }
    return true;
  }
  return false;
}

void EditEnumItem::setChoice( const std::string& new_choice )
{
  if( new_choice != choice_ )
  {
    choice_ = new_choice;
    setRightText( choice_ );
  }
}

void EditEnumItem::setChoices( V_string& choices )
{
  if( choices_ != choices )
  {
    choices_ = choices;
    bool ign = getProperty()->getPropertyTreeWidget()->setIgnoreChanges( true );
    emitDataChanged();
    getProperty()->getPropertyTreeWidget()->setIgnoreChanges( ign );
  }
}

void EditEnumItem::setChoiceIndex( int index )
{
  if( signal_changes_ && index >= 0 && index < (int) choices_.size() )
  {
    std::string new_choice = choices_[ index ];
    if( new_choice != choice_ )
    {
      choice_ = new_choice;
      // Set the text and emit data-changed.
      setText( 1, QString::fromStdString( new_choice ));
    }
  }
}

} // end namespace rviz
