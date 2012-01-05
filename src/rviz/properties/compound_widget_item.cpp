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

#include <ros/ros.h> // just for ROS_ASSERT

#include <QLineEdit>

#include "rviz/properties/compound_widget_item.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_widget.h"

namespace rviz
{

CompoundWidgetItem::CompoundWidgetItem( PropertyBase* property,
                                        const std::string& label,
                                        bool editable )
  : PropertyWidgetItem( property, label, editable )
{
}

/** Call this when a child has changed and this item should update
 * its text from them.  Does not trigger data-changed signal. */
void CompoundWidgetItem::updateText()
{
  if( childCount() > 0 )
  {
    QString new_text = child( 0 )->data( 1, Qt::UserRole ).toString();

    for( int i = 1; i < childCount(); i++ )
    {
      new_text += "; " + child( i )->data( 1, Qt::UserRole ).toString();
    }

    bool ign = getProperty()->getPropertyTreeWidget()->setIgnoreChanges( true );
    setText( 1, new_text );
    setData( 1, Qt::UserRole, new_text );
    getProperty()->getPropertyTreeWidget()->setIgnoreChanges( ign );
  }
}

/** Overridden from PropertyWidgetItem.  Updates the child items
 * from the string in this item. */
bool CompoundWidgetItem::setModelData( QWidget* editor )
{
  QLineEdit* line_edit = qobject_cast<QLineEdit *>( editor );
  ROS_ASSERT( line_edit );

  if( !line_edit->isModified() )
  {
    return true;
  }

  bool ign = getProperty()->getPropertyTreeWidget()->setIgnoreChanges( true );

  QStringList texts = line_edit->text().split( QRegExp( "\\s*;\\s*" ));
  for( int i = 0; i < childCount() && i < texts.size(); i++ )
  {
    QVariant orig_data = child( i )->data( 1, Qt::UserRole );
    QVariant new_data = texts.at( i );
    new_data.convert( orig_data.type() );
    child( i )->setData( 1, Qt::UserRole, new_data );
    child( i )->setText( 1, new_data.toString() );
  }

  getProperty()->getPropertyTreeWidget()->setIgnoreChanges( ign );

  emitDataChanged();

  return true;
}

} // end namespace rviz
