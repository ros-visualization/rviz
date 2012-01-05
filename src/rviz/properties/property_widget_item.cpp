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

#include <float.h>

#include "rviz/properties/property_widget_item.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/properties/property.h"
#include "rviz/properties/topic_info_variant.h"

namespace rviz
{

PropertyWidgetItem::PropertyWidgetItem( PropertyBase* property,
                                        const std::string& label,
                                        bool editable,
                                        bool needs_check_box,
                                        bool first_column_spans )
  : max_( FLT_MAX )
  , min_( -FLT_MAX )
  , property_( property )
  , first_column_spans_( first_column_spans )
{
  Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;

  if( needs_check_box && editable )
  {
    flags |= Qt::ItemIsUserCheckable;
  }

  // "ItemIsEditable" for a checkbox means ability to edit a text
  // label adjacent to it.  Not what we want.
  if( !needs_check_box && editable )
  {
    flags |= Qt::ItemIsEditable;
  }

  setFlags( flags );

  setLeftText( label ); // Set the label in the left column.

  // Setting flags, name, etc (above) must come before adding this as
  // a child of parent with addToParent().  Otherwise those
  // flag-settings etc trigger the itemChanged() signal and the empty
  // data from this item gets written to the Property.
}

void PropertyWidgetItem::addToParent( PropertyWidgetItem* parent_item )
{
  if( parent_item )
  {
    parent_item->addChild( this );
  }
  else
  {
    CategoryPropertyPtr parent = property_->getParent().lock();
    if( parent )
    {
      parent->getWidgetItem()->addChild( this );
    }
    else
    {
      property_->getPropertyTreeWidget()->addTopLevelItem( this );
    }
  }

  // Column-spanning must be done after adding this item to a parent.
  // It does not seem to trigger itemChanged().
  setFirstColumnSpanned( first_column_spans_ );
}

void PropertyWidgetItem::setLeftText( const std::string& text )
{
  bool ign = property_->getPropertyTreeWidget()->setIgnoreChanges( true );

  setText( 0, QString::fromStdString( text ));

  property_->getPropertyTreeWidget()->setIgnoreChanges( ign );
}

void PropertyWidgetItem::setRightText( const QString& text )
{
  bool ign = property_->getPropertyTreeWidget()->setIgnoreChanges( true );

  setText( 1, text );

  property_->getPropertyTreeWidget()->setIgnoreChanges( ign );
}

void PropertyWidgetItem::setRightText( const std::string& text )
{
  setRightText( QString::fromStdString( text ));
}

void PropertyWidgetItem::setUserData( QVariant _data )
{
  bool ign = property_->getPropertyTreeWidget()->setIgnoreChanges( true );

  setData( 1, Qt::UserRole, _data );

  if( _data.canConvert<ros::master::TopicInfo>() )
  {
    ros::master::TopicInfo topic = _data.value<ros::master::TopicInfo>();
    setText( 1, QString::fromStdString( topic.name ));
  }
  else
  {
    setText( 1, _data.toString() );
  }

  property_->getPropertyTreeWidget()->setIgnoreChanges( ign );
}

QVariant PropertyWidgetItem::userData() const
{
  return data( 1, Qt::UserRole );
}

} // end namespace rviz

