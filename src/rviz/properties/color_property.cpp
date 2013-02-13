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

#include <QPainter>
#include <QStringList>
#include <QStyleOptionViewItem>

#include "rviz/properties/parse_color.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/color_editor.h"

namespace rviz
{

ColorProperty::ColorProperty( const QString& name,
                              const QColor& default_value,
                              const QString& description,
                              Property* parent,
                              const char *changed_slot,
                              QObject* receiver )
  : Property( name, QVariant(), description, parent, changed_slot, receiver )
  , color_( default_value )
{
  updateString();
}

bool ColorProperty::setColor( const QColor& new_color )
{
  if( new_color != color_ ) {
    Q_EMIT aboutToChange();
    color_ = new_color;
    updateString();
    Q_EMIT changed();
    return true;
  }
  return false;
}

bool ColorProperty::setValue( const QVariant& new_value )
{
  if( new_value.type() == QVariant::Color )
  {
    return setColor( new_value.value<QColor>() );
  }

  QColor new_color = parseColor( new_value.toString() );
  if( new_color.isValid() )
  {
    return setColor( new_color );
  }
  return false;
}

void ColorProperty::updateString()
{
  value_ = printColor( color_ );
}

bool ColorProperty::paint( QPainter * painter,
                           const QStyleOptionViewItem & option ) const
{
  painter->save();
  QColor color = color_;
  if ( !(getViewFlags( 0 ) & Qt::ItemIsEnabled) )
  {
    color = QColor( 200, 200, 200 );
    painter->setPen( QColor( Qt::lightGray ) );
  }
  QString text = value_.toString();
  QRect rect = option.rect;
  ColorEditor::paintColorBox( painter, rect, color );
  rect.adjust( rect.height() + 4, 1, 0, 0 );
  painter->drawText( rect, text );

  painter->restore();

  return true; // return true, since this function has done the painting.
}

QWidget *ColorProperty::createEditor( QWidget* parent,
                                      const QStyleOptionViewItem& option )
{
  ColorEditor* editor = new ColorEditor( this, parent );
  editor->setFrame( false );
  return editor;
}

} // end namespace rviz
