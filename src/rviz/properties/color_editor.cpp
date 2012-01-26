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

#include <stdio.h>

#include <QPainter>
#include <QColorDialog>

#include "rviz/properties/color_editor.h"

namespace rviz
{

ColorEditor::ColorEditor( QWidget* parent )
  : LineEditWithButton( parent )
{
  connect( this, SIGNAL( textEdited( const QString& )),
           this, SLOT( invalidateParse() ));
}

void ColorEditor::paintEvent( QPaintEvent* event )
{
  LineEditWithButton::paintEvent( event );
  QPainter painter( this );
  paintColorBox( &painter, rect(), color_ );
}

void ColorEditor::paintColorBox( QPainter* painter, const QRect& rect, const QColor& color )
{
  int padding = 1;
  int size = rect.height() - padding * 2 - 1;
  painter->save();
  painter->setPen( Qt::black );
  painter->setBrush( color );
  painter->drawRect( rect.x() + padding, rect.y() + padding, size, size );
  painter->restore();
}

void ColorEditor::resizeEvent( QResizeEvent* event )
{
  // Do the normal line-edit-with-button thing
  LineEditWithButton::resizeEvent( event );

  // Then add text padding on the left to make room for the color swatch
  QMargins marge = textMargins();
  setTextMargins( height(), marge.top(), marge.right(), marge.bottom() );
}

void ColorEditor::invalidateParse()
{
  parse_valid_ = false;
}

void ColorEditor::parseText()
{
  // Check for a color name like "black", "red", etc.
  if( QColor::colorNames().contains( text(), Qt::CaseInsensitive ))
  {
    color_.setNamedColor( text().toLower() );
    parse_valid_ = true;
    return;
  }
  // Next look for comma-separated list of ints.
  else
  {
    QStringList list = text().split(QRegExp("[,;]\\s*"));
    if( list.size() >= 3 )
    {
      bool red_ok, green_ok, blue_ok;
      QColor new_color( list.at( 0 ).toInt( &red_ok ), list.at( 1 ).toInt( &green_ok ), list.at( 2 ).toInt( &blue_ok ));
      if( red_ok && green_ok && blue_ok )
      {
        color_ = new_color;
        parse_valid_ = true;
        return;
      }
    }
  }
  // Finally, if we can't figure out the string, leave the color
  // unchanged and reset the text.
  setColor( color_ );
}

void ColorEditor::setColor( const QColor& color )
{
  color_ = color;
  setText( QString("%1, %2, %3").arg( color.red() ).arg( color.green() ).arg( color.blue() ) );
  parse_valid_ = true;
}

QColor ColorEditor::getColor()
{
  if( !parse_valid_ )
  {
    parseText();
  }
  return color_;
}

void ColorEditor::onButtonClick()
{
  Q_EMIT startPersistence();
  QColorDialog* dialog = new QColorDialog( color_, this );
  connect( dialog, SIGNAL( colorSelected( const QColor& )),
           this, SLOT( setColor( const QColor& )));
  if( dialog->exec() == QDialog::Accepted )
  {
    update();
    setModified( true );
  }
  Q_EMIT endPersistence();
}

} // end namespace rviz
