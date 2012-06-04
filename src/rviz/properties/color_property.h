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
#ifndef COLOR_PROPERTY_H
#define COLOR_PROPERTY_H

#include <QColor>

#include "rviz/properties/parse_color.h"

#include "rviz/properties/property.h"

namespace rviz
{

class ColorProperty: public Property
{
Q_OBJECT
public:
  ColorProperty( const QString& name = QString(),
                 const QColor& default_value = Qt::black,
                 const QString& description = QString(),
                 Property* parent = 0,
                 const char *changed_slot = 0,
                 QObject* receiver = 0 );

  virtual bool setValue( const QVariant& new_value );

  virtual bool paint( QPainter* painter,
                      const QStyleOptionViewItem& option ) const;

  virtual QWidget* createEditor( QWidget* parent,
                                 const QStyleOptionViewItem& option );

  virtual QColor getColor() const { return color_; }
  Ogre::ColourValue getOgreColor() const { return qtToOgre( color_ ); }

public Q_SLOTS:
  virtual bool setColor( const QColor& color );

private:
  void updateString();

  QColor color_;
};

} // end namespace rviz

#endif // VECTOR_PROPERTY_H
