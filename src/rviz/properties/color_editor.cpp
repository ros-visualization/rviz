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
#include <QMetaObject>
#include <QMetaProperty>

#include <QPainter>
#include <QColorDialog>

#include <rviz/properties/color_property.h>
#include <rviz/properties/parse_color.h>

#include <rviz/properties/color_editor.h>

namespace rviz
{
ColorEditor::ColorEditor(ColorProperty* property, QWidget* parent)
  : LineEditWithButton(parent), property_(property)
{
  connect(this, SIGNAL(textChanged(const QString&)), this, SLOT(parseText()));
}

void ColorEditor::paintEvent(QPaintEvent* event)
{
  LineEditWithButton::paintEvent(event);
  QPainter painter(this);
  painter.setPen(Qt::black);
  paintColorBox(&painter, rect(), color_);
}

void ColorEditor::paintColorBox(QPainter* painter, const QRect& rect, const QColor& color)
{
  int padding = 3;
  int size = rect.height() - padding * 2 - 1;
  painter->save();
  painter->setBrush(color);
  painter->drawRoundedRect(rect.x() + padding + 3, rect.y() + padding, size, size, 0, 0,
                           Qt::AbsoluteSize);
  painter->restore();
}

void ColorEditor::resizeEvent(QResizeEvent* event)
{
  // Do the normal line-edit-with-button thing
  LineEditWithButton::resizeEvent(event);

  // Then add text padding on the left to make room for the color swatch
  QMargins marge = textMargins();
  setTextMargins(height(), marge.top(), marge.right(), marge.bottom());
}

void ColorEditor::parseText()
{
  QColor new_color = parseColor(text());
  if (new_color.isValid())
  {
    color_ = new_color;
    if (property_)
    {
      property_->setColor(new_color);
    }
  }
}

void ColorEditor::setColor(const QColor& color)
{
  color_ = color;
  setText(printColor(color));
  if (property_)
  {
    property_->setColor(color);
  }
}

void ColorEditor::onButtonClick()
{
  ColorProperty* prop = property_;
  QColor original_color = prop->getColor();

  QColorDialog* dialog = new QColorDialog(color_, parentWidget());

  connect(dialog, SIGNAL(currentColorChanged(const QColor&)), property_, SLOT(setColor(const QColor&)));

  // Without this connection the PropertyTreeWidget does not update
  // the color info "live" when it changes in the dialog and the 3D
  // view.
  connect(dialog, SIGNAL(currentColorChanged(const QColor&)), parentWidget(), SLOT(update()));

  // On the TWM window manager under linux, and on OSX, this
  // ColorEditor object is destroyed when (or soon after) the dialog
  // opens.  Therefore, here we delete this ColorEditor immediately to
  // force them all to act the same.
  deleteLater();

  // dialog->exec() will call an event loop internally, so
  // deleteLater() will take effect and "this" will be destroyed.
  // Therefore, everything we do in this function after dialog->exec()
  // should only use variables on the stack, not member variables.
  if (dialog->exec() != QDialog::Accepted)
  {
    prop->setColor(original_color);
  }
}

} // end namespace rviz
