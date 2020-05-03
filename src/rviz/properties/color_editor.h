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
#ifndef RVIZ_COLOR_EDITOR_H
#define RVIZ_COLOR_EDITOR_H

#include <rviz/properties/line_edit_with_button.h>

namespace rviz
{
class ColorProperty;

class ColorEditor : public LineEditWithButton
{
  Q_OBJECT
public:
  ColorEditor(ColorProperty* property = nullptr, QWidget* parent = nullptr);

  /** Static function to paint just the color box.  Paints it in the
   * left end of rect, size rect.height() by rect.height(). */
  static void paintColorBox(QPainter* painter, const QRect& rect, const QColor& color);

public Q_SLOTS:
  void setColor(const QColor& color);
  void parseText();

protected:
  /** Call parent version then paint color swatch. */
  void paintEvent(QPaintEvent* event) override;

  void resizeEvent(QResizeEvent* event) override;

protected Q_SLOTS:
  void onButtonClick() override;

private:
  QColor color_;
  ColorProperty* property_;
};

} // end namespace rviz

#endif // RVIZ_COLOR_EDITOR_H
