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

#ifndef RVIZ_SCALED_IMAGE_WIDGET_H
#define RVIZ_SCALED_IMAGE_WIDGET_H

#include <QWidget>

namespace rviz
{

/**
 * \brief A widget for showing a scaled version of an image (QPixmap).
 *
 * The scale is just a suggestion, given to Qt by calls to sizeHint(),
 * which returns the image size multiplied by the scale.  The actual
 * rendered size is the largest that fits the image into the current
 * widget size without changing the aspect ratio.  It is always
 * rendered in the center.
 */
class ScaledImageWidget: public QWidget
{
Q_OBJECT
public:
  ScaledImageWidget( float scale, QWidget* parent = 0 );
  virtual ~ScaledImageWidget() {}

  void setImage( QPixmap image );

  virtual QSize sizeHint() const;

protected:
  virtual void paintEvent( QPaintEvent* event );

private:
  QPixmap image_;
  float scale_;
};

} // namespace rviz

#endif // RVIZ_SCALED_IMAGE_WIDGET_H
