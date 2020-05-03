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
#ifndef SPLITTER_HANDLE_H
#define SPLITTER_HANDLE_H

#include <QWidget>

class QTreeView;

namespace rviz
{
/** @brief A tall skinny invisible widget providing left-right sliding
 * column separator adjustment for a two-column QTreeView via mouse
 * drags.  Shows splitter cursor when mouse hovers over it.  Uses
 * event filtering to catch resize events for the parent. */
class SplitterHandle : public QWidget
{
  Q_OBJECT
public:
  SplitterHandle(QTreeView* parent = nullptr);

  /** @brief Set the ratio of the parent's left column to the parent widget width. */
  void setRatio(float ratio);

  /** @brief Get the ratio of the parent's left column to the parent widget width. */
  float getRatio();

  /** @brief Catch resize events sent to parent to update splitter's
   * geometry.  Always returns false. */
  bool eventFilter(QObject* event_target, QEvent* event) override;

  void setColor(QColor color)
  {
    color_ = color;
    update();
  }
  QColor getColor() const
  {
    return color_;
  }

protected:
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

private:
  /** @brief Update the parent's column widths and this splitter's
   * geometry based on first_column_size_ratio_. */
  void updateGeometry();

  QTreeView* parent_;
  int x_press_offset_;

  /** The ratio of the first column width to the entire widget width.
   * Preserved during parent widget resize. */
  float first_column_size_ratio_;
  QColor color_;
};

} // end namespace rviz

#endif // SPLITTER_HANDLE_H
