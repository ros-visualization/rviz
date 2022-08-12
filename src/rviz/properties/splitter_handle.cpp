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

#include <QEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QTreeView>
#include <QHeaderView>
#include <QScrollBar>

#include <rviz/properties/splitter_handle.h>

namespace rviz
{
SplitterHandle::SplitterHandle(QTreeView* parent)
  : QWidget(parent), parent_(parent), first_column_size_ratio_(0.5f), color_(128, 128, 128, 64)
{
  setCursor(Qt::SplitHCursor);
  parent_->header()->setStretchLastSection(false);
  parent_->viewport()->installEventFilter(this);
  updateGeometry();
}

bool SplitterHandle::eventFilter(QObject* /*event_target*/, QEvent* event)
{
  if (event->type() == QEvent::Resize)
  {
    updateGeometry();
  }
  return false; // Return false regardless so resize event also does its normal job.
}

void SplitterHandle::updateGeometry()
{
  auto* header = parent_->header();
  const int sbw = parent_->verticalScrollBar()->isVisible() ? 0 : parent_->verticalScrollBar()->width();
  const int w = 7;
  const int min = header->minimumSectionSize();
  const int available = parent_->viewport()->contentsRect().width();
  const auto& content = parent_->contentsRect();

  // determine new width of first column (w.r.t. overall content width!)
  int new_column_width = int(first_column_size_ratio_ * content.width());
  if (new_column_width <= 0)
    new_column_width = 0;
  else if (new_column_width >= available)
    new_column_width = available;
  else
    new_column_width = qBound(min,                    // minimum
                              new_column_width,       // desired
                              available - min - sbw); // maximum

  if (new_column_width > header->sectionSize(0))
  { // decrease 2nd column before increasing 1st one
    header->resizeSection(1, available - new_column_width);
    header->resizeSection(0, new_column_width);
  }
  else
  { // decrease 1st column before increasing 2nd one
    header->resizeSection(0, new_column_width);
    header->resizeSection(1, available - new_column_width);
  }

  // update geometry + position of splitter itself
  int new_x = content.x() + new_column_width - w / 2;
  if (new_x != x() || content.height() != height())
    setGeometry(new_x, content.y(), w, content.height());
}

void SplitterHandle::setRatio(float ratio)
{
  first_column_size_ratio_ = ratio;
  updateGeometry();
}

float SplitterHandle::getRatio()
{
  return first_column_size_ratio_;
}

void SplitterHandle::setDesiredWidth(int width)
{
  first_column_size_ratio_ = width / (float)parent_->contentsRect().width();
  updateGeometry();
}

void SplitterHandle::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton)
  {
    // position of mouse press relative to splitter line / the center of the widget
    x_press_offset_ = event->x() - width() / 2;
  }
}

void SplitterHandle::mouseMoveEvent(QMouseEvent* event)
{
  if (event->buttons() & Qt::LeftButton)
  {
    QPoint pos_rel_parent = parent_->mapFromGlobal(event->globalPos());
    setDesiredWidth(pos_rel_parent.x() - parent_->contentsRect().x() - x_press_offset_);
  }
}

// adjust splitter position to optimally fit content
void SplitterHandle::mouseDoubleClickEvent(QMouseEvent* /*event*/)
{
  int available_width = parent_->contentsRect().width();
  int default_width = 0.5f * available_width;
  // missing width to default
  int col0 = static_cast<QAbstractItemView*>(parent_)->sizeHintForColumn(0) - default_width;
  int col1 = static_cast<QAbstractItemView*>(parent_)->sizeHintForColumn(1) - default_width;

  if (col0 <= 0 && col1 <= 0) // each column fits
    setDesiredWidth(default_width);
  else if (col0 + col1 <= 0) // both columns fit together, but require a non-default splitting
    setDesiredWidth(default_width + col0 + 0.5f * std::abs(col0 + col1)); // uniformly split extra space
  else
    setDesiredWidth(default_width + col0 - 0.5f * (col0 + col1)); // uniformly cut missing space
}

void SplitterHandle::paintEvent(QPaintEvent* /*event*/)
{
  QPainter painter(this);
  painter.setPen(color_);
  painter.drawLine(width() / 2, 0, width() / 2, height());
}

} // end namespace rviz
