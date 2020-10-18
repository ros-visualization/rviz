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

#include <rviz/properties/splitter_handle.h>

namespace rviz
{
SplitterHandle::SplitterHandle(QTreeView* parent)
  : QWidget(parent), parent_(parent), first_column_size_ratio_(0.5f), color_(128, 128, 128, 64)
{
  setCursor(Qt::SplitHCursor);
  updateGeometry();
  parent_->header()->setStretchLastSection(false);
  parent_->installEventFilter(this);
}

bool SplitterHandle::eventFilter(QObject* event_target, QEvent* event)
{
  if (event_target == parent_ &&
      (event->type() == QEvent::Resize || event->type() == QEvent::LayoutRequest))
  {
    updateGeometry();
  }
  return false; // Return false regardless so resize event also does its normal job.
}

void SplitterHandle::updateGeometry()
{
  int w = 7;
  int new_column_width = int(first_column_size_ratio_ * parent_->contentsRect().width());
  parent_->setColumnWidth(0, new_column_width);
  parent_->setColumnWidth(1, parent_->viewport()->contentsRect().width() - new_column_width);

  int new_x = new_column_width - w / 2 + parent_->columnViewportPosition(0);
  if (new_x != x() || parent_->height() != height())
    setGeometry(new_x, 0, w, parent_->height());
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

void SplitterHandle::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton)
  {
    // position of mouse press inside this QWidget
    x_press_offset_ = event->x();
  }
}

void SplitterHandle::mouseMoveEvent(QMouseEvent* event)
{
  int padding = 55;

  if (event->buttons() & Qt::LeftButton)
  {
    QPoint pos_rel_parent = parent_->mapFromGlobal(event->globalPos());

    int new_x = pos_rel_parent.x() - x_press_offset_ - parent_->columnViewportPosition(0);

    if (new_x > parent_->width() - width() - padding)
    {
      new_x = parent_->width() - width() - padding;
    }

    if (new_x < padding)
    {
      new_x = padding;
    }

    if (new_x != x())
    {
      int new_column_width = new_x + width() / 2 - parent_->contentsRect().x();
      first_column_size_ratio_ = new_column_width / (float)parent_->contentsRect().width();
      updateGeometry();
    }
  }
}

void SplitterHandle::paintEvent(QPaintEvent* /*event*/)
{
  QPainter painter(this);
  painter.setPen(color_);
  painter.drawLine(1 + width() / 2, 0, 1 + width() / 2, height());
}

} // end namespace rviz
