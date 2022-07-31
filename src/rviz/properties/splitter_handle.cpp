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
#include <QAction>
#include <QDebug>

#include "rviz/properties/splitter_handle.h"

namespace rviz
{
SplitterHandle::SplitterHandle(QTreeView* parent)
  : QWidget(parent), parent_(parent), first_column_size_ratio_(0.5f), color_(128, 128, 128, 64)
{
  setCursor(Qt::SplitHCursor);
  updateGeometry();
  parent_->header()->setStretchLastSection(false);
  parent_->installEventFilter(this);

  parent->setStyleSheet("border-width: 1 1 1 10; border-style: solid; border-color: magenta");
  auto act = new QAction("Update Geometry", this);
  act->setShortcut(QKeySequence("F5"));
  act->setShortcutContext(Qt::WidgetShortcut);
  parent->addAction(act);
  connect(act, &QAction::triggered, this, &SplitterHandle::updateGeometry);
}

bool SplitterHandle::eventFilter(QObject* event_target, QEvent* event)
{
  if (event_target == parent_ && event->type() == QEvent::Resize)
  {
    updateGeometry();
  }
  return false; // Return false regardless so resize event also does its normal job.
}

void SplitterHandle::updateGeometry()
{
  int w = 7;
  const auto& content = parent_->contentsRect();
  int new_column_width = int(first_column_size_ratio_ * content.width());
  qDebug() << parent_->contentsRect() << parent_->contentsRect().width() << parent_->width()
           << new_column_width;
  parent_->header()->resizeSection(0, new_column_width); // fixed size for name column
  parent_->header()->resizeSection(1, content.width() - new_column_width);

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
  const auto& content = parent_->contentsRect();
  int new_column_width = qBound(parent_->header()->minimumSectionSize(), // minimum
                                width,                                   // desired
                                content.width());                        // maximum

  if (new_column_width != parent_->header()->sectionSize(0))
  {
    first_column_size_ratio_ = new_column_width / (float)content.width();
    updateGeometry();
  }
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

void SplitterHandle::paintEvent(QPaintEvent* /*event*/)
{
  QPainter painter(this);
  painter.fillRect(0, 0, width(), height(), Qt::yellow);
  painter.setPen(color_);
  painter.drawLine(width() / 2, 0, width() / 2, height());
}

} // end namespace rviz
