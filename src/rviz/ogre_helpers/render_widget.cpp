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

#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/ogre_helpers/version_check.h>

#include <OGRE/OgreRenderWindow.h>

#include <QtGlobal>
#include <QApplication>
#include <QMoveEvent>
#include <QPaintEvent>
#include <QShowEvent>
#include <QVBoxLayout>
#include <QWindow>

namespace rviz
{
RenderWidget::RenderWidget(RenderSystem* render_system, QWidget* parent)
  : QWidget(parent), render_system_(render_system), render_window_(nullptr)
{
  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);

  rviz::RenderSystem::WindowIDType win_id = this->winId();
  QWindow* window = windowHandle();
  pixel_ratio_ = window ? window->devicePixelRatio() : 1.0;

  render_window_ = render_system_->makeRenderWindow(win_id, width(), height(), pixel_ratio_);
}

RenderWidget::~RenderWidget()
{
  if (render_window_)
  {
    render_window_->removeViewport(0);
    render_window_->destroy();
  }

  render_window_ = nullptr;
}

void RenderWidget::moveEvent(QMoveEvent* e)
{
  QWidget::moveEvent(e);

  if (e->isAccepted() && render_window_)
  {
    render_window_->windowMovedOrResized();
  }
}

void RenderWidget::paintEvent(QPaintEvent* e)
{
  if (render_window_)
  {
    render_window_->update();
  }
  e->accept();
}

void RenderWidget::resizeEvent(QResizeEvent* e)
{
  QWidget::resizeEvent(e);
  if (e->isAccepted() && render_window_)
  {
    /* render_window_->writeContentsToFile() (used in VisualizationFrame::onSaveImage())
     * does not work right for window with an odd width.
     * So here we just always force it to be even. */
    const int w = width() * pixel_ratio_;
    render_window_->resize(w + (w % 2), height() * pixel_ratio_);
#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 12, 0)
    render_window_->windowMovedOrResized();
#endif
  }
}

} // end namespace rviz
