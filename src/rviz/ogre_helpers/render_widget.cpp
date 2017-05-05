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

#include "ogre_helpers/render_widget.h"
#include "ogre_helpers/render_system.h"

#include <OgreRenderWindow.h>

#include <QtGlobal>
#include <QApplication>
#include <QMoveEvent>
#include <QPaintEvent>
#include <QShowEvent>
#include <QVBoxLayout>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QWindow>
#endif

namespace rviz
{

RenderWidget::RenderWidget( RenderSystem* render_system, QWidget *parent )
  : QWidget( parent )
  , render_system_( render_system )
  , render_window_( 0 )
{
  setAttribute(Qt::WA_OpaquePaintEvent,true);
  setAttribute(Qt::WA_PaintOnScreen,true);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  // It is not clear to me why, but having this frame sub-widget
  // inside the main widget makes an important difference (under X at
  // least).  Without the frame and using this widget's winId()
  // below causes trouble when using RenderWidget as a child
  // widget.  The frame graphics are completely covered up by the 3D
  // render, so using it does not affect the appearance at all.
  this->renderFrame = new QFrame;
  this->renderFrame->setLineWidth(1);
  this->renderFrame->setFrameShadow(QFrame::Sunken);
  this->renderFrame->setFrameShape(QFrame::Box);
  this->renderFrame->show();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setContentsMargins( 0, 0, 0, 0 );
  mainLayout->addWidget(this->renderFrame);
  this->setLayout(mainLayout);
#endif

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  rviz::RenderSystem::WindowIDType win_id = this->renderFrame->winId();
#else
  rviz::RenderSystem::WindowIDType win_id = this->winId();
#endif
  QApplication::flush();

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::syncX();
  double pixel_ratio = 1.0;
#else
  QWindow* window = windowHandle();
  double pixel_ratio = window ? window->devicePixelRatio() : 1.0;
#endif
  render_window_ = render_system_->makeRenderWindow(win_id, width(), height(), pixel_ratio);
}

RenderWidget::~RenderWidget()
{
  if( render_window_ )
  {
    render_window_->removeViewport( 0 );
    render_window_->destroy();
  }

  render_window_ = 0;
}

void RenderWidget::moveEvent(QMoveEvent *e)
{
  QWidget::moveEvent(e);

  if(e->isAccepted() && render_window_)
  {
    render_window_->windowMovedOrResized();
  }
}

void RenderWidget::paintEvent(QPaintEvent *e)
{
  if( render_window_ )
  {
    render_window_->update();
  }
  e->accept();
}

void RenderWidget::resizeEvent(QResizeEvent *e)
{
  if( render_window_ )
  {
    // render_window_->writeContentsToFile() (used in
    // VisualizationFrame::onSaveImage()) does not work right for
    // window with an odd width, so here I just always force it to be
    // even.
    render_window_->resize( width() + (width() % 2), height() );
    render_window_->windowMovedOrResized();
  }
}

} // end namespace rviz
