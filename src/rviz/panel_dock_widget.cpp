/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <QApplication>
#include <QChildEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QToolButton>
#include <QCloseEvent>

#include <rviz/panel_dock_widget.h>

namespace rviz
{
PanelDockWidget::PanelDockWidget(const QString& name)
  : QDockWidget(name), collapsed_(false), forced_hidden_(false)
{
  QWidget* title_bar = new QWidget(this);

  QPalette pal(palette());
  pal.setColor(QPalette::Background, QApplication::palette().color(QPalette::Mid));
  title_bar->setAutoFillBackground(true);
  title_bar->setPalette(pal);
  title_bar->setContentsMargins(0, 0, 0, 0);

  QToolButton* close_button = new QToolButton();
  close_button->setIcon(QIcon::fromTheme("window-close"));
  close_button->setIconSize(QSize(10, 10));

  connect(close_button, SIGNAL(clicked()), this, SLOT(close()));

  title_label_ = new QLabel(name, this);

  icon_label_ = new QLabel(this);
  icon_label_->setContentsMargins(2, 2, 0, 0);
  setIcon(QIcon());

  QHBoxLayout* title_layout = new QHBoxLayout();
  title_layout->setContentsMargins(2, 2, 2, 2);
  title_layout->addWidget(icon_label_, 0);
  title_layout->addWidget(title_label_, 1);
  title_layout->addWidget(close_button, 0);
  title_bar->setLayout(title_layout);
  setTitleBarWidget(title_bar);
}

void PanelDockWidget::setWindowTitle(const QString& title)
{
  QDockWidget::setWindowTitle(title);
  title_label_->setText(title);
}


void PanelDockWidget::setIcon(const QIcon& icon)
{
  if (icon.isNull())
  {
    icon_label_->setVisible(false);
  }
  else
  {
    icon_label_->setVisible(true);
    icon_label_->setPixmap(icon.pixmap(16, 16));
  }
}

void PanelDockWidget::setCollapsed(bool collapse)
{
  if (collapsed_ == collapse || isFloating())
    return;


  if (collapse)
  {
    if (isVisible())
    {
      PanelDockWidget::setVisible(false);
      collapsed_ = collapse;
    }
  }
  else
  {
    PanelDockWidget::setVisible(true);
    collapsed_ = collapse;
  }
}

void PanelDockWidget::setContentWidget(QWidget* child)
{
  if (widget())
  {
    disconnect(widget(), SIGNAL(destroyed(QObject*)), this, SLOT(onChildDestroyed(QObject*)));
  }
  setWidget(child);
  if (child)
  {
    connect(child, SIGNAL(destroyed(QObject*)), this, SLOT(onChildDestroyed(QObject*)));
  }
}

void PanelDockWidget::closeEvent(QCloseEvent* /*event*/)
{
  Q_EMIT closed();
}

void PanelDockWidget::onChildDestroyed(QObject* /*unused*/)
{
  deleteLater();
}

void PanelDockWidget::save(Config config)
{
  config.mapSetValue("collapsed", collapsed_);
}

void PanelDockWidget::load(const Config& config)
{
  config.mapGetBool("collapsed", &collapsed_);
}

void PanelDockWidget::setVisible(bool visible)
{
  requested_visibility_ = visible;
  QDockWidget::setVisible(requested_visibility_ && !forced_hidden_);
}

void PanelDockWidget::overrideVisibility(bool hidden)
{
  forced_hidden_ = hidden;
  setVisible(requested_visibility_);
}

} // end namespace rviz
