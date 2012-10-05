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

#include <stdio.h>

#include <QChildEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QToolButton>
#include <QCloseEvent>

#include "rviz/panel_dock_widget.h"

namespace rviz
{

PanelDockWidget::PanelDockWidget( const QString& name )
  : QDockWidget( name )
  , collapsed_(false)
{
  QWidget *title_bar = new QWidget(this);

  QPalette pal(palette());
  pal.setColor(QPalette::Background, QColor( 200,200,200 ) );
  title_bar->setAutoFillBackground(true);
  title_bar->setPalette(pal);
  title_bar->setContentsMargins(0,0,0,0);

  QToolButton *close_button = new QToolButton();
  close_button->setIcon(QIcon::fromTheme("window-close"));
  close_button->setIconSize( QSize(10,10) );

  connect( close_button, SIGNAL( clicked() ), this, SLOT(close()) );

  QLabel *title_label = new QLabel( name, this );
  icon_label_ = new QLabel( name, this );

  QHBoxLayout *title_layout = new QHBoxLayout();
  title_layout->setContentsMargins(2,2,2,2);
  title_layout->addWidget( icon_label_, 0 );
  title_layout->addWidget( title_label, 1 );
  title_layout->addWidget( close_button, 0 );
  title_bar->setLayout(title_layout);
  setTitleBarWidget( title_bar );
}

void PanelDockWidget::setIcon( QIcon icon )
{
  icon_label_->setPixmap( icon.pixmap(16,16) );
}

void PanelDockWidget::setCollapsed( bool collapsed )
{
  if ( collapsed_ == collapsed || isFloating() ) return;

  collapsed_ = collapsed;
  if ( collapsed )
  {
    QDockWidget::setVisible( false );
  }
  else
  {
    QDockWidget::setVisible( true );
  }
}

void PanelDockWidget::setContentWidget( QWidget* child )
{
  if( widget() )
  {
    disconnect( widget(), SIGNAL( destroyed( QObject* )), this, SLOT( onChildDestroyed( QObject* )));
  }
  setWidget( child );
  if( child )
  {
    connect( child, SIGNAL( destroyed( QObject* )), this, SLOT( onChildDestroyed( QObject* )));
  }
}

void PanelDockWidget::onChildDestroyed( QObject* )
{
  deleteLater();
}

} // end namespace rviz
