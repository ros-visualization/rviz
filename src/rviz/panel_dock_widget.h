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

#ifndef RVIZ_PANEL_DOCK_WIDGET_H
#define RVIZ_PANEL_DOCK_WIDGET_H

#include "rviz/config.h"

#include <QDockWidget>
#include <QLabel>

namespace rviz
{

/** @brief Dock widget class for docking widgets into VisualizationFrame.
 *
 * Use setContentWidget() instead of QDockWidget::setWidget() if you
 * want the PanelDockWidget to be destroyed when the content widget is
 * destroyed. */
class PanelDockWidget: public QDockWidget
{
Q_OBJECT
public:
  PanelDockWidget( const QString& name );

  void setContentWidget( QWidget* child );

  void setCollapsed( bool collapsed );

  void setIcon( QIcon icon );

  virtual void save( Config config );
  virtual void load( Config config );

protected:

  virtual void closeEvent ( QCloseEvent * event );

public Q_SLOTS:

  void setWindowTitle( QString title );

private Q_SLOTS:
  void onChildDestroyed( QObject* );

Q_SIGNALS:

  void closed();

private:
  // set to true if this panel was collapsed
  bool collapsed_;
  QLabel *icon_label_;
  QLabel *title_label_;
};

} // end namespace rviz

#endif // RVIZ_PANEL_DOCK_WIDGET_H
