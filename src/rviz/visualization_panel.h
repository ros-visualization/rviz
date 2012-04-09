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

#ifndef RVIZ_VISUALIZATION_PANEL_H
#define RVIZ_VISUALIZATION_PANEL_H

#include <QSplitter>
#include <QString>

#include <string>

namespace rviz
{

class Config;
class RenderPanel;
class DisplaysPanel;
class VisualizationManager;

class VisualizationPanel: public QSplitter
{
Q_OBJECT
public:
  VisualizationPanel( QWidget* parent = 0 );
  ~VisualizationPanel();

  VisualizationManager* getManager() { return manager_; }

  void loadDisplayConfig( const std::string& filepath );
  void setViewControllerType( const std::string& view_type_name );
  void setViewString( const std::string& view_string );
  void setTargetFrame( const std::string& target_frame );

  void loadDisplayConfig( const QString& filepath ) { loadDisplayConfig( filepath.toStdString() ); }
  void setViewControllerType( const QString& view_type_name ) { setViewControllerType( view_type_name.toStdString() ); }
  void setViewString( const QString& view_string ) { setViewString( view_string.toStdString() ); }
  void setTargetFrame( const QString& target_frame ) { setTargetFrame( target_frame.toStdString() ); }

protected:
  RenderPanel* render_panel_;
  DisplaysPanel* displays_panel_;

  VisualizationManager* manager_;
};

}

#endif // RVIZ_VISUALIZATION_PANEL_H
