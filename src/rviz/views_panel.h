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

#ifndef RVIZ_VIEWS_PANEL_H
#define RVIZ_VIEWS_PANEL_H

#include <QWidget>

#include <vector>

#include <boost/shared_ptr.hpp>

class QListWidget;
class QComboBox;

namespace rviz
{

class Config;
class Display;
class VisualizationManager;
class ViewController;

/**
 * Panel for choosing the view controller and saving and restoring
 * viewpoints.
 */
class ViewsPanel: public QWidget
{
Q_OBJECT
public:
  ViewsPanel( QWidget* parent = 0 );
  virtual ~ViewsPanel();

  void initialize( VisualizationManager* manager );

  VisualizationManager* getManager() { return manager_; }

Q_SIGNALS:
  /** @brief Emitted when something changes which will change the display config file. */
  void configChanged();

protected Q_SLOTS:
  void onCameraTypeSelected( int index );
  void onSaveClicked();
  void onDeleteClicked();
  void onZeroClicked();
  void loadSelected();
  void clear();

  void readFromConfig( const boost::shared_ptr<Config>& config );
  void writeToConfig( const boost::shared_ptr<Config>& config );
  void onViewControllerTypeAdded( const std::string& class_name, const std::string& name );
  void onViewControllerChanged( ViewController* controller );

protected:
  struct View
  {
    std::string name_;
    std::string controller_class_;
    std::string controller_config_;
    std::string target_frame_;
  };
  typedef std::vector<View> V_View;

  void save( const std::string& name );
  void addView( const View& view );

  VisualizationManager* manager_;

  V_View views_;

  QListWidget* views_list_;
  QComboBox* camera_type_selector_;
};

} // namespace rviz

#endif // RVIZ_VIEWS_PANEL_H


