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

#ifndef RVIZ_VIEWS_PANEL_H
#define RVIZ_VIEWS_PANEL_H

#include "rviz/panel.h"

class QComboBox;
class QModelIndex;
class QPushButton;

namespace rviz
{

class ViewManager;
class PropertyTreeWidget;

/**
 * @brief Panel for choosing the view controller and saving and restoring
 * viewpoints.
 */
class ViewsPanel: public Panel
{
Q_OBJECT
public:
  ViewsPanel( QWidget* parent = 0 );
  virtual ~ViewsPanel() {}

  /** @brief Overridden from Panel.  Just calls setViewManager() with vis_manager_->getViewManager(). */
  virtual void onInitialize();

  /** @brief Set the ViewManager which this panel should display and edit.
   *
   * If this ViewsPanel is to be used with a ViewManager other than
   * the one in the VisualizationManager sent in through
   * Panel::initialize(), either Panel::initialize() must not be
   * called or setViewManager() must be called after
   * Panel::initialize(). */
  void setViewManager( ViewManager* view_man );

  /** @brief Returns the current ViewManager. */
  ViewManager* getViewManager() const { return view_man_; }

  /** @brief Load configuration data, specifically the PropertyTreeWidget view settings. */
  virtual void load( const Config& config );

  /** @brief Save configuration data, specifically the PropertyTreeWidget view settings. */
  virtual void save( Config config ) const;

private Q_SLOTS:
  void onTypeSelectorChanged( int selected_index );
  void onDeleteClicked();
  void renameSelected();
  void onZeroClicked();
  void onCurrentChanged();

  void setCurrentViewFromIndex( const QModelIndex& index );

private:
  ViewManager* view_man_;
  PropertyTreeWidget* properties_view_;
  QPushButton* save_button_;
  QComboBox* camera_type_selector_;
};

} // namespace rviz

#endif // RVIZ_VIEWS_PANEL_H
