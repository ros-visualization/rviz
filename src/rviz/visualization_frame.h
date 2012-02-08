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

#ifndef RVIZ_VISUALIZATION_FRAME_H
#define RVIZ_VISUALIZATION_FRAME_H

#include <QMainWindow>
#include <QList>

#include <string>
#include <deque>
#include <set>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>

#include "rviz/window_manager_interface.h"
#include "rviz/config.h"
#include "rviz/panel.h"
#include "rviz/properties/forwards.h" // just for StatusCallback

class QSplashScreen;
class QAction;
class QActionGroup;

namespace rviz
{

class PanelDockWidget;
class RenderPanel;
class DisplaysPanel;
class ViewsPanel;
class TimePanel;
class SelectionPanel;
class ToolPropertiesPanel;
class VisualizationManager;
class Tool;

/** @brief The main rviz window.
 *
 * VisualizationFrame is a QMainWindow, which means it has a center
 * area and a bunch of dock areas around it.  The central widget here
 * is a RenderPanel, and around it (by default) are a DisplaysPanel,
 * ViewsPanel, TimePanel, SelectionPanel, and ToolPropertiesPanel.  At
 * the top is a toolbar with "Move Camera", "Select", etc.  There is
 * also a menu bar with file/open, etc.
 */
class VisualizationFrame : public QMainWindow, public WindowManagerInterface
{
Q_OBJECT
public:
  VisualizationFrame( QWidget* parent = 0 );
  ~VisualizationFrame();

  void initialize( const std::string& display_config_file = "",
                   const std::string& fixed_frame = "",
                   const std::string& target_frame = "",
                   const std::string& splash_path = "",
                   bool verbose = false,
                   bool show_choose_new_master_option = false );

  VisualizationManager* getManager() { return manager_; }

  // overrides from WindowManagerInterface
  virtual QWidget* getParentWindow();
  virtual PanelDockWidget* addPane( const std::string& name,
                                    QWidget* panel,
                                    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
                                    bool floating = true );

protected Q_SLOTS:
  void onOpen();
  void onSave();
  void onSaveImage();
  void onRecentConfigSelected();
  void onHelpWiki();
  void openNewPanelDialog();

  /** @brief Looks up the Tool for this action and calls
   * VisualizationManager::setCurrentTool(). */
  void onToolbarActionTriggered( QAction* action );

  /** @brief Add the given tool to this frame's toolbar.
   *
   * This creates a QAction internally which listens for the Tool's
   * shortcut key.  When the action is triggered by the toolbar or by
   * the shortcut key, onToolbarActionTriggered() is called. */
  void addTool(Tool* tool);

  /** @brief Mark the given tool as the current one.
   *
   * This is purely a visual change in the GUI, it does not call any
   * tool functions. */
  void indicateToolIsCurrent(Tool* tool);

  /** @brief Save the current state and quit with exit code 255 to
   * signal the wrapper that we would like to restart with a different
   * ROS master URI. */
  void changeMaster();

  /** @brief Remove the given panel's name from the list of current
   * panel names. */
  void onPanelRemoved( QObject* panel );

  /** @brief Delete a panel widget.
   *
   * The sender() of the signal should be a QAction whose text() is
   * the name of the panel. */
  void onDeletePanel();

protected:
  void initConfigs();
  void initMenus();
  void loadDisplayConfig(const std::string& path);
  void saveConfigs();

  void moveEvent( QMoveEvent* event );
  void closeEvent( QCloseEvent* event );

  void setSplashStatus( const std::string& status );

  void markRecentConfig(const std::string& path);
  void updateRecentConfigMenu();

  QRect hackedFrameGeometry();

  PanelDockWidget* addCustomPanel( const std::string& name,
                                   const std::string& class_lookup_name,
                                   Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
                                   bool floating = true );

  void loadCustomPanels( const boost::shared_ptr<Config>& config );
  void saveCustomPanels( const boost::shared_ptr<Config>& config );

  void loadWindowGeometry( const boost::shared_ptr<Config>& config );
  void saveWindowGeometry( const boost::shared_ptr<Config>& config );

  /** @brief Load display and other settings from the given config object.
   * @param The config object to read from.
   * @param cb Optional.  Callback function to call with status updates, such as "loading displays".*/
  void loadDisplayConfig( const boost::shared_ptr<Config>& config, const StatusCallback& cb = StatusCallback() );

  /** @brief Save display and other settings to the given config object.
   * @param The config object to write to. */
  void saveDisplayConfig( const boost::shared_ptr<Config>& config );

  RenderPanel* render_panel_;
  DisplaysPanel* displays_panel_;
  ViewsPanel* views_panel_;
  TimePanel* time_panel_;
  SelectionPanel* selection_panel_;
  ToolPropertiesPanel* tool_properties_panel_;

  boost::shared_ptr<Config> general_config_;
  boost::shared_ptr<Config> display_config_;
  std::string config_dir_;
  std::string general_config_file_;
  std::string display_config_file_;
  std::string last_config_dir_;

  QMenu* file_menu_;
  QMenu* recent_configs_menu_;
  QMenu* view_menu_;
  QMenu* delete_view_menu_;
  QMenu* plugins_menu_;
  QList<QAction*> view_menu_actions_;

  QToolBar* toolbar_;

  VisualizationManager* manager_;

  std::string package_path_;

  QSplashScreen* splash_;

  typedef std::deque<std::string> D_string;
  D_string recent_configs_;

  QPoint first_position_;
  QPoint position_correction_;
  int num_move_events_;
  QActionGroup* toolbar_actions_;
  std::map<QAction*,Tool*> action_to_tool_map_;
  std::map<Tool*,QAction*> tool_to_action_map_;
  bool show_choose_new_master_option_;

  typedef std::set<std::string> S_string;
  S_string panel_names_;
  pluginlib::ClassLoader<Panel>* panel_class_loader_;

  struct PanelRecord
  {
    Panel* panel;
    PanelDockWidget* dock;
    std::string name;
    std::string lookup_name; // class lookup name needed by pluginlib.
    QAction* delete_action;
  };
  typedef std::map<std::string, PanelRecord> M_PanelRecord;
  M_PanelRecord custom_panels_;
};

}

#endif // RVIZ_VISUALIZATION_FRAME_H
