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

#include <fstream>

#include <QAction>
#include <QShortcut>
#include <QApplication>
#include <QCloseEvent>
#include <QDesktopServices>
#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QUrl>
#include <QStatusBar>
#include <QLabel>
#include <QToolButton>
#include <QHBoxLayout>
#include <QTabBar>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <ros/console.h>
#include <ros/package.h>
#include <ros/init.h>

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>

#include <rviz/ogre_helpers/initialization.h>

#include <rviz/displays_panel.h>
#include <rviz/env_config.h>
#include <rviz/failed_panel.h>
#include <rviz/help_panel.h>
#include <rviz/loading_dialog.h>
#include <rviz/new_object_dialog.h>
#include <rviz/preferences.h>
#include <rviz/preferences_dialog.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/panel_factory.h>
#include <rviz/render_panel.h>
#include <rviz/screenshot_dialog.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/selection_panel.h>
#include <rviz/splash_screen.h>
#include <rviz/time_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/tool_properties_panel.h>
#include <rviz/views_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/widget_geometry_change_detector.h>
#include <rviz/load_resource.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/yaml_config_writer.h>

#include <rviz/visualization_frame.h>

namespace fs = boost::filesystem;

#define CONFIG_EXTENSION "rviz"
#define CONFIG_EXTENSION_WILDCARD "*." CONFIG_EXTENSION
#define RECENT_CONFIG_COUNT 10

namespace rviz
{
VisualizationFrame::VisualizationFrame(QWidget* parent)
  : QMainWindow(parent)
  , app_(nullptr)
  , render_panel_(nullptr)
  , show_help_action_(nullptr)
  , preferences_(new Preferences())
  , file_menu_(nullptr)
  , recent_configs_menu_(nullptr)
  , toolbar_(nullptr)
  , manager_(nullptr)
  , splash_(nullptr)
  , toolbar_actions_(nullptr)
  , show_choose_new_master_option_(false)
  , toolbar_separator_(nullptr)
  , remove_tool_menu_(nullptr)
  , initialized_(false)
  , geom_change_detector_(new WidgetGeometryChangeDetector(this))
  , loading_(false)
  , post_load_timer_(new QTimer(this))
  , frame_count_(0)
  , toolbar_visible_(true)
{
  panel_factory_ = new PanelFactory();

  installEventFilter(geom_change_detector_);
  connect(geom_change_detector_, SIGNAL(changed()), this, SLOT(setDisplayConfigModified()));

  post_load_timer_->setSingleShot(true);
  connect(post_load_timer_, SIGNAL(timeout()), this, SLOT(markLoadingDone()));

  package_path_ = ros::package::getPath("rviz");
  help_path_ = QString::fromStdString((fs::path(package_path_) / "help/help.html").string());
  splash_path_ = QString::fromStdString((fs::path(package_path_) / "images/splash.png").string());

  QToolButton* reset_button = new QToolButton();
  reset_button->setText("Reset");
  reset_button->setContentsMargins(0, 0, 0, 0);
  statusBar()->addPermanentWidget(reset_button, 0);
  connect(reset_button, SIGNAL(clicked(bool)), this, SLOT(reset()));

  status_label_ = new QLabel("");
  statusBar()->addPermanentWidget(status_label_, 1);
  connect(this, SIGNAL(statusUpdate(const QString&)), status_label_, SLOT(setText(const QString&)));

  fps_label_ = new QLabel("");
  fps_label_->setMinimumWidth(40);
  fps_label_->setAlignment(Qt::AlignRight);
  statusBar()->addPermanentWidget(fps_label_, 0);
  original_status_bar_ = statusBar();

  setWindowTitle("RViz[*]");
}

VisualizationFrame::~VisualizationFrame()
{
  delete render_panel_;
  delete manager_;

  for (int i = 0; i < custom_panels_.size(); i++)
  {
    delete custom_panels_[i].dock;
  }

  delete panel_factory_;
}

void VisualizationFrame::setApp(QApplication* app)
{
  app_ = app;
}

void VisualizationFrame::setStatus(const QString& message)
{
  Q_EMIT statusUpdate(message);
}

void VisualizationFrame::updateFps()
{
  frame_count_++;
  ros::WallDuration wall_diff = ros::WallTime::now() - last_fps_calc_time_;

  if (wall_diff.toSec() > 1.0)
  {
    float fps = frame_count_ / wall_diff.toSec();
    frame_count_ = 0;
    last_fps_calc_time_ = ros::WallTime::now();
    if (original_status_bar_ == statusBar())
    {
      fps_label_->setText(QString::number(int(fps)) + QString(" fps"));
    }
  }
}

void VisualizationFrame::closeEvent(QCloseEvent* event)
{
  if (prepareToExit())
  {
    event->accept();
  }
  else
  {
    event->ignore();
  }
}

void VisualizationFrame::leaveEvent(QEvent* /*event*/)
{
  setStatus("");
}

void VisualizationFrame::reset()
{
  Ogre::MeshManager::getSingleton().removeAll();
  manager_->resetTime();
}

void VisualizationFrame::changeMaster()
{
  if (prepareToExit())
  {
    QApplication::exit(255);
  }
}

void VisualizationFrame::setShowChooseNewMaster(bool show)
{
  show_choose_new_master_option_ = show;
}

void VisualizationFrame::setHelpPath(const QString& help_path)
{
  help_path_ = help_path;
  manager_->setHelpPath(help_path_);
}

void VisualizationFrame::setSplashPath(const QString& splash_path)
{
  splash_path_ = splash_path;
}

void VisualizationFrame::initialize(const QString& display_config_file)
{
  initConfigs();

  loadPersistentSettings();

  QIcon app_icon(QString::fromStdString((fs::path(package_path_) / "icons/package.png").string()));
  setWindowIcon(app_icon);

  if (splash_path_ != "")
  {
    QPixmap splash_image(splash_path_);
    splash_ = new SplashScreen(splash_image);
    splash_->show();
    connect(this, SIGNAL(statusUpdate(const QString&)), splash_, SLOT(showMessage(const QString&)));
  }
  Q_EMIT statusUpdate("Initializing");

  // Periodically process events for the splash screen.
  // See: http://doc.qt.io/qt-5/qsplashscreen.html#details
  if (app_)
    app_->processEvents();

  if (!ros::isInitialized())
  {
    int argc = 0;
    ros::init(argc, nullptr, "rviz", ros::init_options::AnonymousName);
  }

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  QWidget* central_widget = new QWidget(this);
  QHBoxLayout* central_layout = new QHBoxLayout;
  central_layout->setSpacing(0);
  central_layout->setMargin(0);

  render_panel_ = new RenderPanel(central_widget);

  hide_left_dock_button_ = new QToolButton();
  hide_left_dock_button_->setContentsMargins(0, 0, 0, 0);
  hide_left_dock_button_->setArrowType(Qt::LeftArrow);
  hide_left_dock_button_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding));
  hide_left_dock_button_->setFixedWidth(16);
  hide_left_dock_button_->setAutoRaise(true);
  hide_left_dock_button_->setCheckable(true);

  connect(hide_left_dock_button_, SIGNAL(toggled(bool)), this, SLOT(hideLeftDock(bool)));

  hide_right_dock_button_ = new QToolButton();
  hide_right_dock_button_->setContentsMargins(0, 0, 0, 0);
  hide_right_dock_button_->setArrowType(Qt::RightArrow);
  hide_right_dock_button_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding));
  hide_right_dock_button_->setFixedWidth(16);
  hide_right_dock_button_->setAutoRaise(true);
  hide_right_dock_button_->setCheckable(true);

  connect(hide_right_dock_button_, SIGNAL(toggled(bool)), this, SLOT(hideRightDock(bool)));

  central_layout->addWidget(hide_left_dock_button_, 0);
  central_layout->addWidget(render_panel_, 1);
  central_layout->addWidget(hide_right_dock_button_, 0);

  central_widget->setLayout(central_layout);

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  initMenus();

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  initToolbars();

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  setCentralWidget(central_widget);

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  manager_ = new VisualizationManager(render_panel_, this);
  manager_->setHelpPath(help_path_);
  connect(manager_, SIGNAL(escapePressed()), this, SLOT(exitFullScreen()));

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  render_panel_->initialize(manager_->getSceneManager(), manager_);

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  ToolManager* tool_man = manager_->getToolManager();

  connect(manager_, SIGNAL(configChanged()), this, SLOT(setDisplayConfigModified()));
  connect(tool_man, SIGNAL(toolAdded(Tool*)), this, SLOT(addTool(Tool*)));
  connect(tool_man, SIGNAL(toolRemoved(Tool*)), this, SLOT(removeTool(Tool*)));
  connect(tool_man, SIGNAL(toolRefreshed(Tool*)), this, SLOT(refreshTool(Tool*)));
  connect(tool_man, SIGNAL(toolChanged(Tool*)), this, SLOT(indicateToolIsCurrent(Tool*)));

  manager_->initialize();

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  if (display_config_file != "")
  {
    loadDisplayConfig(display_config_file);
  }
  else
  {
    loadDisplayConfig(QString::fromStdString(default_display_config_file_));
  }

  // Periodically process events for the splash screen.
  if (app_)
    app_->processEvents();

  delete splash_;
  splash_ = nullptr;

  manager_->startUpdate();
  initialized_ = true;
  Q_EMIT statusUpdate("RViz is ready.");

  connect(manager_, SIGNAL(preUpdate()), this, SLOT(updateFps()));
  connect(manager_, SIGNAL(statusUpdate(const QString&)), this, SIGNAL(statusUpdate(const QString&)));
}

void VisualizationFrame::initConfigs()
{
  home_dir_ = QDir::toNativeSeparators(QDir::homePath()).toStdString();

  config_dir_ = (fs::path(home_dir_) / ".rviz").string();
  persistent_settings_file_ = (fs::path(config_dir_) / "persistent_settings").string();
  default_display_config_file_ = (fs::path(config_dir_) / "default." CONFIG_EXTENSION).string();

  if (fs::is_regular_file(config_dir_))
  {
    ROS_ERROR("Moving file [%s] out of the way to recreate it as a directory.", config_dir_.c_str());
    std::string backup_file = config_dir_ + ".bak";

    fs::rename(config_dir_, backup_file);
    fs::create_directory(config_dir_);
  }
  else if (!fs::exists(config_dir_))
  {
    fs::create_directory(config_dir_);
  }
}

void VisualizationFrame::loadPersistentSettings()
{
  YamlConfigReader reader;
  Config config;
  reader.readFile(config, QString::fromStdString(persistent_settings_file_));
  if (!reader.error())
  {
    QString last_config_dir, last_image_dir;
    if (config.mapGetString("Last Config Dir", &last_config_dir) &&
        config.mapGetString("Last Image Dir", &last_image_dir))
    {
      last_config_dir_ = last_config_dir.toStdString();
      last_image_dir_ = last_image_dir.toStdString();
    }

    Config recent_configs_list = config.mapGetChild("Recent Configs");
    recent_configs_.clear();
    int num_recent = recent_configs_list.listLength();
    for (int i = 0; i < num_recent; i++)
    {
      recent_configs_.push_back(recent_configs_list.listChildAt(i).getValue().toString().toStdString());
    }
  }
  else
  {
    ROS_ERROR("%s", qPrintable(reader.errorMessage()));
  }
}

void VisualizationFrame::savePersistentSettings()
{
  Config config;
  config.mapSetValue("Last Config Dir", QString::fromStdString(last_config_dir_));
  config.mapSetValue("Last Image Dir", QString::fromStdString(last_image_dir_));
  Config recent_configs_list = config.mapMakeChild("Recent Configs");
  for (D_string::iterator it = recent_configs_.begin(); it != recent_configs_.end(); ++it)
  {
    recent_configs_list.listAppendNew().setValue(QString::fromStdString(*it));
  }

  YamlConfigWriter writer;
  writer.writeFile(config, QString::fromStdString(persistent_settings_file_));

  if (writer.error())
  {
    ROS_ERROR("%s", qPrintable(writer.errorMessage()));
  }
}

void VisualizationFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu("&File");

  QAction* file_menu_open_action =
      file_menu_->addAction("&Open Config", this, SLOT(onOpen()), QKeySequence("Ctrl+O"));
  this->addAction(file_menu_open_action);
  QAction* file_menu_save_action =
      file_menu_->addAction("&Save Config", this, SLOT(onSave()), QKeySequence("Ctrl+S"));
  this->addAction(file_menu_save_action);
  QAction* file_menu_save_as_action =
      file_menu_->addAction("Save Config &As", this, SLOT(onSaveAs()), QKeySequence("Ctrl+Shift+S"));
  this->addAction(file_menu_save_as_action);

  recent_configs_menu_ = file_menu_->addMenu("&Recent Configs");
  file_menu_->addAction("Save &Image", this, SLOT(onSaveImage()));
  if (show_choose_new_master_option_)
  {
    file_menu_->addSeparator();
    file_menu_->addAction("Change &Master", this, SLOT(changeMaster()));
  }
  file_menu_->addSeparator();
  file_menu_->addAction("&Preferences", this, SLOT(openPreferencesDialog()), QKeySequence("Ctrl+P"));

  QAction* file_menu_quit_action =
      file_menu_->addAction("&Quit", this, SLOT(close()), QKeySequence("Ctrl+Q"));
  this->addAction(file_menu_quit_action);

  view_menu_ = menuBar()->addMenu("&Panels");
  view_menu_->addAction("Add &New Panel", this, SLOT(openNewPanelDialog()));
  delete_view_menu_ = view_menu_->addMenu("&Delete Panel");
  delete_view_menu_->setEnabled(false);

  QAction* fullscreen_action =
      view_menu_->addAction("&Fullscreen", this, SLOT(setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  this->addAction(
      fullscreen_action); // Also add to window, or the shortcut doest work when the menu is hidden.
  connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action, SLOT(setChecked(bool)));
  view_menu_->addSeparator();

  QMenu* help_menu = menuBar()->addMenu("&Help");
  help_menu->addAction("Show &Help panel", this, SLOT(showHelpPanel()));
  help_menu->addAction("Open rviz wiki in browser", this, SLOT(onHelpWiki()));
  help_menu->addSeparator();
  help_menu->addAction("&About", this, SLOT(onHelpAbout()));
}

void VisualizationFrame::initToolbars()
{
  QFont font;
  font.setPointSize(font.pointSizeF() * 0.9);

  // make toolbar with plugin tools

  toolbar_ = addToolBar("Tools");
  toolbar_->setFont(font);
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setObjectName("Tools");
  toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toolbar_actions_ = new QActionGroup(this);
  connect(toolbar_actions_, SIGNAL(triggered(QAction*)), this, SLOT(onToolbarActionTriggered(QAction*)));
  view_menu_->addAction(toolbar_->toggleViewAction());

  toolbar_separator_ = toolbar_->addSeparator();

  QToolButton* add_tool_button = new QToolButton();
  add_tool_button->setToolTip("Add a new tool");
  add_tool_button->setIcon(loadPixmap("package://rviz/icons/plus.png"));
  toolbar_->addWidget(add_tool_button);
  connect(add_tool_button, SIGNAL(clicked()), this, SLOT(openNewToolDialog()));

  remove_tool_menu_ = new QMenu(toolbar_);
  QToolButton* remove_tool_button = new QToolButton();
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip("Remove a tool from the toolbar");
  remove_tool_button->setIcon(loadPixmap("package://rviz/icons/minus.png"));
  toolbar_->addWidget(remove_tool_button);
  connect(remove_tool_menu_, SIGNAL(triggered(QAction*)), this, SLOT(onToolbarRemoveTool(QAction*)));

  QMenu* button_style_menu = new QMenu();
  QAction* action_tool_button_icon_only = new QAction("Icon only", toolbar_actions_);
  action_tool_button_icon_only->setData(Qt::ToolButtonIconOnly);
  button_style_menu->addAction(action_tool_button_icon_only);
  QAction* action_tool_button_text_only = new QAction("Text only", toolbar_actions_);
  action_tool_button_text_only->setData(Qt::ToolButtonTextOnly);
  button_style_menu->addAction(action_tool_button_text_only);
  QAction* action_tool_button_text_beside_icon = new QAction("Text beside icon", toolbar_actions_);
  action_tool_button_text_beside_icon->setData(Qt::ToolButtonTextBesideIcon);
  button_style_menu->addAction(action_tool_button_text_beside_icon);
  QAction* action_tool_button_text_under_icon = new QAction("Text under icon", toolbar_actions_);
  action_tool_button_text_under_icon->setData(Qt::ToolButtonTextUnderIcon);
  button_style_menu->addAction(action_tool_button_text_under_icon);

  QToolButton* button_style_button = new QToolButton();
  button_style_button->setMenu(button_style_menu);
  button_style_button->setPopupMode(QToolButton::InstantPopup);
  button_style_button->setToolTip("Set toolbar style");
  button_style_button->setIcon(loadPixmap("package://rviz/icons/visibility.svg"));
  toolbar_->addWidget(button_style_button);
  connect(button_style_menu, SIGNAL(triggered(QAction*)), this, SLOT(onButtonStyleTool(QAction*)));
}

void VisualizationFrame::hideDockImpl(Qt::DockWidgetArea area, bool hide)
{
  QList<PanelDockWidget*> dock_widgets = findChildren<PanelDockWidget*>();

  for (QList<PanelDockWidget*>::iterator it = dock_widgets.begin(); it != dock_widgets.end(); it++)
  {
    Qt::DockWidgetArea curr_area = dockWidgetArea(*it);
    if (area == curr_area)
    {
      (*it)->setCollapsed(hide);
    }
    // allow/disallow docking to this area for all widgets
    if (hide)
    {
      (*it)->setAllowedAreas((*it)->allowedAreas() & ~area);
    }
    else
    {
      (*it)->setAllowedAreas((*it)->allowedAreas() | area);
    }
  }
}

void VisualizationFrame::setHideButtonVisibility(bool visible)
{
  hide_left_dock_button_->setVisible(visible);
  hide_right_dock_button_->setVisible(visible);
}

void VisualizationFrame::hideLeftDock(bool hide)
{
  hideDockImpl(Qt::LeftDockWidgetArea, hide);
  hide_left_dock_button_->setArrowType(hide ? Qt::RightArrow : Qt::LeftArrow);
}

void VisualizationFrame::hideRightDock(bool hide)
{
  hideDockImpl(Qt::RightDockWidgetArea, hide);
  hide_right_dock_button_->setArrowType(hide ? Qt::LeftArrow : Qt::RightArrow);
}

void VisualizationFrame::onDockPanelVisibilityChange(bool visible)
{
  // if a dock widget becomes visible and is resting inside the
  // left or right dock area, we want to unhide the whole area
  if (visible)
  {
    QDockWidget* dock_widget = dynamic_cast<QDockWidget*>(sender());
    if (dock_widget)
    {
      Qt::DockWidgetArea area = dockWidgetArea(dock_widget);
      if (area == Qt::LeftDockWidgetArea)
      {
        hide_left_dock_button_->setChecked(false);
      }
      if (area == Qt::RightDockWidgetArea)
      {
        hide_right_dock_button_->setChecked(false);
      }
    }
  }
}

void VisualizationFrame::openPreferencesDialog()
{
  Preferences temp_preferences(*preferences_);
  PreferencesDialog* dialog = new PreferencesDialog(panel_factory_, &temp_preferences, this);
  manager_->stopUpdate();
  if (dialog->exec() == QDialog::Accepted)
  {
    // Apply preferences.
    preferences_ = boost::make_shared<Preferences>(temp_preferences);
  }
  manager_->startUpdate();
}

void VisualizationFrame::openNewPanelDialog()
{
  QString class_id;
  QString display_name;
  QStringList empty;

  NewObjectDialog* dialog =
      new NewObjectDialog(panel_factory_, "Panel", empty, empty, &class_id, &display_name, this);
  manager_->stopUpdate();
  if (dialog->exec() == QDialog::Accepted)
  {
    QDockWidget* dock = addPanelByName(display_name, class_id);
    if (dock)
    {
      connect(dock, SIGNAL(dockLocationChanged(Qt::DockWidgetArea)), this, SLOT(onDockPanelChange()));
    }
  }
  manager_->startUpdate();
}

void VisualizationFrame::openNewToolDialog()
{
  QString class_id;
  QStringList empty;
  ToolManager* tool_man = manager_->getToolManager();

  NewObjectDialog* dialog =
      new NewObjectDialog(tool_man->getFactory(), "Tool", empty, tool_man->getToolClasses(), &class_id);
  manager_->stopUpdate();
  if (dialog->exec() == QDialog::Accepted)
  {
    tool_man->addTool(class_id);
  }
  manager_->startUpdate();
  activateWindow(); // Force keyboard focus back on main window.
}

void VisualizationFrame::updateRecentConfigMenu()
{
  recent_configs_menu_->clear();

  D_string::iterator it = recent_configs_.begin();
  D_string::iterator end = recent_configs_.end();
  for (; it != end; ++it)
  {
    if (!it->empty())
    {
      std::string display_name = *it;
      if (display_name == default_display_config_file_)
      {
        display_name += " (default)";
      }
      if (display_name.find(home_dir_) == 0)
      {
        display_name = ("~" / fs::path(display_name.substr(home_dir_.size()))).string();
      }
      QString qdisplay_name = QString::fromStdString(display_name);
      QAction* action = new QAction(qdisplay_name, this);
      action->setData(QString::fromStdString(*it));
      connect(action, SIGNAL(triggered()), this, SLOT(onRecentConfigSelected()));
      recent_configs_menu_->addAction(action);
    }
  }
}

void VisualizationFrame::markRecentConfig(const std::string& path)
{
  D_string::iterator it = std::find(recent_configs_.begin(), recent_configs_.end(), path);
  if (it != recent_configs_.end())
  {
    recent_configs_.erase(it);
  }

  recent_configs_.push_front(path);

  if (recent_configs_.size() > RECENT_CONFIG_COUNT)
  {
    recent_configs_.pop_back();
  }

  updateRecentConfigMenu();
}

void VisualizationFrame::loadDisplayConfig(const QString& qpath)
{
  std::string path = qpath.toStdString();
  fs::path actual_load_path = path;
  bool valid_load_path = fs::is_regular_file(actual_load_path);

  if (!valid_load_path && fs::portable_posix_name(path))
  {
    if (actual_load_path.extension() != "." CONFIG_EXTENSION)
      actual_load_path += "." CONFIG_EXTENSION;
    actual_load_path = fs::path(config_dir_) / actual_load_path;
    if ((valid_load_path = fs::is_regular_file(actual_load_path)))
      path = actual_load_path.string();
  }

  if (!valid_load_path)
  {
    actual_load_path = (fs::path(package_path_) / "default.rviz");
    if (!(valid_load_path = fs::is_regular_file(actual_load_path)))
    {
      ROS_ERROR("Default display config '%s' not found.  RViz will be very empty at first.",
                actual_load_path.c_str());
      return;
    }
  }
  loadDisplayConfigHelper(actual_load_path.string());
}

bool VisualizationFrame::loadDisplayConfigHelper(const std::string& full_path)
{
  // Check if we have unsaved changes to the current config the same
  // as we do during exit, with the same option to cancel.
  if (!prepareToExit())
  {
    return false;
  }

  setWindowModified(false);
  loading_ = true;

  LoadingDialog* dialog = nullptr;
  if (initialized_)
  {
    dialog = new LoadingDialog(this);
    dialog->show();
    connect(this, SIGNAL(statusUpdate(const QString&)), dialog, SLOT(showMessage(const QString&)));
  }

  YamlConfigReader reader;
  Config config;
  reader.readFile(config, QString::fromStdString(full_path));
  if (reader.error())
    return false;

  load(config);

  markRecentConfig(full_path);

  setDisplayConfigFile(full_path);

  last_config_dir_ = fs::path(full_path).parent_path().string();

  delete dialog;

  post_load_timer_->start(1000);

  return true;
}

void VisualizationFrame::markLoadingDone()
{
  loading_ = false;
}

void VisualizationFrame::setImageSaveDirectory(const QString& directory)
{
  last_image_dir_ = directory.toStdString();
}

void VisualizationFrame::setDisplayConfigModified()
{
  if (!loading_)
  {
    if (!isWindowModified())
    {
      setWindowModified(true);
    }
  }
}

void VisualizationFrame::setDisplayConfigFile(const std::string& path)
{
  display_config_file_ = path;

  std::string title;
  if (path == default_display_config_file_)
  {
    title = "RViz[*]";
  }
  else
  {
    title = fs::path(path).filename().string() + "[*] - RViz";
  }
  setWindowTitle(QString::fromStdString(title));
}

bool VisualizationFrame::saveDisplayConfig(const QString& path)
{
  Config config;
  save(config);

  YamlConfigWriter writer;
  writer.writeFile(config, path);

  if (writer.error())
  {
    ROS_ERROR("%s", qPrintable(writer.errorMessage()));
    error_message_ = writer.errorMessage();
    return false;
  }
  else
  {
    setWindowModified(false);
    error_message_ = "";
    return true;
  }
}

void VisualizationFrame::save(Config config)
{
  manager_->save(config.mapMakeChild("Visualization Manager"));
  savePanels(config.mapMakeChild("Panels"));
  saveWindowGeometry(config.mapMakeChild("Window Geometry"));
  savePreferences(config.mapMakeChild("Preferences"));
  saveToolbars(config.mapMakeChild("Toolbars"));
}

void VisualizationFrame::load(const Config& config)
{
  manager_->load(config.mapGetChild("Visualization Manager"));
  loadPanels(config.mapGetChild("Panels"));
  loadWindowGeometry(config.mapGetChild("Window Geometry"));
  loadPreferences(config.mapGetChild("Preferences"));
  configureToolbars(config.mapGetChild("Toolbars"));
}

void VisualizationFrame::loadWindowGeometry(const Config& config)
{
  int x, y;
  if (config.mapGetInt("X", &x) && config.mapGetInt("Y", &y))
  {
    move(x, y);
  }

  int width, height;
  if (config.mapGetInt("Width", &width) && config.mapGetInt("Height", &height))
  {
    resize(width, height);
  }

  QString main_window_config;
  if (config.mapGetString("QMainWindow State", &main_window_config))
  {
    restoreState(QByteArray::fromHex(qPrintable(main_window_config)));
  }

  // load panel dock widget states (collapsed or not)
  QList<PanelDockWidget*> dock_widgets = findChildren<PanelDockWidget*>();

  for (QList<PanelDockWidget*>::iterator it = dock_widgets.begin(); it != dock_widgets.end(); it++)
  {
    Config itConfig = config.mapGetChild((*it)->windowTitle());

    if (itConfig.isValid())
    {
      (*it)->load(itConfig);
    }
  }

  bool b = false;
  config.mapGetBool("Hide Left Dock", &b);
  hide_left_dock_button_->setChecked(b);
  hideLeftDock(b);
  config.mapGetBool("Hide Right Dock", &b);
  hideRightDock(b);
  hide_right_dock_button_->setChecked(b);
}

void VisualizationFrame::configureToolbars(const Config& config)
{
  int tool_button_style;
  if (config.mapGetInt("toolButtonStyle", &tool_button_style))
  {
    toolbar_->setToolButtonStyle(static_cast<Qt::ToolButtonStyle>(tool_button_style));
  }
}

void VisualizationFrame::saveToolbars(Config config)
{
  config.mapSetValue("toolButtonStyle", static_cast<int>(toolbar_->toolButtonStyle()));
}

void VisualizationFrame::saveWindowGeometry(Config config)
{
  config.mapSetValue("X", x());
  config.mapSetValue("Y", y());
  config.mapSetValue("Width", width());
  config.mapSetValue("Height", height());

  QByteArray window_state = saveState().toHex();
  config.mapSetValue("QMainWindow State", window_state.constData());

  config.mapSetValue("Hide Left Dock", hide_left_dock_button_->isChecked());
  config.mapSetValue("Hide Right Dock", hide_right_dock_button_->isChecked());

  // save panel dock widget states (collapsed or not)
  QList<PanelDockWidget*> dock_widgets = findChildren<PanelDockWidget*>();

  for (QList<PanelDockWidget*>::iterator it = dock_widgets.begin(); it != dock_widgets.end(); it++)
  {
    (*it)->save(config.mapMakeChild((*it)->windowTitle()));
  }
}

void VisualizationFrame::loadPanels(const Config& config)
{
  // First destroy any existing custom panels.
  for (int i = 0; i < custom_panels_.size(); i++)
  {
    delete custom_panels_[i].dock;
    delete custom_panels_[i].delete_action;
  }
  custom_panels_.clear();

  // Then load the ones in the config.
  int num_custom_panels = config.listLength();
  for (int i = 0; i < num_custom_panels; i++)
  {
    Config panel_config = config.listChildAt(i);

    QString class_id, name;
    if (panel_config.mapGetString("Class", &class_id) && panel_config.mapGetString("Name", &name))
    {
      QDockWidget* dock = addPanelByName(name, class_id);
      // This is kind of ridiculous - should just be something like
      // createPanel() and addPanel() so I can do load() without this
      // qobject_cast.
      if (dock)
      {
        connect(dock, SIGNAL(dockLocationChanged(Qt::DockWidgetArea)), this, SLOT(onDockPanelChange()));
        Panel* panel = qobject_cast<Panel*>(dock->widget());
        if (panel)
        {
          panel->load(panel_config);
        }
      }
    }
  }

  onDockPanelChange();
}

void VisualizationFrame::savePanels(Config config)
{
  config.setType(Config::List); // Not really necessary, but gives an empty list if there are no entries,
                                // instead of an Empty config node.

  for (int i = 0; i < custom_panels_.size(); i++)
  {
    custom_panels_[i].panel->save(config.listAppendNew());
  }
}

void VisualizationFrame::loadPreferences(const Config& config)
{
  config.mapGetBool("PromptSaveOnExit", &(preferences_->prompt_save_on_exit));
}

void VisualizationFrame::savePreferences(Config config)
{
  config.mapSetValue("PromptSaveOnExit", preferences_->prompt_save_on_exit);
}

bool VisualizationFrame::prepareToExit()
{
  if (!initialized_)
  {
    return true;
  }

  savePersistentSettings();

  if (isWindowModified() && preferences_->prompt_save_on_exit)
  {
    QMessageBox box(this);
    box.setText("There are unsaved changes.");
    box.setInformativeText(QString::fromStdString("Save changes to " + display_config_file_ + "?"));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);
    manager_->stopUpdate();
    int result = box.exec();
    manager_->startUpdate();
    switch (result)
    {
    case QMessageBox::Save:
      if (saveDisplayConfig(QString::fromStdString(display_config_file_)))
      {
        return true;
      }
      else
      {
        QMessageBox box(this);
        box.setWindowTitle("Failed to save.");
        box.setText(getErrorMessage());
        box.setInformativeText(
            QString::fromStdString("Save copy of " + display_config_file_ + " to another file?"));
        box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        box.setDefaultButton(QMessageBox::Save);
        int result = box.exec();
        switch (result)
        {
        case QMessageBox::Save:
          onSaveAs();
          return true;
        case QMessageBox::Discard:
          return true;
        default:
          return false;
        }
      }
    case QMessageBox::Discard:
      return true;
    default:
      return false;
    }
  }
  else
  {
    return true;
  }
}

void VisualizationFrame::onOpen()
{
  manager_->stopUpdate();
  QString filename = QFileDialog::getOpenFileName(this, "Choose a file to open",
                                                  QString::fromStdString(last_config_dir_),
                                                  "RViz config files (" CONFIG_EXTENSION_WILDCARD ")");
  manager_->startUpdate();

  if (!filename.isEmpty())
  {
    std::string path = filename.toStdString();

    if (!fs::exists(path))
    {
      QString message = filename + " does not exist!";
      QMessageBox::critical(this, "Config file does not exist", message);
      return;
    }

    loadDisplayConfig(filename);
  }
}

void VisualizationFrame::onSave()
{
  if (!initialized_)
  {
    return;
  }

  savePersistentSettings();

  if (!saveDisplayConfig(QString::fromStdString(display_config_file_)))
  {
    manager_->stopUpdate();
    QMessageBox box(this);
    box.setWindowTitle("Failed to save.");
    box.setText(getErrorMessage());
    box.setInformativeText(
        QString::fromStdString("Save copy of " + display_config_file_ + " to another file?"));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);
    if (box.exec() == QMessageBox::Save)
    {
      onSaveAs();
    }
    manager_->startUpdate();
  }
}

void VisualizationFrame::onSaveAs()
{
  manager_->stopUpdate();
  QString q_filename = QFileDialog::getSaveFileName(this, "Choose a file to save to",
                                                    QString::fromStdString(last_config_dir_),
                                                    "RViz config files (" CONFIG_EXTENSION_WILDCARD ")");
  manager_->startUpdate();

  if (!q_filename.isEmpty())
  {
    std::string filename = q_filename.toStdString();
    fs::path path(filename);
    if (path.extension() != "." CONFIG_EXTENSION)
    {
      filename += "." CONFIG_EXTENSION;
    }

    if (!saveDisplayConfig(QString::fromStdString(filename)))
    {
      QMessageBox::critical(this, "Failed to save.", getErrorMessage());
    }

    markRecentConfig(filename);
    last_config_dir_ = fs::path(filename).parent_path().string();
    setDisplayConfigFile(filename);
  }
}

void VisualizationFrame::onSaveImage()
{
  ScreenshotDialog* dialog =
      new ScreenshotDialog(this, render_panel_, QString::fromStdString(last_image_dir_));
  connect(dialog, SIGNAL(savedInDirectory(const QString&)), this,
          SLOT(setImageSaveDirectory(const QString&)));
  dialog->show();
}

void VisualizationFrame::onRecentConfigSelected()
{
  QAction* action = dynamic_cast<QAction*>(sender());
  if (action)
  {
    std::string path = action->data().toString().toStdString();
    if (!path.empty())
    {
      if (!fs::exists(path))
      {
        QString message = QString::fromStdString(path) + " does not exist!";
        QMessageBox::critical(this, "Config file does not exist", message);
        return;
      }

      loadDisplayConfig(QString::fromStdString(path));
    }
  }
}

void VisualizationFrame::addTool(Tool* tool)
{
  QAction* action = new QAction(tool->getName(), toolbar_actions_);
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
  action->setCheckable(true);
  toolbar_->insertAction(toolbar_separator_, action);
  action_to_tool_map_[action] = tool;
  tool_to_action_map_[tool] = action;

  remove_tool_menu_->addAction(tool->getName());
}

void VisualizationFrame::onToolbarActionTriggered(QAction* action)
{
  Tool* tool = action_to_tool_map_[action];

  if (tool)
  {
    manager_->getToolManager()->setCurrentTool(tool);
  }
}

void VisualizationFrame::onToolbarRemoveTool(QAction* remove_tool_menu_action)
{
  QString name = remove_tool_menu_action->text();
  for (int i = 0; i < manager_->getToolManager()->numTools(); i++)
  {
    Tool* tool = manager_->getToolManager()->getTool(i);
    if (tool->getName() == name)
    {
      manager_->getToolManager()->removeTool(i);
      return;
    }
  }
}

void VisualizationFrame::onButtonStyleTool(QAction* button_style_tool_menu_action)
{
  toolbar_->setToolButtonStyle(
      static_cast<Qt::ToolButtonStyle>(button_style_tool_menu_action->data().toInt()));
}

void VisualizationFrame::removeTool(Tool* tool)
{
  QAction* action = tool_to_action_map_[tool];
  if (action)
  {
    toolbar_actions_->removeAction(action);
    toolbar_->removeAction(action);
    tool_to_action_map_.erase(tool);
    action_to_tool_map_.erase(action);
  }
  QString tool_name = tool->getName();
  QList<QAction*> remove_tool_actions = remove_tool_menu_->actions();
  for (int i = 0; i < remove_tool_actions.size(); i++)
  {
    QAction* removal_action = remove_tool_actions.at(i);
    if (removal_action->text() == tool_name)
    {
      remove_tool_menu_->removeAction(removal_action);
      break;
    }
  }
}

void VisualizationFrame::refreshTool(Tool* tool)
{
  QAction* action = tool_to_action_map_[tool];
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
}

void VisualizationFrame::indicateToolIsCurrent(Tool* tool)
{
  QAction* action = tool_to_action_map_[tool];
  if (action)
  {
    action->setChecked(true);
  }
}

void VisualizationFrame::showHelpPanel()
{
  if (!show_help_action_)
  {
    QDockWidget* dock = addPanelByName("Help", "rviz/Help");
    show_help_action_ = dock->toggleViewAction();
    connect(dock, SIGNAL(destroyed(QObject*)), this, SLOT(onHelpDestroyed()));
  }
  else
  {
    // show_help_action_ is a toggle action, so trigger() changes its
    // state.  Therefore we must force it to the opposite state from
    // what we want before we call trigger().  (I think.)
    show_help_action_->setChecked(false);
    show_help_action_->trigger();
  }
}

void VisualizationFrame::onHelpDestroyed()
{
  show_help_action_ = nullptr;
}

void VisualizationFrame::onHelpWiki()
{
  QDesktopServices::openUrl(QUrl("http://wiki.ros.org/rviz"));
}

void VisualizationFrame::onHelpAbout()
{
  QString about_text = QString("This is RViz version %1 (%2).\n"
                               "\n"
                               "Compiled against Qt version %3."
                               "\n"
                               "Compiled against OGRE version %4.%5.%6%7 (%8).")
                           .arg(get_version().c_str())
                           .arg(get_distro().c_str())
                           .arg(QT_VERSION_STR)
                           .arg(OGRE_VERSION_MAJOR)
                           .arg(OGRE_VERSION_MINOR)
                           .arg(OGRE_VERSION_PATCH)
                           .arg(OGRE_VERSION_SUFFIX)
                           .arg(OGRE_VERSION_NAME);

  QMessageBox::about(QApplication::activeWindow(), "About", about_text);
}

void VisualizationFrame::onDockPanelChange()
{
  QList<QTabBar*> tab_bars = findChildren<QTabBar*>(QString(), Qt::FindDirectChildrenOnly);
  for (QList<QTabBar*>::iterator it = tab_bars.begin(); it != tab_bars.end(); it++)
  {
    (*it)->setElideMode(Qt::ElideNone);
  }
}

QWidget* VisualizationFrame::getParentWindow()
{
  return this;
}

void VisualizationFrame::onDeletePanel()
{
  // This should only be called as a SLOT from a QAction in the
  // "delete panel" submenu, so the sender will be one of the QActions
  // stored as "delete_action" in a PanelRecord.  This code looks for
  // a delete_action in custom_panels_ matching sender() and removes
  // the panel associated with it.
  if (QAction* action = qobject_cast<QAction*>(sender()))
  {
    for (int i = 0; i < custom_panels_.size(); i++)
    {
      if (custom_panels_[i].delete_action == action)
      {
        delete custom_panels_[i].dock;
        custom_panels_.removeAt(i);
        setDisplayConfigModified();
        action->deleteLater();
        if (delete_view_menu_->actions().size() == 1 && delete_view_menu_->actions().first() == action)
        {
          delete_view_menu_->setEnabled(false);
        }
        return;
      }
    }
  }
}

void VisualizationFrame::setFullScreen(bool full_screen)
{
  Qt::WindowStates state = windowState();
  if (full_screen == state.testFlag(Qt::WindowFullScreen))
    return;
  Q_EMIT(fullScreenChange(full_screen));

  // when switching to fullscreen, remember visibility state of toolbar
  if (full_screen)
    toolbar_visible_ = toolbar_->isVisible();
  menuBar()->setVisible(!full_screen);
  toolbar_->setVisible(!full_screen && toolbar_visible_);
  statusBar()->setVisible(!full_screen);
  setHideButtonVisibility(!full_screen);

  if (full_screen)
    setWindowState(state | Qt::WindowFullScreen);
  else
    setWindowState(state & ~Qt::WindowFullScreen);
  show();
}

void VisualizationFrame::exitFullScreen()
{
  setFullScreen(false);
}

QDockWidget* VisualizationFrame::addPanelByName(const QString& name,
                                                const QString& class_id,
                                                Qt::DockWidgetArea area,
                                                bool floating)
{
  QString error;
  Panel* panel = panel_factory_->make(class_id, &error);
  if (!panel)
  {
    panel = new FailedPanel(class_id, error);
  }
  panel->setName(name);
  connect(panel, SIGNAL(configChanged()), this, SLOT(setDisplayConfigModified()));

  PanelRecord record;
  record.dock = addPane(name, panel, area, floating);
  record.panel = panel;
  record.name = name;
  record.delete_action = delete_view_menu_->addAction(name, this, SLOT(onDeletePanel()));
  custom_panels_.append(record);
  delete_view_menu_->setEnabled(true);

  record.panel->initialize(manager_);

  record.dock->setIcon(panel_factory_->getIcon(class_id));

  return record.dock;
}

PanelDockWidget*
VisualizationFrame::addPane(const QString& name, QWidget* panel, Qt::DockWidgetArea area, bool floating)
{
  PanelDockWidget* dock;
  dock = new PanelDockWidget(name);
  dock->setContentWidget(panel);
  dock->setFloating(floating);
  dock->setObjectName(name); // QMainWindow::saveState() needs objectName to be set.
  addDockWidget(area, dock);

  // we want to know when that panel becomes visible
  connect(dock, SIGNAL(visibilityChanged(bool)), this, SLOT(onDockPanelVisibilityChange(bool)));
  connect(this, SIGNAL(fullScreenChange(bool)), dock, SLOT(overrideVisibility(bool)));

  QAction* toggle_action = dock->toggleViewAction();
  view_menu_->addAction(toggle_action);

  connect(toggle_action, SIGNAL(triggered(bool)), this, SLOT(setDisplayConfigModified()));
  connect(dock, SIGNAL(closed()), this, SLOT(setDisplayConfigModified()));

  dock->installEventFilter(geom_change_detector_);

  // repair/update visibility status
  hideLeftDock(area == Qt::LeftDockWidgetArea ? false : hide_left_dock_button_->isChecked());
  hideRightDock(area == Qt::RightDockWidgetArea ? false : hide_right_dock_button_->isChecked());

  return dock;
}

} // end namespace rviz
