#include "quick_visualization_frame.h"

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>
#include <QProcessEnvironment>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/ogre_helpers/qt_quick_ogre_render_window.h"
#include "rviz/tool_manager.h"

#include <ros/console.h>
#include <ros/package.h>
#include <ros/init.h>

namespace rviz {

QuickVisualizationFrame::QuickVisualizationFrame(QQuickItem *parent)
  : QQuickItem (parent)
  , render_panel_( nullptr )
  , render_window_( nullptr )
  , manager_( nullptr )
  , initializing_( false )
  , initialized_( false )
  , status_text_("")
{
  const auto loop = QProcessEnvironment::systemEnvironment();
  if (loop.value("QSG_RENDER_LOOP", "") != "basic") {
      qWarning() << "RViz QtQuick support currently only supports "
                    "QSG_RENDER_LOOP=basic";
  }
}

QuickVisualizationFrame::~QuickVisualizationFrame()
{
  if (render_panel_ != nullptr) {
    delete render_panel_;
    render_panel_ = nullptr;
  }

  if (manager_ != nullptr) {
    delete manager_;
    manager_ = nullptr;
  }

  if (render_window_ != nullptr) {
    render_window_ = nullptr;
  }

  Ogre::MeshManager::getSingleton().removeAll();
}

void QuickVisualizationFrame::initialize(QtQuickOgreRenderWindow *render_window)
{
  initializing_ = true;
  // TODO: init configs

  // TODO: load persistent settings

  if( !ros::isInitialized() )
  {
    int argc = 0;
    ros::init( argc, 0, "rviz", ros::init_options::AnonymousName );
  }

  connect(render_window, &QtQuickOgreRenderWindow::ogreInitializing,
          this, &QuickVisualizationFrame::onOgreInitializing);
  connect(render_window, &QtQuickOgreRenderWindow::ogreInitialized,
          this, &QuickVisualizationFrame::onOgreInitialized);

  render_panel_ = new RenderPanel( render_window );
}

void QuickVisualizationFrame::onOgreInitializing()
{
  manager_ = new VisualizationManager( render_panel_, nullptr );
  manager_->setRenderFromRenderPanel( true );

  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  // would connect tool manager signals here
  //ToolManager* tool_man = manager_->getToolManager();
  //connect(tool_man, &ToolManager::toolAdded, this &QuickVisualizationFrame::addTool);
  //connect(tool_man, &ToolManager::toolRemoved, this, &QuickVisualizationFrame::removeTool);
  //connect(tool_man, &ToolManager::toolRefreshed, this, &QuickVisualizationFrame::refreshTool);
  //connect(tool_man, &ToolManager::toolChanged, this, &QuickVisualizationFrame::inidicateToolIsCurrent);

  Q_EMIT managerChanged(manager_);
}

void QuickVisualizationFrame::onOgreInitialized()
{
  manager_->initialize();

  // would load display config here

  manager_->startUpdate();
  initialized_ = true;
  Q_EMIT initializedChanged(initialized_);
  setStatus("RViz is ready");

  connect( manager_, &VisualizationManager::preUpdate, this, &QuickVisualizationFrame::updateFps);
  connect( manager_, &VisualizationManager::statusUpdate, this, &QuickVisualizationFrame::setStatus);
}

void QuickVisualizationFrame::updateFps()
{
  frame_count_ ++;
  ros::WallDuration wall_diff = ros::WallTime::now() - last_fps_calc_time_;

  if ( wall_diff.toSec() > 1.0 )
  {
    frame_count_ = 0;
    last_fps_calc_time_ = ros::WallTime::now();
    fps_ = static_cast<double>(frame_count_) / wall_diff.toSec();
    Q_EMIT fpsChanged(fps_);
  }
}

QString QuickVisualizationFrame::getStatusText() const
{
  return status_text_;
}

void QuickVisualizationFrame::componentComplete()
{
  if (!render_window_ || initializing_) {
      return;
    }

  initialize(render_window_);
}

VisualizationManager *QuickVisualizationFrame::getManager() {
  return manager_;
}

QtQuickOgreRenderWindow *QuickVisualizationFrame::getRenderWindow() const
{
  return render_window_;
}

void rviz::QuickVisualizationFrame::setRenderWindow(QtQuickOgreRenderWindow *render_window)
{
  if (render_window_ == render_window) {
    return;
  }

  render_window_ = render_window;
  Q_EMIT renderWindowChanged(render_window_);

  if (render_window_ && !initializing_) {
    initialize(render_window);
  }
}

void QuickVisualizationFrame::load(const Config &config)
{
  if (!manager_) {
    qCritical("Load config called before creating visualization manager");
    return;
  }
  manager_->load( config.mapGetChild( "Visualization Manager" ));
}

void QuickVisualizationFrame::save(Config config)
{
  if (!manager_) {
    qCritical("Save config called before creating visualization manager");
    return;
  }
  manager_->save( config.mapMakeChild( "Visualization Manager" ));
}

bool QuickVisualizationFrame::isInitialized() const
{
  return initialized_;
}

double QuickVisualizationFrame::fps() const
{
  return fps_;
}

void QuickVisualizationFrame::registerTypes()
{
  qRegisterMetaType<VisualizationManager*>("VisualizationManager*");
  qRegisterMetaType<QtQuickOgreRenderWindow*>("QtQuickOgreRenderWindow*");
  qmlRegisterUncreatableType<VisualizationManager>("ros.rviz", 1, 0, "VisualizationManager", "Created by Rviz");
  qmlRegisterType<QuickVisualizationFrame>("ros.rviz", 1, 0, "VisualizationFrame");
  qmlRegisterType<QtQuickOgreRenderWindow>("ros.rviz", 1, 0, "RenderWindow");
}

void QuickVisualizationFrame::reset()
{
  Ogre::MeshManager::getSingleton().removeAll();
  manager_->resetTime();
}

void QuickVisualizationFrame::setStatus(const QString &message)
{
  status_text_ = message;
  Q_EMIT statusTextChanged(status_text_);
}

} // namespace rviz
