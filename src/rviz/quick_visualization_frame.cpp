#include "quick_visualization_frame.h"

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"

#include <ros/console.h>
#include <ros/package.h>
#include <ros/init.h>

namespace rviz {

QuickVisualizationFrame::QuickVisualizationFrame(QQuickItem *parent)
  : QQuickItem (parent)
  , render_panel_( nullptr )
  , render_window_( nullptr )
  , manager_( nullptr )
  , initialized_( false )
  , status_text_("")
{

}

QuickVisualizationFrame::~QuickVisualizationFrame()
{

}

void QuickVisualizationFrame::initialize(QtQuickOgreRenderWindow *render_window)
{
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

  render_panel_ = new RenderPanel( render_window, this );
}

QString QuickVisualizationFrame::getStatusText() const
{
  return status_text_;
}

void QuickVisualizationFrame::componentComplete()
{
  Q_ASSERT(render_window_ != nullptr);

  initialize(render_window_);
}

VisualizationManager *QuickVisualizationFrame::getManager() {
  return manager_;
}

QtQuickOgreRenderWindow *QuickVisualizationFrame::getRenderWindow() const
{
  return render_window_;
}

void QuickVisualizationFrame::registerTypes()
{
  qmlRegisterType<QuickVisualizationFrame>("Rviz", 1, 0, "VisualizationFrame");
  qmlRegisterType<QtQuickOgreRenderWindow>("Rviz", 1, 0, "RenderWindow");
  qmlRegisterUncreatableType<VisualizationManager>("Rviz", 1, 0, "VisualizationManager", "Created by Rviz");
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

void QuickVisualizationFrame::onOgreInitializing()
{
  manager_ = new VisualizationManager( render_panel_, nullptr );
  manager_->disableRender();

  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  // ToolManager* tool_man = manager_->getToolManager();
  // TODO: connect signals

  manager_->initialize();

  Q_EMIT managerChanged(manager_);
}

void QuickVisualizationFrame::onOgreInitialized()
{
  // TODO: load display config

  manager_->startUpdate();
  initialized_ = true;
  setStatus("RViz is ready");

  connect( manager_, &VisualizationManager::statusUpdate, this, &QuickVisualizationFrame::setStatus);

  auto grid = manager_->createDisplay( "rviz/Grid", "My grid", true );
  ROS_ASSERT( grid != NULL );
  grid->subProp( "Color" )->setValue( QColor( Qt::yellow ) );

  Q_EMIT initializationCompleted();
}

void rviz::QuickVisualizationFrame::setRenderWindow(QtQuickOgreRenderWindow *render_window)
{
  if (render_window_ == render_window) {
    return;
  }

  render_window_ = render_window;
  Q_EMIT renderWindowChanged(render_window_);
}

} // namespace rviz
