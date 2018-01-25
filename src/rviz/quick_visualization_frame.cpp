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

QtQuickOgreRenderWindow *QuickVisualizationFrame::getRenderWindow() const
{
  return render_window_;
}

void QuickVisualizationFrame::reset()
{
  Ogre::MeshManager::getSingleton().removeAll();
  manager_->resetTime();
}

void QuickVisualizationFrame::showMessage(const QString &message)
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
}

void QuickVisualizationFrame::onOgreInitialized()
{
  // TODO: load display config

  manager_->startUpdate();
  initialized_ = true;
  showMessage("RViz is ready");

  connect( manager_, &VisualizationManager::preUpdate, this, &QuickVisualizationFrame::updateFps);
  connect( manager_, &VisualizationManager::statusUpdate, this, &QuickVisualizationFrame::statusTextChanged);

  auto grid = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( grid != NULL );
  grid->subProp( "Line Style" )->setValue( "Billboards" );
  grid->subProp( "Color" )->setValue( QColor( Qt::yellow ) );
}

void QuickVisualizationFrame::updateFps()
{

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
