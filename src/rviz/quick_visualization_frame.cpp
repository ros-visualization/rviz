#include "quick_visualization_frame.h"

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/ogre_helpers/qt_quick_ogre_render_window.h"

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

}

QuickVisualizationFrame::~QuickVisualizationFrame()
{

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

  render_panel_ = new RenderPanel( render_window, this );
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

bool QuickVisualizationFrame::isInitialized() const
{
  return initialized_;
}

void QuickVisualizationFrame::registerTypes()
{
  qRegisterMetaType<VisualizationManager*>("VisualizationManager*");
  qmlRegisterUncreatableType<VisualizationManager>("Rviz", 1, 0, "VisualizationManager", "Created by Rviz");
  qmlRegisterType<QuickVisualizationFrame>("Rviz", 1, 0, "VisualizationFrame");
  qmlRegisterType<QtQuickOgreRenderWindow>("Rviz", 1, 0, "RenderWindow");
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

  // would connect tool manager signals here

  manager_->initialize();

  Q_EMIT managerChanged(manager_);
}

void QuickVisualizationFrame::onOgreInitialized()
{
  // would load display config here

  manager_->startUpdate();
  initialized_ = true;
  Q_EMIT initializedChanged(initialized_);
  setStatus("RViz is ready");

  connect( manager_, &VisualizationManager::statusUpdate, this, &QuickVisualizationFrame::setStatus);
}

} // namespace rviz
