#include "quick_visualization_frame.h"

#include <ros/console.h>
#include <ros/package.h>
#include <ros/init.h>

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>

#include "rviz/ogre_helpers/qt_quick_ogre_render_window.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"

namespace rviz {

QuickVisualizationFrame::QuickVisualizationFrame(QQuickItem *parent)
  : QQuickItem (parent)
  , render_panel_( nullptr )
  , manager_( nullptr )
  , initialized_( false )
  , status_text_("")
{

}

QuickVisualizationFrame::~QuickVisualizationFrame()
{

}

void QuickVisualizationFrame::initialize(const QString &display_config_file)
{
  // TODO: init configs

  // TODO: load persistent settings

  if( !ros::isInitialized() )
  {
    int argc = 0;
    ros::init( argc, 0, "rviz", ros::init_options::AnonymousName );
  }

  //auto render_window = new QtWidgetOgreRenderWindow( central_widget );
  auto render_window = new QtQuickOgreRenderWindow(this);
  render_window->setParent(this);
  // anchors.fill: parent
  qvariant_cast<QObject*>(render_window->property("anchors"))
      ->setProperty("fill", QVariant::fromValue(this));
  connect(render_window, &QtQuickOgreRenderWindow::ogreInitialized, this, &QuickVisualizationFrame::onOgreInitialized);

  //render_window_ = render_window;
  render_panel_ = new RenderPanel( render_window, this );
}

QString QuickVisualizationFrame::getStatusText() const
{
  return status_text_;
}

void QuickVisualizationFrame::componentComplete()
{
  initialize("");
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

void QuickVisualizationFrame::onOgreInitialized()
{
  manager_ = new VisualizationManager( render_panel_, nullptr );

  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  ToolManager* tool_man = manager_->getToolManager();
  // TODO: connect signals

  manager_->initialize();

  // TODO: load display config

  manager_->startUpdate();
  initialized_ = true;
  Q_EMIT statusTextChanged("RViz is ready");

  connect( manager_, &VisualizationManager::preUpdate, this, &QuickVisualizationFrame::updateFps);
  connect( manager_, &VisualizationManager::statusUpdate, this, &QuickVisualizationFrame::statusTextChanged);
}

void QuickVisualizationFrame::updateFps()
{

}

} // namespace rviz
