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

#include <algorithm>

#include <QApplication>
#include <QCursor>
#include <QPixmap>
#include <QTimer>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QWindow>
#endif

#include <boost/bind.hpp>

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreLight.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreRenderWindow.h>
#include <OgreSharedPtr.h>
#include <OgreCamera.h>

#include <boost/filesystem.hpp>

#include <tf/transform_listener.h>

#include <ros/package.h>
#include <ros/callback_queue.h>

#include "rviz/display.h"
#include "rviz/display_factory.h"
#include "rviz/display_group.h"
#include "rviz/displays_panel.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "rviz/load_resource.h"
#include "rviz/ogre_helpers/ogre_render_queue_clearer.h"
#include "rviz/ogre_helpers/render_system.h"

#include "rviz/visualization_manager.h"
#include "rviz/window_manager_interface.h"

namespace rviz
{
// helper class needed to display an icon besides "Global Options"
class IconizedProperty : public Property
{
public:
  IconizedProperty(const QString& name = QString(),
                   const QVariant default_value = QVariant(),
                   const QString& description = QString(),
                   Property* parent = nullptr,
                   const char* changed_slot = nullptr,
                   QObject* receiver = nullptr)
    : Property(name, default_value, description, parent, changed_slot, receiver){};
  QVariant getViewData(int column, int role) const override
  {
    return (column == 0 && role == Qt::DecorationRole) ? icon_ : Property::getViewData(column, role);
  }
  void setIcon(const QIcon& icon) override
  {
    icon_ = icon;
  }

private:
  QIcon icon_;
};

class VisualizationManagerPrivate
{
public:
  ros::CallbackQueue threaded_queue_;
  boost::thread_group threaded_queue_threads_;
  ros::NodeHandle update_nh_;
  ros::NodeHandle threaded_nh_;
  boost::mutex render_mutex_;
};

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
VisualizationManager::VisualizationManager(RenderPanel* render_panel, WindowManagerInterface* wm)
  : VisualizationManager(render_panel, wm, boost::shared_ptr<tf::TransformListener>())
{
}
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

VisualizationManager::VisualizationManager(RenderPanel* render_panel,
                                           WindowManagerInterface* wm,
                                           boost::shared_ptr<tf::TransformListener> tf)
  : ogre_root_(Ogre::Root::getSingletonPtr())
  , update_timer_(nullptr)
  , shutting_down_(false)
  , render_panel_(render_panel)
  , time_update_timer_(0.0f)
  , frame_update_timer_(0.0f)
  , render_requested_(1)
  , frame_count_(0)
  , window_manager_(wm)
  , private_(new VisualizationManagerPrivate)
{
  // visibility_bit_allocator_ is listed after default_visibility_bit_ (and thus initialized later be
  // default):
  default_visibility_bit_ = visibility_bit_allocator_.allocBit();

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

  frame_manager_ = new FrameManager(tf);

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

  render_panel->setAutoRender(false);

  private_->threaded_nh_.setCallbackQueue(&private_->threaded_queue_);

  scene_manager_ = ogre_root_->createSceneManager(Ogre::ST_GENERIC);

  rviz::RenderSystem::RenderSystem::get()->prepareOverlays(scene_manager_);

  directional_light_ = scene_manager_->createLight("MainDirectional");
  directional_light_->setType(Ogre::Light::LT_DIRECTIONAL);
  directional_light_->setDirection(Ogre::Vector3(-1, 0, -1));
  directional_light_->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

  root_display_group_ = new DisplayGroup();
  root_display_group_->setName("root");
  display_property_tree_model_ = new PropertyTreeModel(root_display_group_);
  display_property_tree_model_->setDragDropClass("display");
  connect(display_property_tree_model_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

  tool_manager_ = new ToolManager(this);
  connect(tool_manager_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));
  connect(tool_manager_, SIGNAL(toolChanged(Tool*)), this, SLOT(onToolChanged(Tool*)));

  view_manager_ = new ViewManager(this);
  view_manager_->setRenderPanel(render_panel_);
  connect(view_manager_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

  IconizedProperty* ip = new IconizedProperty("Global Options", QVariant(), "", root_display_group_);
  ip->setIcon(loadPixmap("package://rviz/icons/options.png"));
  global_options_ = ip;

  fixed_frame_property_ =
      new TfFrameProperty("Fixed Frame", "/map",
                          "Frame into which all data is transformed before being displayed.",
                          global_options_, frame_manager_, false, SLOT(updateFixedFrame()), this);

  background_color_property_ =
      new ColorProperty("Background Color", QColor(48, 48, 48), "Background color for the 3D view.",
                        global_options_, SLOT(updateBackgroundColor()), this);

  fps_property_ =
      new IntProperty("Frame Rate", 30, "RViz will try to render this many frames per second.",
                      global_options_, SLOT(updateFps()), this);

  default_light_enabled_property_ =
      new BoolProperty("Default Light", true, "Light source attached to the current 3D view.",
                       global_options_, SLOT(updateDefaultLightVisible()), this);

  root_display_group_->initialize(
      this); // only initialize() a Display after its sub-properties are created.
  root_display_group_->setEnabled(true);

  updateFixedFrame();
  updateBackgroundColor();

  global_status_ = new StatusList("Global Status", root_display_group_);

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);

  private_->threaded_queue_threads_.create_thread(
      boost::bind(&VisualizationManager::threadedQueueThreadFunc, this));

  display_factory_ = new DisplayFactory();

  ogre_render_queue_clearer_ = new OgreRenderQueueClearer();
  Ogre::Root::getSingletonPtr()->addFrameListener(ogre_render_queue_clearer_);

  update_timer_ = new QTimer;
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
}

VisualizationManager::~VisualizationManager()
{
  delete update_timer_;

  shutting_down_ = true;
  private_->threaded_queue_threads_.join_all();

  if (selection_manager_)
  {
    selection_manager_->setSelection(M_Picked());
  }

  delete display_property_tree_model_;
  delete tool_manager_;
  delete display_factory_;
  delete selection_manager_;
  delete view_manager_;

  if (ogre_root_)
  {
    ogre_root_->destroySceneManager(scene_manager_);
  }
  delete frame_manager_;
  delete private_;

  Ogre::Root::getSingletonPtr()->removeFrameListener(ogre_render_queue_clearer_);
  delete ogre_render_queue_clearer_;
}

void VisualizationManager::initialize()
{
  emitStatusUpdate("Initializing managers.");

  view_manager_->initialize();
  selection_manager_->initialize();
  tool_manager_->initialize();

  last_update_ros_time_ = ros::Time::now();
  last_update_wall_time_ = ros::WallTime::now();
}

ros::CallbackQueueInterface* VisualizationManager::getThreadedQueue()
{
  return &private_->threaded_queue_;
}

void VisualizationManager::lockRender()
{
  private_->render_mutex_.lock();
}

void VisualizationManager::unlockRender()
{
  private_->render_mutex_.unlock();
}

ros::CallbackQueueInterface* VisualizationManager::getUpdateQueue()
{
  return ros::getGlobalCallbackQueue();
}

void VisualizationManager::startUpdate()
{
  float interval = 1000.0 / float(fps_property_->getInt());
  update_timer_->start(interval);
}

void VisualizationManager::stopUpdate()
{
  update_timer_->stop();
}

void createColorMaterial(const std::string& name,
                         const Ogre::ColourValue& color,
                         bool use_self_illumination)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(name, ROS_PACKAGE_NAME);
  mat->setAmbient(color * 0.5f);
  mat->setDiffuse(color);
  if (use_self_illumination)
  {
    mat->setSelfIllumination(color);
  }
  mat->setLightingEnabled(true);
  mat->setReceiveShadows(false);
}

void VisualizationManager::createColorMaterials()
{
  createColorMaterial("RVIZ/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), true);
  createColorMaterial("RVIZ/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), true);
  createColorMaterial("RVIZ/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), true);
  createColorMaterial("RVIZ/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), true);
  createColorMaterial("RVIZ/ShadedRed", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedGreen", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedBlue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedCyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), false);
}

void VisualizationManager::queueRender()
{
  render_requested_ = 1;
}

void VisualizationManager::onUpdate()
{
  ros::WallDuration wall_diff = ros::WallTime::now() - last_update_wall_time_;
  ros::Duration ros_diff = ros::Time::now() - last_update_ros_time_;
  float wall_dt = wall_diff.toSec();
  float ros_dt = ros_diff.toSec();
  last_update_ros_time_ = ros::Time::now();
  last_update_wall_time_ = ros::WallTime::now();

  if (ros_dt < 0.0)
  {
    resetTime();
  }

  ros::spinOnce();

  Q_EMIT preUpdate();

  frame_manager_->update();

  root_display_group_->update(wall_dt, ros_dt);

  view_manager_->update(wall_dt, ros_dt);

  time_update_timer_ += wall_dt;

  if (time_update_timer_ > 0.1f)
  {
    time_update_timer_ = 0.0f;

    updateTime();
  }

  frame_update_timer_ += wall_dt;

  if (frame_update_timer_ > 1.0f)
  {
    frame_update_timer_ = 0.0f;

    updateFrames();
  }

  selection_manager_->update();

  if (tool_manager_->getCurrentTool())
  {
    tool_manager_->getCurrentTool()->update(wall_dt, ros_dt);
  }

  if (view_manager_ && view_manager_->getCurrent() && view_manager_->getCurrent()->getCamera())
  {
    directional_light_->setDirection(view_manager_->getCurrent()->getCamera()->getDerivedDirection());
  }

  frame_count_++;

  if (render_requested_ || wall_dt > 0.01)
  {
    render_requested_ = 0;
    boost::mutex::scoped_lock lock(private_->render_mutex_);
    ogre_root_->renderOneFrame();
  }
}

void VisualizationManager::updateTime()
{
  if (ros_time_begin_.isZero())
  {
    ros_time_begin_ = ros::Time::now();
  }

  ros_time_elapsed_ = ros::Time::now() - ros_time_begin_;

  if (wall_clock_begin_.isZero())
  {
    wall_clock_begin_ = ros::WallTime::now();
  }

  wall_clock_elapsed_ = ros::WallTime::now() - wall_clock_begin_;
}

void VisualizationManager::updateFrames()
{
  typedef std::vector<std::string> V_string;
  V_string frames;
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  frame_manager_->getTFClient()->getFrameStrings(frames);
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

  // Check the fixed frame to see if it's ok
  std::string error;
  if (frame_manager_->frameHasProblems(getFixedFrame().toStdString(), ros::Time(), error))
  {
    if (frames.empty())
    {
      // fixed_prop->setToWarn();
      std::stringstream ss;
      ss << "No tf data.  Actual error: " << error;
      global_status_->setStatus(StatusProperty::Warn, "Fixed Frame", QString::fromStdString(ss.str()));
    }
    else
    {
      // fixed_prop->setToError();
      global_status_->setStatus(StatusProperty::Error, "Fixed Frame", QString::fromStdString(error));
    }
  }
  else
  {
    // fixed_prop->setToOK();
    global_status_->setStatus(StatusProperty::Ok, "Fixed Frame", "OK");
  }
}

tf::TransformListener* VisualizationManager::getTFClient() const
{
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return frame_manager_->getTFClient();
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
}

std::shared_ptr<tf2_ros::Buffer> VisualizationManager::getTF2BufferPtr() const
{
  return frame_manager_->getTF2BufferPtr();
}

void VisualizationManager::resetTime()
{
  root_display_group_->reset();
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  frame_manager_->getTFClient()->clear();
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

  ros_time_begin_ = ros::Time();
  wall_clock_begin_ = ros::WallTime();

  queueRender();
}

void VisualizationManager::addDisplay(Display* display, bool enabled)
{
  root_display_group_->addDisplay(display);
  display->initialize(this);
  display->setEnabled(enabled);
}

void VisualizationManager::removeAllDisplays()
{
  root_display_group_->removeAllDisplays();
}

void VisualizationManager::emitStatusUpdate(const QString& message)
{
  Q_EMIT statusUpdate(message);
}

void VisualizationManager::load(const Config& config)
{
  stopUpdate();

  emitStatusUpdate("Creating displays");
  root_display_group_->load(config);

  emitStatusUpdate("Creating tools");
  tool_manager_->load(config.mapGetChild("Tools"));

  emitStatusUpdate("Creating views");
  view_manager_->load(config.mapGetChild("Views"));

  startUpdate();
}

void VisualizationManager::save(Config config) const
{
  root_display_group_->save(config);
  tool_manager_->save(config.mapMakeChild("Tools"));
  view_manager_->save(config.mapMakeChild("Views"));
}

Display*
VisualizationManager::createDisplay(const QString& class_lookup_name, const QString& name, bool enabled)
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  Display* new_display = root_display_group_->createDisplay(class_lookup_name);
  addDisplay(new_display, enabled);
  new_display->setName(name);
  QApplication::restoreOverrideCursor();
  return new_display;
}

double VisualizationManager::getWallClock()
{
  return ros::WallTime::now().toSec();
}

double VisualizationManager::getROSTime()
{
  return frame_manager_->getTime().toSec();
}

double VisualizationManager::getWallClockElapsed()
{
  return wall_clock_elapsed_.toSec();
}

double VisualizationManager::getROSTimeElapsed()
{
  return (frame_manager_->getTime() - ros_time_begin_).toSec();
}

void VisualizationManager::updateBackgroundColor()
{
  render_panel_->setBackgroundColor(qtToOgre(background_color_property_->getColor()));

  queueRender();
}

void VisualizationManager::updateFps()
{
  if (update_timer_->isActive())
  {
    startUpdate();
  }
}

void VisualizationManager::updateDefaultLightVisible()
{
  directional_light_->setVisible(default_light_enabled_property_->getBool());
}

void VisualizationManager::handleMouseEvent(const ViewportMouseEvent& vme)
{
  // process pending mouse events
  Tool* current_tool = tool_manager_->getCurrentTool();

  int flags = 0;
  if (current_tool)
  {
    ViewportMouseEvent _vme = vme;
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    QWindow* window = vme.panel->windowHandle();
    if (window)
    {
      double pixel_ratio = window->devicePixelRatio();
      _vme.x = static_cast<int>(pixel_ratio * _vme.x);
      _vme.y = static_cast<int>(pixel_ratio * _vme.y);
      _vme.last_x = static_cast<int>(pixel_ratio * _vme.last_x);
      _vme.last_y = static_cast<int>(pixel_ratio * _vme.last_y);
    }
#endif
    flags = current_tool->processMouseEvent(_vme);
    vme.panel->setCursor(current_tool->getCursor());
  }
  else
  {
    vme.panel->setCursor(QCursor(Qt::ArrowCursor));
  }

  if (flags & Tool::Render)
  {
    queueRender();
  }

  if (flags & Tool::Finished)
  {
    tool_manager_->setCurrentTool(tool_manager_->getDefaultTool());
  }
}

void VisualizationManager::handleChar(QKeyEvent* event, RenderPanel* panel)
{
  if (event->key() == Qt::Key_Escape)
    Q_EMIT escapePressed();
  tool_manager_->handleChar(event, panel);
}

void VisualizationManager::threadedQueueThreadFunc()
{
  while (!shutting_down_)
  {
    private_->threaded_queue_.callOne(ros::WallDuration(0.1));
  }
}

void VisualizationManager::notifyConfigChanged()
{
  Q_EMIT configChanged();
}

void VisualizationManager::onToolChanged(Tool* /*tool*/)
{
}

void VisualizationManager::updateFixedFrame()
{
  QString frame = fixed_frame_property_->getFrame();

  frame_manager_->setFixedFrame(frame.toStdString());
  root_display_group_->setFixedFrame(frame);
}

QString VisualizationManager::getFixedFrame() const
{
  return fixed_frame_property_->getFrame();
}

void VisualizationManager::setFixedFrame(const QString& frame)
{
  fixed_frame_property_->setValue(frame);
}

void VisualizationManager::setStatus(const QString& message)
{
  emitStatusUpdate(message);
}

} // namespace rviz
