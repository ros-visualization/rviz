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


#ifndef RVIZ_VISUALIZATION_MANAGER_H_
#define RVIZ_VISUALIZATION_MANAGER_H_

#include <deque>

#include <ros/time.h>
#include <tf2_ros/transform_listener.h>

#include <rviz/bit_allocator.h>
#include <rviz/config.h>
#include <rviz/display_context.h>
#include <rviz/rviz_export.h>

class QKeyEvent;
class QTimer;

namespace Ogre
{
class Root;
class SceneManager;
class SceneNode;
class Light;
} // namespace Ogre

namespace ros
{
class CallbackQueueInterface;
}

namespace rviz
{
class ColorProperty;
class Display;
class DisplayFactory;
class DisplayGroup;
class FrameManager;
class Property;
class BoolProperty;
class IntProperty;
class PropertyTreeModel;
class RenderPanel;
class SelectionManager;
class StatusList;
class TfFrameProperty;
class ViewportMouseEvent;
class WindowManagerInterface;
class Tool;
class OgreRenderQueueClearer;

class VisualizationManagerPrivate;

/**
 * \brief The VisualizationManager class is the central manager class
 *        of rviz, holding all the Displays, Tools, ViewControllers,
 *        and other managers.
 *
 * It keeps the current view controller for the main render window.
 * It has a timer which calls update() on all the displays.  It
 * creates and holds pointers to the other manager objects:
 * SelectionManager, FrameManager, the PropertyManager s, and
 * Ogre::SceneManager.
 *
 * The "protected" members should probably all be "private", as
 * VisualizationManager is not intended to be subclassed.
 */
class RVIZ_EXPORT VisualizationManager : public DisplayContext
{
  Q_OBJECT
public:
  /**
   * \brief Constructor
   * Creates managers and sets up global properties.
   * @param render_panel a pointer to the main render panel widget of the app.
   * @param wm a pointer to the window manager
   *        (which is really just a VisualizationFrame, the top-level container widget of rviz).
   * @param tf_buffer an (optional) pointer to the tf2_ros::Buffer to be used by the FrameManager
   * @param tf_listener an (optional) pointer to the tf2_ros::TransformListener to be used
   *        This listener's tf buffer needs to be the same as the passed tf_buffer!
   *        Both tf_buffer and tf_listener are automatically created if not provided.
   */
  explicit VisualizationManager(
      RenderPanel* render_panel,
      WindowManagerInterface* wm = nullptr,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
      std::shared_ptr<tf2_ros::TransformListener> tf_listener =
          std::shared_ptr<tf2_ros::TransformListener>());

  /**
   * \brief Destructor
   * Stops update timers and destroys all displays, tools, and managers.
   */
  ~VisualizationManager() override;

  /**
   * \brief Do initialization that wasn't done in constructor.
   * Initializes tool manager, view manager, selection manager.
   */
  void initialize();

  /**
   * \brief Start timers.
   * Creates and starts the update and idle timers, both set to 30Hz (33ms).
   */
  void startUpdate();

  /*
   * \brief Stop the update timers. No Displays will be updated and no ROS
   *        callbacks will be called during this period.
   */
  void stopUpdate();

  /**
   * \brief Create and add a display to this panel, by class lookup name
   * @param class_lookup_name "lookup name" of the Display subclass, for pluginlib.
   *        Should be of the form "packagename/displaynameofclass", like "rviz/Image".
   * @param name The name of this display instance shown on the GUI, like "Left arm camera".
   * @param enabled Whether to start enabled
   * @return A pointer to the new display.
   */
  Display* createDisplay(const QString& class_lookup_name, const QString& name, bool enabled);

  /**
   * \brief Add a display to be managed by this panel
   * @param display The display to be added
   */
  void addDisplay(Display* display, bool enabled);

  /**
   * \brief Remove and delete all displays
   */
  void removeAllDisplays();

  /** @brief Load the properties of each Display and most editable rviz data.
   *
   * This is what is called when loading a "*.rviz" file.
   *
   * @param config The Config object to read from.  Expected to be a Config::Map type.
   * @sa save()
   */
  void load(const Config& config);

  /**
   * \brief Save the properties of each Display and most editable rviz
   *        data.
   *
   * This is what is called when saving a "*.vcg" file.
   * \param config The object to write to.
   * \sa loadDisplayConfig()
   */
  void save(Config config) const;

  /** @brief Return the fixed frame name.
   * @sa setFixedFrame() */
  QString getFixedFrame() const override;

  /** @brief Set the coordinate frame we should be transforming all fixed data into.
   * @param frame The name of the frame -- must match the frame name broadcast to libTF
   * @sa getFixedFrame() */
  void setFixedFrame(const QString& frame);

  /**
   * @brief Convenience function: returns getFrameManager()->getTF2BufferPtr().
   */
  std::shared_ptr<tf2_ros::Buffer> getTF2BufferPtr() const;

  /**
   * @brief Returns the Ogre::SceneManager used for the main RenderPanel.
   */
  Ogre::SceneManager* getSceneManager() const override
  {
    return scene_manager_;
  }

  /**
   * @brief Return the main RenderPanel.
   */
  RenderPanel* getRenderPanel() const
  {
    return render_panel_;
  }

  /**
   * @brief Return the wall clock time, in seconds since 1970.
   */
  double getWallClock();

  /**
   * @brief Return the ROS time, in seconds.
   */
  double getROSTime();

  /**
   * @brief Return the wall clock time in seconds since the last reset.
   */
  double getWallClockElapsed();

  /**
   * @brief Return the ROS time in seconds since the last reset.
   */
  double getROSTimeElapsed();

  /**
   * @brief Handle a single key event for a given RenderPanel.
   *
   * If the key is Escape, switches to the default Tool (via
   * getDefaultTool()).  All other key events are passed to the
   * current Tool (via getCurrentTool()).
   */
  void handleChar(QKeyEvent* event, RenderPanel* panel) override;

  /**
   * @brief Handle a mouse event.
   *
   * This just copies the given event into an event queue.  The events
   * in the queue are processed by onUpdate() which is called from the
   * main thread by a timer every 33ms.
   */
  void handleMouseEvent(const ViewportMouseEvent& event) override;

  /**
   * @brief Resets the wall and ROS elapsed time to zero and calls resetDisplays().
   */
  void resetTime();

  /**
   * @brief Return a pointer to the SelectionManager.
   */
  SelectionManager* getSelectionManager() const override
  {
    return selection_manager_;
  }

  /** @brief Return a pointer to the ToolManager. */
  ToolManager* getToolManager() const override
  {
    return tool_manager_;
  }

  /** @brief Return a pointer to the ViewManager. */
  ViewManager* getViewManager() const override
  {
    return view_manager_;
  }

  /**
   * @brief Lock a mutex to delay calls to Ogre::Root::renderOneFrame().
   */
  void lockRender();

  /**
   * @brief Unlock a mutex, allowing calls to Ogre::Root::renderOneFrame().
   */
  void unlockRender();

  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender() override;

  /**
   * @brief Return the window manager, if any.
   */
  WindowManagerInterface* getWindowManager() const override
  {
    return window_manager_;
  }

  /**
   * @brief Return the CallbackQueue using the main GUI thread.
   */
  ros::CallbackQueueInterface* getUpdateQueue() override;

  /**
   * @brief Return a CallbackQueue using a different thread than the main GUI one.
   */
  ros::CallbackQueueInterface* getThreadedQueue() override;

  /** @brief Return the FrameManager instance. */
  FrameManager* getFrameManager() const override
  {
    return frame_manager_;
  }

  /** @brief Return the current value of the frame count.
   *
   * The frame count is just a number which increments each time a
   * frame is rendered.  This lets clients check if a new frame has
   * been rendered since the last time they did something. */
  uint64_t getFrameCount() const override
  {
    return frame_count_;
  }

  /** @brief Notify this VisualizationManager that something about its
   * display configuration has changed. */
  void notifyConfigChanged();

  /** @brief Return a factory for creating Display subclasses based on a class id string. */
  DisplayFactory* getDisplayFactory() const override
  {
    return display_factory_;
  }

  PropertyTreeModel* getDisplayTreeModel() const
  {
    return display_property_tree_model_;
  }

  /** @brief Emits statusUpdate() signal with the given @a message. */
  void emitStatusUpdate(const QString& message);

  DisplayGroup* getRootDisplayGroup() const override
  {
    return root_display_group_;
  }

  uint32_t getDefaultVisibilityBit() const override
  {
    return default_visibility_bit_;
  }

  BitAllocator* visibilityBits() override
  {
    return &visibility_bit_allocator_;
  }

  void setStatus(const QString& message) override;

  virtual void setHelpPath(const QString& help_path)
  {
    help_path_ = help_path;
  }
  virtual QString getHelpPath() const
  {
    return help_path_;
  }

Q_SIGNALS:

  /** @brief Emitted before updating all Displays */
  void preUpdate();

  /** @brief Emitted whenever the display configuration changes. */
  void configChanged();

  /** @brief Emitted during file-loading and initialization to indicate progress. */
  void statusUpdate(const QString& message);

  /** @brief Emitted when ESC key is pressed */
  void escapePressed();

protected Q_SLOTS:
  /** @brief Call update() on all managed objects.
   *
   * This is the central place where update() is called on most rviz
   * objects.  Display objects, the FrameManager, the current
   * ViewController, the SelectionManager, PropertyManager.  Also
   * calls ros::spinOnce(), so any callbacks on the global
   * CallbackQueue get called from here as well.
   *
   * It is called at 30Hz from the update timer. */
  void onUpdate();

  void onToolChanged(Tool* /*unused*/);

protected:
  void updateTime();
  void updateFrames();

  void createColorMaterials();

  void threadedQueueThreadFunc();

  Ogre::Root* ogre_root_;             ///< Ogre Root
  Ogre::SceneManager* scene_manager_; ///< Ogre scene manager associated with this panel

  QTimer* update_timer_; ///< Display::update is called on each display whenever this timer fires
  ros::Time last_update_ros_time_; ///< Stores how long it's been since the last update
  ros::WallTime last_update_wall_time_;

  volatile bool shutting_down_;

  PropertyTreeModel* display_property_tree_model_;
  DisplayGroup* root_display_group_;

  ToolManager* tool_manager_;
  ViewManager* view_manager_;

  Property* global_options_;
  TfFrameProperty* fixed_frame_property_; ///< Frame to transform fixed data to
  StatusList* global_status_;
  IntProperty* fps_property_;
  BoolProperty* default_light_enabled_property_;

  RenderPanel* render_panel_;

  ros::WallTime wall_clock_begin_;
  ros::Time ros_time_begin_;
  ros::WallDuration wall_clock_elapsed_;
  ros::Duration ros_time_elapsed_;

  ColorProperty* background_color_property_;

  float time_update_timer_;
  float frame_update_timer_;

  SelectionManager* selection_manager_;

  uint32_t render_requested_;
  uint64_t frame_count_;

  WindowManagerInterface* window_manager_;

  FrameManager* frame_manager_;

  OgreRenderQueueClearer* ogre_render_queue_clearer_;

private Q_SLOTS:
  void updateFixedFrame();
  void updateBackgroundColor();
  void updateFps();
  void updateDefaultLightVisible();

private:
  DisplayFactory* display_factory_;
  VisualizationManagerPrivate* private_;
  uint32_t default_visibility_bit_;
  BitAllocator visibility_bit_allocator_;
  QString help_path_;
  Ogre::Light* directional_light_;
};

} // namespace rviz

#endif /* RVIZ_VISUALIZATION_MANAGER_H_ */
