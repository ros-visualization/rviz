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

#include <QObject>
#include <QTimer>

#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include <boost/thread.hpp>

#include <vector>
#include <map>
#include <set>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/time.h>

#include <pluginlib/class_loader.h>

class QKeyEvent;

namespace Ogre
{
class Root;
class SceneManager;
class SceneNode;
}

namespace tf
{
class TransformListener;
}

namespace rviz
{

class Config;
class PropertyManager;
class SelectionManager;
class RenderPanel;
class Display;
class Tool;
class ViewportMouseEvent;
class WindowManagerInterface;
class PluginManager;
class PluginStatus;
class FrameManager;
class ViewController;
typedef boost::shared_ptr<FrameManager> FrameManagerPtr;

class DisplayWrapper;
typedef std::vector<DisplayWrapper*> V_DisplayWrapper;

class DisplayTypeInfo;
typedef boost::shared_ptr<DisplayTypeInfo> DisplayTypeInfoPtr;

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
class VisualizationManager: public QObject
{
Q_OBJECT
public:
  /**
   * \brief Constructor
   * Creates managers and sets up global properties.
   * @param render_panel a pointer to the main render panel widget of the app.
   *        This is the render_panel connected to the ViewController managed by this VisualizationManager.
   * @param wm a pointer to the window manager (which is really just a
   *        VisualizationFrame, the top-level container widget of rviz).
   */
  VisualizationManager(RenderPanel* render_panel, WindowManagerInterface* wm = 0);

  /**
   * \brief Destructor
   * Stops update timers and destroys all displays, tools, and managers.
   */
  virtual ~VisualizationManager();

  /**
   * \brief Do initialization that wasn't done in constructor.
   * Sets initial fixed and target frames, adds view controllers and
   * tools, and initializes SelectionManager.
   */
  void initialize(const StatusCallback& cb = StatusCallback(), bool verbose=false);

  /**
   * \brief Start timers.
   * Creates and starts the update and idle timers, both set to 30Hz (33ms).
   */
  void startUpdate();

  /**
   * \brief Create and add a display to this panel, by class lookup name
   * @param class_lookup_name "lookup name" of the Display subclass, for pluginlib.
   *        Should be of the form "packagename/displaynameofclass", like "rviz/Image".
   * @param name The name of this display instance shown on the GUI, like "Left arm camera".
   * @param enabled Whether to start enabled
   * @return A pointer to the wrapper for the new display
   */
  DisplayWrapper* createDisplay( const std::string& class_lookup_name, const std::string& name, bool enabled );

  /**
   * \brief Remove a display, by wrapper pointer.
   * @param display The wrapper of the display to remove
   */
  void removeDisplay( DisplayWrapper* display );

  /**
   * \brief Remove a display by name.
   * Removes the display with the given GUI display name, like "Left arm camera".
   * @param name The name of the display to remove
   */
  void removeDisplay( const std::string& name );

  /**
   * \brief Remove all displays
   */
  void removeAllDisplays();

  /**
   * \brief Create and add a tool.
   * This will go away in visualization 1.9, it is just a
   * wrapper around the constructor for T and a call to addTool().
   */
  template< class T >
  T* createTool( const std::string& name, char shortcut_key )
  {
    T* tool = new T( name, shortcut_key, this );
    addTool( tool );

    return tool;
  }

  /**
   * \brief Add a tool.
   * Adds a tool to the list of tools and emits the toolAdded(Tool*) signal.
   */
  void addTool( Tool* tool );

  /**
   * \brief Return the tool currently in use.
   * \sa setCurrentTool()
   */
  Tool* getCurrentTool() { return current_tool_; }

  /**
   * \brief Return the tool at a given index in the Tool list.
   * If index is less than 0 or greater than the number of tools, this
   * will fail an assertion.
   */
  Tool* getTool( int index );

  /**
   * \brief Set the current tool.
   * The current tool is given all mouse and keyboard events which
   * VisualizationManager receives via handleMouseEvent() and
   * handleChar().
   * \sa getCurrentTool()
   */
  void setCurrentTool( Tool* tool );

  /**
   * \brief Set the default tool.
   *
   * The default tool is selected directly by pressing the Escape key.
   * The default tool is indirectly selected when a Tool returns
   * Finished in the bit field result of Tool::processMouseEvent().
   * This is how control moves from the InitialPoseTool back to
   * MoveCamera when InitialPoseTool receives a left mouse button
   * release event.
   * \sa getDefaultTool()
   */
  void setDefaultTool( Tool* tool );

  /**
   * \brief Get the default tool.
   * \sa setDefaultTool()
   */
  Tool* getDefaultTool() { return default_tool_; }

  /**
   * \brief Load the properties of each Display and most editable rviz
   *        data.
   * 
   * This is what is called when loading a "*.vcg" file.
   * \param config The object to read data from.
   * \param cb An optional callback function to call with status
   *        updates, such as "loading displays".
   * \sa saveDisplayConfig()
   */
  void loadDisplayConfig( const boost::shared_ptr<Config>& config, const StatusCallback& cb = StatusCallback() );

  /**
   * \brief Save the properties of each Display and most editable rviz
   *        data.
   * 
   * This is what is called when saving a "*.vcg" file.
   * \param config The object to write to.
   * \sa loadDisplayConfig()
   */
  void saveDisplayConfig( const boost::shared_ptr<Config>& config );

  /**
   * \brief Set the coordinate frame whose position the display should track.
   *
   * The view controller sets the camera position by looking at the
   * \em position of the target frame relative to the fixed frame and
   * adding that to the position of the camera as controlled by the
   * user.  This lets the user keep the virtual camera following a
   * robot, for example, but not spinning the camera when the robot
   * spins.
   *
   * \param frame The target frame name -- must match the frame name broadcast to libTF
   * \sa getTargetFrame()
   */
  void setTargetFrame( const std::string& frame );

  /**
   * \brief Return the target frame name.
   * @sa setTargetFrame()
   */
  std::string getTargetFrame();

  /**
   * @brief Set the coordinate frame we should be transforming all fixed data into.
   * @param frame The name of the frame -- must match the frame name broadcast to libTF
   * @sa getFixedFrame()
   */
  void setFixedFrame( const std::string& frame );

  /**
   * @brief Return the fixed frame name.
   * @sa setFixedFrame()
   */
  const std::string& getFixedFrame() { return fixed_frame_; }

  /**
   * @brief Performs a linear search to find a display wrapper based on its name
   * @param name Name of the display to search for
   */
  DisplayWrapper* getDisplayWrapper( const std::string& name );

  /**
   * @brief Performs a linear search to find the DisplayWrapper
   *        holding a given Display.
   *
   * @param display Display to search for.
   */
  DisplayWrapper* getDisplayWrapper( Display* display );

  /**
   * @brief Get the PropertyManager which handles
   *        <span>Property</span>s of <span>Display</span>s.
   */
  PropertyManager* getPropertyManager() { return property_manager_; }

  /**
   * @brief Get the PropertyManager which handles
   *        <span>Property</span>s of <span>Tool</span>s.
   */
  PropertyManager* getToolPropertyManager() { return tool_property_manager_; }

  /**
   * @brief Return true if the given DisplayWrapper is currently in
   *        the list of displays, false otherwise.
   *
   * This does not check that the Display has actually been loaded
   * into the DisplayWrapper.
   */
  bool isValidDisplay( const DisplayWrapper* display );

  /**
   * @brief Convenience function: returns getFrameManager()->getTFClient().
   */
  tf::TransformListener* getTFClient();

  /**
   * @brief Returns the Ogre::SceneManager used for the main RenderPanel.
   */
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }

  /**
   * @brief Return the main RenderPanel.
   */
  RenderPanel* getRenderPanel() { return render_panel_; }

  typedef std::set<std::string> S_string;

  /**
   * @brief Return a std::set with all the current display names.
   */
  void getDisplayNames(S_string& displays);

  /**
   * @brief Return a reference to the DisplayWrapper array.
   */
  V_DisplayWrapper& getDisplays() { return displays_; }

  /**
   * @brief call Display::reset() on every Display.
   */
  void resetDisplays();

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
  void handleChar( QKeyEvent* event, RenderPanel* panel );

  /**
   * @brief Handle a mouse event.
   *
   * This just copies the given event into an event queue.  The events
   * in the queue are processed by onUpdate() which is called from the
   * main thread by a timer every 33ms.
   */
  void handleMouseEvent( ViewportMouseEvent& event );

  /**
   * @brief Set the background color of the main RenderWindow.
   */
  void setBackgroundColor(const Color& c);

  /**
   * @brief Return the background color of the main RenderWindow.
   */
  const Color& getBackgroundColor();

  /**
   * @brief Resets the wall and ROS elapsed time to zero and calls resetDisplays().
   */
  void resetTime();

  /**
   * @brief Return the current ViewController in use for the main RenderWindow.
   */
  ViewController* getCurrentViewController() { return view_controller_; }

  /**
   * @brief Return the type of the current ViewController as a
   *        std::string, like "rviz::OrbitViewController".
   */
  std::string getCurrentViewControllerType();

  /**
   * @brief Set the current view controller by specifying the desired type.
   *
   * This accepts the actual C++ class name (with namespace) of the
   * subclass of ViewController and also accepts a number of variants for backward-compatibility:
   *  - "rviz::OrbitViewController", "Orbit"
   *  - "rviz::XYOrbitViewController", "XYOrbit", "rviz::SimpleOrbitViewController", "SimpleOrbit"
   *  - "rviz::FPSViewController", "FPS"
   *  - "rviz::FixedOrientationOrthoViewController", "TopDownOrtho", "Top-down Orthographic"
   *
   * If `type` is not one of these and there is not a current
   * ViewController, the type defaults to rviz::OrbitViewController.
   * If `type` is not one of these and there *is* a current
   * ViewController, nothing happens.
   *
   * If the selected type is different from the current type, a new
   * instance of the selected type is created, set in the main
   * RenderPanel, and sent out via the viewControllerChanged() signal.
   */
  bool setCurrentViewControllerType(const std::string& type);

  /**
   * @brief Return a pointer to the SelectionManager.
   */
  SelectionManager* getSelectionManager() { return selection_manager_; }

  /**
   * @brief Lock a mutex to delay calls to Ogre::Root::renderOneFrame().
   */
  void lockRender() { render_mutex_.lock(); }

  /**
   * @brief Unlock a mutex, allowing calls to Ogre::Root::renderOneFrame().
   */
  void unlockRender() { render_mutex_.unlock(); }

  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender();

  /**
   * @brief Return the window manager, if any.
   */
  WindowManagerInterface* getWindowManager() { return window_manager_; }

  /**
   * @brief Return the CallbackQueue using the main GUI thread.
   */
  ros::CallbackQueueInterface* getUpdateQueue() { return ros::getGlobalCallbackQueue(); }

  /**
   * @brief Return a CallbackQueue using a different thread than the main GUI one.
   */
  ros::CallbackQueueInterface* getThreadedQueue() { return &threaded_queue_; }

  /** @brief Return the pluginlib::ClassLoader instance to use for
   * loading Display subclasses. */
  pluginlib::ClassLoader<Display>* getDisplayClassLoader() { return display_class_loader_; }

  /** @brief Return the FrameManager instance. */
  FrameManager* getFrameManager() { return frame_manager_.get(); }

  /** @brief Return the current value of the frame count.
   *
   * The frame count is just a number which increments each time a
   * frame is rendered.  This lets clients check if a new frame has
   * been rendered since the last time they did something. */
  uint64_t getFrameCount() { return frame_count_; }

  /** @brief Notify this VisualizationManager that something about its
   * display configuration has changed. */
  void notifyConfigChanged();

Q_SIGNALS:
  /** @brief Emitted just before a DisplayWrapper is added to the list of displays. */
  void displayAdding( DisplayWrapper* );

  /** @brief Emitted after a DisplayWrapper has been added and its
   * Display has been created, but before the display is enabled for
   * the first time. */
  void displayAdded( DisplayWrapper* );

  /** @brief Emitted just before a DisplayWrapper is removed from the list of displays. */
  void displayRemoving( DisplayWrapper* );

  /** @brief Emitted just after a DisplayWrapper is removed from the
   * list of displays, and before it is deleted. */
  void displayRemoved( DisplayWrapper* );

  /** @brief Emitted by removeAllDisplays() just before the list of displays is emptied. */
  void displaysRemoving( const V_DisplayWrapper& );

  /** @brief Emitted by removeAllDisplays() just after the list of displays is emptied. */
  void displaysRemoved( const V_DisplayWrapper& );

  /**
   * @brief Emitted by loadDisplayConfig() after Displays are loaded.
   *
   * Connect to this signal in order to load your object's state from
   * a Config object when a new display Config is being loaded.
   */
  void displaysConfigLoaded( const boost::shared_ptr<Config>& );

  /**
   * @brief Emitted by saveDisplayConfig() after Displays are saved.
   *
   * Connect to this signal in order to save your object's state to
   * a Config object when a display Config is being saved.
   */
  void displaysConfigSaved( const boost::shared_ptr<Config>& );

  /**
   * @brief Emitted by addTool() after the tool is added to the list of tools.
   */
  void toolAdded( Tool* );

  /**
   * @brief Emitted by setCurrentTool() after the newly chosen tool is
   * activated.
   */
  void toolChanged( Tool* );

  /**
   * @brief Emitted when a new ViewController type is added.
   * @param class_name is the C++ class name with namespace, like "rviz::OrbitViewController".
   * @param name is the name used for displaying, like "Orbit".
   */
  void viewControllerTypeAdded( const std::string& class_name, const std::string& name );

  /**
   * @brief Emitted after the current ViewController has changed.
   */
  void viewControllerChanged( ViewController* );

  /**
   * @brief Emitted at most once every 100ms.
   */
  void timeChanged();

  /** @brief Emitted whenever the display configuration changes. */
  void configChanged();

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

  /** @brief Render one frame if requested and enough time has passed
   *         since the previous render.
   *
   * Called at 30Hz from the "idle" timer */
  void onIdle();

  void onDisplayCreated( DisplayWrapper* wrapper );

protected:
  /**
   * \brief Add a display to be managed by this panel
   * @param display The display to be added
   */
  bool addDisplay(DisplayWrapper* wrapper, bool enabled);

  void addViewController(const std::string& class_name, const std::string& name);

  void updateRelativeNode();

  void incomingROSTime();

  void updateTime();
  void updateFrames();

  void createColorMaterials();

  void threadedQueueThreadFunc();

  void onPluginUnloading(const PluginStatus& status);

  Ogre::Root* ogre_root_;                                 ///< Ogre Root
  Ogre::SceneManager* scene_manager_;                     ///< Ogre scene manager associated with this panel

  QTimer* update_timer_;                                 ///< Update timer.  Display::update is called on each display whenever this timer fires
  ros::Time last_update_ros_time_;                        ///< Update stopwatch.  Stores how long it's been since the last update
  ros::WallTime last_update_wall_time_;

  QTimer* idle_timer_; ///< Timer with a timeout of 0.  Called by Qt event loop when it has no events to process.

  ros::CallbackQueue threaded_queue_;
  boost::thread_group threaded_queue_threads_;
  ros::NodeHandle update_nh_;
  ros::NodeHandle threaded_nh_;
  volatile bool shutting_down_;


  V_DisplayWrapper displays_;                          ///< Our list of displays

  typedef std::vector< Tool* > V_Tool;
  V_Tool tools_;
  Tool* current_tool_;
  Tool* default_tool_;

  std::string target_frame_;                              ///< Target coordinate frame we're displaying everything in
  std::string fixed_frame_;                               ///< Frame to transform fixed data to

  PropertyManager* property_manager_;
  PropertyManager* tool_property_manager_;
  TFFramePropertyWPtr target_frame_property_;
  EditEnumPropertyWPtr fixed_frame_property_;
  StatusPropertyWPtr status_property_;

  V_string available_frames_;

  RenderPanel* render_panel_;

  ros::WallTime wall_clock_begin_;
  ros::Time ros_time_begin_;
  ros::WallDuration wall_clock_elapsed_;
  ros::Duration ros_time_elapsed_;

  Color background_color_;
  ColorPropertyWPtr background_color_property_;

  float time_update_timer_;
  float frame_update_timer_;

  ViewController* view_controller_;

  SelectionManager* selection_manager_;

  boost::mutex render_mutex_;
  uint32_t render_requested_;
  uint64_t frame_count_;
  ros::WallTime last_render_;

  WindowManagerInterface* window_manager_;
  
  pluginlib::ClassLoader<Display>* display_class_loader_;
//  PluginManager* plugin_manager_;
  FrameManagerPtr frame_manager_;

  bool disable_update_;
  bool target_frame_is_fixed_frame_;

  Ogre::SceneNode *target_scene_node_;

  std::deque<ViewportMouseEvent> vme_queue_;
  boost::mutex vme_queue_mutex_;
};

}

#endif /* RVIZ_VISUALIZATION_MANAGER_H_ */
