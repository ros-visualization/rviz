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
 */
class VisualizationManager: public QObject
{
Q_OBJECT
public:
  /**
   * \brief Constructor
   * Creates managers and sets up global properties.
   * @param render_panel a pointer to the main render panel widget of the app.
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

  RenderPanel* getRenderPanel() { return render_panel_; }

  typedef std::set<std::string> S_string;
  void getDisplayNames(S_string& displays);
  V_DisplayWrapper& getDisplays() { return displays_; }

  void resetDisplays();

  double getWallClock();
  double getROSTime();
  double getWallClockElapsed();
  double getROSTimeElapsed();

  void handleChar( QKeyEvent* event, RenderPanel* panel );
  void handleMouseEvent( ViewportMouseEvent& event );

  void setBackgroundColor(const Color& c);
  const Color& getBackgroundColor();

  void resetTime();

  ViewController* getCurrentViewController() { return view_controller_; }
  std::string getCurrentViewControllerType();
  bool setCurrentViewControllerType(const std::string& type);

  SelectionManager* getSelectionManager() { return selection_manager_; }

  void lockRender() { render_mutex_.lock(); }
  void unlockRender() { render_mutex_.unlock(); }
  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender();

  WindowManagerInterface* getWindowManager() { return window_manager_; }

  ros::CallbackQueueInterface* getUpdateQueue() { return ros::getGlobalCallbackQueue(); }
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
  void displaysConfigLoaded( const boost::shared_ptr<Config>& );
  void displaysConfigSaved( const boost::shared_ptr<Config>& );
  void toolAdded( Tool* );
  void toolChanged( Tool* );
  void viewControllerTypeAdded( const std::string& class_name, const std::string& name );
  void viewControllerChanged( ViewController* );
  void timeChanged();

protected Q_SLOTS:
  /// Called at 30Hz from the update timer
  void onUpdate();

  /// Called whenever the event loop has no other events to process.
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
