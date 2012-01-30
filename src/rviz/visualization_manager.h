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

class VisualizationManager: public QObject
{
Q_OBJECT
public:
  /**
   * \brief Constructor
   */
  VisualizationManager(RenderPanel* render_panel, WindowManagerInterface* wm = 0);
  virtual ~VisualizationManager();

  void initialize(const StatusCallback& cb = StatusCallback(), bool verbose=false);
  void startUpdate();

  /**
   * \brief Create and add a display to this panel, by class lookup name
   * @param class_lookup_name "lookup name" of the Display subclass, for pluginlib.
   * @param name The name of this display instance shown on the GUI.
   * @param enabled Whether to start enabled
   * @return A pointer to the new display
   */
  DisplayWrapper* createDisplay( const std::string& class_lookup_name, const std::string& name, bool enabled );

  /**
   * \brief Remove a display
   * @param display The display to remove
   */
  void removeDisplay( DisplayWrapper* display );
  /**
   * \brief Remove a display by name
   * @param name The name of the display to remove
   */
  void removeDisplay( const std::string& name );
  /**
   * \brief Remove all displays
   */
  void removeAllDisplays();

  /** Create a tool by class lookup name and add it. */
  void addTool( const std::string& tool_class_lookup_name );

  Tool* getCurrentTool() { return current_tool_; }
  Tool* getTool( int index );
  void setCurrentTool( Tool* tool );
  void setDefaultTool( Tool* tool );
  Tool* getDefaultTool() { return default_tool_; }

  // The "general" config file stores window geometry, plugin status, and view controller state.
  void loadGeneralConfig( const boost::shared_ptr<Config>& config, const StatusCallback& cb = StatusCallback() );
  void saveGeneralConfig( const boost::shared_ptr<Config>& config );

  // The "display" config file stores the properties of each Display.
  void loadDisplayConfig( const boost::shared_ptr<Config>& config, const StatusCallback& cb = StatusCallback() );
  void saveDisplayConfig( const boost::shared_ptr<Config>& config );

  /**
   * \brief Set the coordinate frame we should be displaying in
   * @param frame The string name -- must match the frame name broadcast to libTF
   */
  void setTargetFrame( const std::string& frame );
  std::string getTargetFrame();

  /**
   * \brief Set the coordinate frame we should be transforming all fixed data to
   * @param frame The string name -- must match the frame name broadcast to libTF
   */
  void setFixedFrame( const std::string& frame );
  const std::string& getFixedFrame() { return fixed_frame_; }

  /**
   * \brief Performs a linear search to find a display wrapper based on its name
   * @param name Name of the display to search for
   */
  DisplayWrapper* getDisplayWrapper( const std::string& name );

  /**
   * \brief Performs a linear search to find a display wrapper based on its display
   * @param display Display to search for
   */
  DisplayWrapper* getDisplayWrapper( Display* display );

  PropertyManager* getPropertyManager() { return property_manager_; }
  PropertyManager* getToolPropertyManager() { return tool_property_manager_; }

  bool isValidDisplay( const DisplayWrapper* display );

  tf::TransformListener* getTFClient();
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

  pluginlib::ClassLoader<Display>* getDisplayClassLoader() { return display_class_loader_; }
  pluginlib::ClassLoader<Tool>* getToolClassLoader() { return tool_class_loader_; }
//  PluginManager* getPluginManager() { return plugin_manager_; }
  FrameManager* getFrameManager() { return frame_manager_.get(); }

  uint64_t getFrameCount() { return frame_count_; }

Q_SIGNALS:
  void displayAdding( DisplayWrapper* );
  void displayAdded( DisplayWrapper* );
  void displayRemoving( DisplayWrapper* );
  void displayRemoved( DisplayWrapper* );
  void displaysRemoving( const V_DisplayWrapper& );
  void displaysRemoved( const V_DisplayWrapper& );
  void displaysConfigLoaded( const boost::shared_ptr<Config>& );
  void displaysConfigSaved( const boost::shared_ptr<Config>& );
  void generalConfigLoaded( const boost::shared_ptr<Config>& );
  void generalConfigSaving( const boost::shared_ptr<Config>& );
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

  struct ToolRecord
  {
    Tool* tool;
    std::string lookup_name; // for looking up the class with pluginlib
  };
  typedef std::vector<ToolRecord> V_ToolRecord;
  V_ToolRecord tools_;
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
  pluginlib::ClassLoader<Tool>* tool_class_loader_;
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
