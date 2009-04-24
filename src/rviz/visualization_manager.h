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

#include "helpers/color.h"
#include "properties/forwards.h"

#include <wx/event.h>
#include <wx/stopwatch.h>

#include <boost/signal.hpp>

#include <vector>
#include <map>
#include <set>

#include <roslib/Time.h>
#include <ros/time.h>

namespace ogre_tools
{
class wxOgreRenderWindow;
class FPSCamera;
class OrbitCamera;
class CameraBase;
class OrthoCamera;
}

namespace Ogre
{
class Root;
class SceneManager;
class SceneNode;
class Camera;
class RaySceneQuery;
}

namespace ros
{
class Node;
}

namespace tf
{
class TransformListener;
}

class wxTimerEvent;
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;
class wxKeyEvent;

namespace rviz
{

class PropertyManager;
class SelectionManager;
class RenderPanel;
class Display;
class DisplayFactory;
class Tool;
class ViewportMouseEvent;
class WindowManagerInterface;

typedef std::vector<std::string> V_string;
typedef std::vector<Display*> V_Display;

typedef boost::signal<void (Display*)> DisplaySignal;
typedef boost::signal<void (const V_Display&)> DisplaysSignal;
typedef boost::signal<void (const V_string&)> FramesChangedSignal;
typedef boost::signal<void (wxConfigBase*)> ConfigSignal;
typedef boost::signal<void (Tool*)> ToolSignal;
typedef boost::signal<void (ogre_tools::CameraBase*, const std::string&)> CameraTypeAddedSignal;
typedef boost::signal<void (ogre_tools::CameraBase*)> CameraSignal;
typedef boost::signal<void (void)> TimeSignal;

class VisualizationManager : public wxEvtHandler
{
public:
  /**
   * \brief Constructor
   */
  VisualizationManager(RenderPanel* render_panel, WindowManagerInterface* wm = 0);
  virtual ~VisualizationManager();

  void initialize();

  /**
   * \brief Create and then add a display to this panel.
   * @param name Display name of the display
   * @param enabled Whether to start enabled
   * @return A pointer to the new display
   */
  template< class T >
  T* createDisplay( const std::string& name, bool enabled )
  {
    Display* current_vis = getDisplay( name );
    if ( current_vis )
    {
      return NULL;
    }

    T* display = new T( name, this );
    addDisplay( display, enabled );

    return display;
  }

  /**
   * \brief Create and add a display to this panel, by type name
   * @param type Type name of the display
   * @param name Display name of the display
   * @param enabled Whether to start enabled
   * @return A pointer to the new display
   */
  Display* createDisplay( const std::string& type, const std::string& name, bool enabled );

  /**
   * \brief Remove a display
   * @param display The display to remove
   */
  void removeDisplay( Display* display );
  /**
   * \brief Remove a display by name
   * @param name The name of the display to remove
   */
  void removeDisplay( const std::string& name );
  /**
   * \brief Remove all displays
   */
  void removeAllDisplays();

  template< class T >
  T* createTool( const std::string& name, char shortcut_key )
  {
    T* tool = new T( name, shortcut_key, this );
    addTool( tool );

    return tool;
  }

  void addTool( Tool* tool );
  Tool* getCurrentTool() { return current_tool_; }
  Tool* getTool( int index );
  void setCurrentTool( Tool* tool );
  void setDefaultTool( Tool* tool );
  Tool* getDefaultTool() { return default_tool_; }

  /**
   * \brief Load general configuration
   * @param config The wx config object to load from
   */
  void loadGeneralConfig( wxConfigBase* config );
  /**
   * \brief Save general configuration
   * @param config The wx config object to save to
   */
  void saveGeneralConfig( wxConfigBase* config );
  /**
   * \brief Load display configuration
   * @param config The wx config object to load from
   */
  void loadDisplayConfig( wxConfigBase* config );
  /**
   * \brief Save display configuration
   * @param config The wx config object to save to
   */
  void saveDisplayConfig( wxConfigBase* config );

  /**
   * \brief Register a display factory with the panel.  Allows you to create a display by type.
   * @param type Type of the display.  Must be unique.
   * @param factory The factory which will create this type of display
   * @return Whether or not the registration succeeded.  The only failure condition is a non-unique type.
   */
  bool registerFactory( const std::string& type, const std::string& description, DisplayFactory* factory );

  /**
   * \brief Set the coordinate frame we should be displaying in
   * @param frame The string name -- must match the frame name broadcast to libTF
   */
  void setTargetFrame( const std::string& frame );
  const std::string& getTargetFrame() { return target_frame_; }

  /**
   * \brief Set the coordinate frame we should be transforming all fixed data to
   * @param frame The string name -- must match the frame name broadcast to libTF
   */
  void setFixedFrame( const std::string& frame );
  const std::string& getFixedFrame() { return fixed_frame_; }

  /**
   * \brief Performs a linear search to find a display based on its name
   * @param name Name of the display to search for
   */
  Display* getDisplay( const std::string& name );

  /**
   * \brief Enables/disables a display.  Raises the signal retrieved through getDisplayStateSignal()
   * @param display The display to act on
   * @param enabled Whether or not it should be enabled
   */
  void setDisplayEnabled( Display* display, bool enabled );

  PropertyManager* getPropertyManager() { return property_manager_; }

  bool isValidDisplay( Display* display );

  ros::Node* getROSNode() { return ros_node_; }
  tf::TransformListener* getTFClient() { return tf_; }
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }

  void getRegisteredTypes( std::vector<std::string>& types, std::vector<std::string>& descriptions );

  Ogre::SceneNode* getTargetRelativeNode() { return target_relative_node_; }

  RenderPanel* getRenderPanel() { return render_panel_; }

  typedef std::set<std::string> S_string;
  void getDisplayNames(S_string& displays);

  void resetDisplays();

  double getWallClock();
  double getROSTime();
  double getWallClockElapsed();
  double getROSTimeElapsed();

  void handleChar( wxKeyEvent& event );
  void handleMouseEvent( ViewportMouseEvent& event );

  void setBackgroundColor(const Color& c);
  const Color& getBackgroundColor();

  void resetTime();

  void moveDisplayUp(Display* display);
  void moveDisplayDown(Display* display);

  ogre_tools::CameraBase* getCurrentCamera() { return current_camera_; }
  const char* getCurrentCameraType();
  bool setCurrentCamera(const std::string& camera_type);
  void setCurrentCamera(int camera_type);

  SelectionManager* getSelectionManager() { return selection_manager_; }

  void lockRender() { render_mutex_.lock(); }
  void unlockRender() { render_mutex_.unlock(); }
  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender();

  WindowManagerInterface* getWindowManager() { return window_manager_; }

protected:
  /**
   * \brief Add a display to be managed by this panel
   * @param display The display to be added
   */
  void addDisplay( Display* display, bool enabled );

  void addCamera(ogre_tools::CameraBase* camera, const std::string& name);

  /// Called from the update timer
  void onUpdate( wxTimerEvent& event );

  void updateRelativeNode();

  void incomingROSTime();

  void updateTime();
  void updateFrames();

  void createColorMaterials();

  Ogre::Root* ogre_root_;                                 ///< Ogre Root
  Ogre::SceneManager* scene_manager_;                     ///< Ogre scene manager associated with this panel

  wxTimer* update_timer_;                                 ///< Update timer.  Display::update is called on each display whenever this timer fires
  wxStopWatch update_stopwatch_;                          ///< Update stopwatch.  Stores how long it's been since the last update

  ros::Node* ros_node_;                                   ///< Our ros::Node
  tf::TransformListener* tf_;                             ///< Our rosTF client


  V_Display displays_;                          ///< Our list of displays

  struct FactoryInfo
  {
    FactoryInfo(const std::string& name, const std::string& description, DisplayFactory* factory)
    : name_( name )
    , description_( description )
    , factory_( factory )
    {}

    std::string name_;
    std::string description_;
    DisplayFactory* factory_;
  };
  typedef std::map<std::string, FactoryInfo> M_FactoryInfo;
  M_FactoryInfo factories_;                                   ///< Factories by display type name

  typedef std::vector< Tool* > V_Tool;
  V_Tool tools_;
  Tool* current_tool_;
  Tool* default_tool_;

  std::string target_frame_;                              ///< Target coordinate frame we're displaying everything in
  std::string fixed_frame_;                               ///< Frame to transform fixed data to

  PropertyManager* property_manager_;
  EditEnumPropertyWPtr target_frame_property_;
  EditEnumPropertyWPtr fixed_frame_property_;

  V_string available_frames_;

  RenderPanel* render_panel_;

  Ogre::SceneNode* target_relative_node_;

  roslib::Time time_message_;
  bool needs_reset_;
  bool new_ros_time_;
  ros::WallTime wall_clock_begin_;
  ros::Time ros_time_begin_;
  ros::WallDuration wall_clock_elapsed_;
  ros::Duration ros_time_elapsed_;

  Color background_color_;
  ColorPropertyWPtr background_color_property_;

  float time_update_timer_;
  float frame_update_timer_;

  ogre_tools::CameraBase* current_camera_;                ///< The current camera
  int current_camera_type_;
  ogre_tools::FPSCamera* fps_camera_;                     ///< FPS camera
  ogre_tools::OrbitCamera* orbit_camera_;                 ///< Orbit camera
  ogre_tools::OrthoCamera* top_down_ortho_;               ///< Top-down orthographic camera

  SelectionManager* selection_manager_;

  boost::mutex render_mutex_;
  uint32_t render_requested_;
  float render_timer_;

  WindowManagerInterface* window_manager_;

public:
  FramesChangedSignal& getFramesChangedSignal() { return frames_changed_; }
  DisplaySignal& getDisplayStateSignal() { return display_state_; }
  DisplaySignal& getDisplayAddingSignal() { return display_adding_; }
  DisplaySignal& getDisplayAddedSignal() { return display_added_; }
  DisplaySignal& getDisplayRemovingSignal() { return display_removing_; }
  DisplaySignal& getDisplayRemovedSignal() { return display_removed_; }
  DisplaysSignal& getDisplaysRemovingSignal() { return displays_removing_; }
  DisplaysSignal& getDisplaysRemovedSignal() { return displays_removed_; }
  ConfigSignal& getDisplaysConfigLoadedSignal() { return displays_config_loaded_; }
  ConfigSignal& getDisplaysConfigSavingSignal() { return displays_config_saving_; }
  ConfigSignal& getGeneralConfigLoadedSignal() { return general_config_loaded_; }
  ConfigSignal& getGeneralConfigSavingSignal() { return general_config_saving_; }
  ToolSignal& getToolAddedSignal() { return tool_added_; }
  ToolSignal& getToolChangedSignal() { return tool_changed_; }
  CameraTypeAddedSignal& getCameraTypeAddedSignal() { return camera_type_added_; }
  CameraSignal& getCameraTypeChangedSignal() { return camera_type_changed_; }
  TimeSignal& getTimeChangedSignal() { return time_changed_; }

private:
  FramesChangedSignal frames_changed_;
  DisplaySignal display_state_;
  DisplaySignal display_adding_;
  DisplaySignal display_added_;
  DisplaySignal display_removing_;
  DisplaySignal display_removed_;
  DisplaysSignal displays_removing_;
  DisplaysSignal displays_removed_;
  ConfigSignal displays_config_loaded_;
  ConfigSignal displays_config_saving_;
  ConfigSignal general_config_loaded_;
  ConfigSignal general_config_saving_;
  ToolSignal tool_added_;
  ToolSignal tool_changed_;
  CameraTypeAddedSignal camera_type_added_;
  CameraSignal camera_type_changed_;
  TimeSignal time_changed_;
};

}

#endif /* RVIZ_VISUALIZATION_MANAGER_H_ */
