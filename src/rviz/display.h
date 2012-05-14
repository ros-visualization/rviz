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

#ifndef RVIZ_DISPLAY_H
#define RVIZ_DISPLAY_H

#include <QObject>

#include "properties/forwards.h"
#include "status_level.h"

#include <string>
#include <boost/function.hpp>

#include <ros/ros.h>

namespace Ogre
{
class SceneManager;
}

namespace rviz
{

class PropertyManager;
class CategoryProperty;
class BoolProperty;

class VisualizationManager;

class Display;

/**
 * @brief Abstract base class for all displays.
 *
 * Generally, a Display is something which listens for data on some
 * ROS topic and displays it in a 3D scene.  Displays do not have to
 * do either though.  The GridDisplay displays a grid attached to a TF
 * frame, but doesn't subscribe to any topic itself.  The Image
 * display subscribes to an image topic but displays in a 2D window,
 * not a 3D scene.
 *
 * One thing every display gets is a top-level entry in the "Displays"
 * panel, with a checkbox to enable or disable it.  When the checkbox
 * changes state, the virtual onEnable() or onDisable() function is
 * called.
 */
class Display: public QObject
{
Q_OBJECT
public:
  Display();
  virtual ~Display();

  /** @brief Main initialization, called right after constructor. */
  void initialize( const std::string& name, VisualizationManager* manager );

  /** @brief Override this function to do subclass-specific initialization.
   *
   * This is called after vis_manager_ and scene_manager_ are set. */
  virtual void onInitialize() {}

  /**
   * \brief Enable this display
   * @param force If false, does not re-enable if this display is already enabled.  If true, it does.
   */
  void enable( bool force = false );
  /**
   * \brief Disable this display
   * @param force If false, does not re-disable if this display is already disabled.  If true, it does.
   */
  void disable( bool force = false );

  bool isEnabled() { return enabled_; }

  /** @brief Call enable() or disable(). */
  void setEnabled(bool enable, bool force = false);

  const std::string& getName() const { return name_; }
  void setName(const std::string& name);

  /**
   * \brief Called periodically by the visualization panel
   * @param dt Wall-clock time, in seconds, since the last time the update list was run through.
   */
  virtual void update( float wall_dt, float ros_dt ) {}

  ///
  /**
   * \brief Set the callback used for causing a render to happen
   * @param func a void(void) function that will cause a render to happen from the correct thread
   */
  void setRenderCallback( boost::function<void ()> func );

  /// Set the callback used to lock the renderer
  void setLockRenderCallback( boost::function<void ()> func );
  /// Set the callback used to unlock the renderer
  void setUnlockRenderCallback( boost::function<void ()> func );

  /**
   * \brief Sets the property manager and parent category for this display
   * @param manager The property manager
   * @param parent The parent category
   */
  void setPropertyManager( PropertyManager* manager, const CategoryPropertyWPtr& parent );

  /**
   * \brief Called from setPropertyManager, gives the display a chance to create some properties immediately.
   *
   * When this function is called, the property_manager_ member is valid and will stay valid
   */
  virtual void createProperties() {}

  /**
   * \brief Set the fixed frame of this display.  This is a frame id which should generally be the top-level frame being broadcast through TF
   * @param frame The fixed frame
   */
  void setFixedFrame( const std::string& frame );

  /**
   * \brief Override to handle changes to fixed_frame_.
   * This base class implementation does nothing.
   */
  virtual void fixedFrameChanged() {}

  /**
   * \brief Called to tell the display to clear its state
   */
  virtual void reset();

  /** @brief Show status level and text.
   * @param level One of status_levels::Ok, status_levels::Warn, or status_levels::Error.
   * @param name The name of the child entry to set.
   * @param text Description of the child's state.
   *
   * Every Display has a StatusProperty to indicate how it is doing.
   * The StatusProperty has children in the PropertyTreeWidget
   * indicating the status of various subcomponents of the Display.
   * Each child of the status has a level, a name, and descriptive
   * text.  The top-level StatusProperty has a level which is set to
   * the worst of all the children's levels.
   */
  void setStatus(StatusLevel level, const std::string& name, const std::string& text);

  /** @brief Delete a status child.
   * @param name The name of the status child entry to remove.
   * This updates the top-level status after deleting the child. */
  void deleteStatus(const std::string& name);

  /** @brief Delete all status children.
   *
   * This removes all status children and updates the top-level status. */
  void clearStatuses();

  /** @brief Return the current top-level StatusLevel. */
  StatusLevel getStatus();

  /** @brief Hides all visible parts of this display, so they do not show up when the scene is rendered. */
  virtual void hideVisible() {}

  /** @brief Restores the display to the state it was in before hideVisible() was called. */
  virtual void restoreVisible() {}

Q_SIGNALS:
  /** @brief Emitted when this display goes from enabled to disabled or vice-versa. */
  void stateChanged( Display* );

protected:
  /** @brief Derived classes override this to do the actual work of enabling themselves. */
  virtual void onEnable() = 0;
  /** @brief Derived classes override this to do the actual work of disabling themselves. */
  virtual void onDisable() = 0;

  /**
   * \brief Cause the scene we're in to be rendered.
   * \note This does not immediately cause a render -- instead, one is queued and happens next run through the event loop.
   */
  void causeRender();

  /// Lock the renderer
  void lockRender();
  /// Unlock the renderer
  void unlockRender();

  VisualizationManager* vis_manager_;

  Ogre::SceneManager* scene_manager_;                 ///< The scene manager we're associated with
  std::string name_;                                  ///< The name of this display
  bool enabled_;                                      ///< Are we enabled?
  StatusLevel status_;

  ros::NodeHandle update_nh_;
  ros::NodeHandle threaded_nh_;

  std::string fixed_frame_;                           ///< The frame we should transform all fixed data into

  boost::function<void ()> render_callback_;          ///< Render callback
  boost::function<void ()> render_lock_;              ///< Render lock callback
  boost::function<void ()> render_unlock_;            ///< Render unlock callback

  std::string property_prefix_;                       ///< Prefix to prepend to our properties

  PropertyManager* property_manager_;                 ///< The property manager to use to create properties
  CategoryPropertyWPtr parent_category_;                 ///< The parent category to use when creating properties
  StatusPropertyWPtr status_property_;

  friend class RenderAutoLock;
};

/**
 * \class RenderAutoLock
 * \brief A scoped lock on the renderer
 *
 * Constructor calls Display::lockRender<br>
 * Destructor calls Display::unlockRender
 */
class RenderAutoLock
{
public:
  RenderAutoLock( Display* display )
  : display_( display )
  {
    display_->lockRender();
  }

  ~RenderAutoLock()
  {
    display_->unlockRender();
  }

private:
  Display* display_;
};

} // namespace rviz

#endif
