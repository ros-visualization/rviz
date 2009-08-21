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

#ifndef RVIZ_DISPLAY_H
#define RVIZ_DISPLAY_H

#include "properties/forwards.h"

#include <string>
#include <boost/function.hpp>
#include <boost/signals.hpp>

#include <ros/ros.h>

namespace Ogre
{
class SceneManager;
class MovableObject;
}

class wxPanel;
class wxWindow;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxPGProperty;
class wxConfigBase;

namespace rviz
{

class PropertyManager;
class CategoryProperty;
class BoolProperty;

class VisualizationManager;

class Display;
typedef boost::signal<void(Display*)> DisplaySignal;

/**
 * \class Display
 * \brief Abstract base class for all displays.
 *
 * Provides a common interface for the visualization panel to interact with,
 * so that new displays can be added without the visualization panel knowing anything about them.
 */
class Display
{
public:
  Display( const std::string& name, VisualizationManager* manager );
  virtual ~Display();

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
  void setEnabled(bool enable, bool force = false);

  const std::string& getName() const { return name_; }

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
   * Once this function is called, the property_manager_ member is valid and will stay valid
   */
  virtual void createProperties() {}

  /// Set the target frame of this display. This is a frame id which should match something being broadcast through TF.
  void setTargetFrame( const std::string& frame );

  /**
   * \brief Called from within setTargetFrame, notifying child classes that the target frame has changed
   */
  virtual void targetFrameChanged() = 0;

  /**
   * \brief Set the fixed frame of this display.  This is a frame id which should generally be the top-level frame being broadcast through TF
   * @param frame The fixed frame
   */
  void setFixedFrame( const std::string& frame );

  /**
   * \brief Called from within setFixedFrame, notifying child classes that the fixed frame has changed
   */
  virtual void fixedFrameChanged() = 0;

  /**
   * \brief Called to tell the display to clear its state
   */
  virtual void reset() {}

  DisplaySignal& getStateChangedSignal() { return state_changed_; }

protected:
  /// Derived classes override this to do the actual work of enabling themselves
  virtual void onEnable() = 0;
  /// Derived classes override this to do the actual work of disabling themselves
  virtual void onDisable() = 0;

  ///
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

  ros::NodeHandle update_nh_;
  ros::NodeHandle threaded_nh_;

  std::string target_frame_;                          ///< The frame we should transform all periodically-updated data into
  std::string fixed_frame_;                           ///< The frame we should transform all fixed data into

  boost::function<void ()> render_callback_;          ///< Render callback
  boost::function<void ()> render_lock_;              ///< Render lock callback
  boost::function<void ()> render_unlock_;            ///< Render unlock callback

  std::string property_prefix_;                       ///< Prefix to prepend to our properties

  PropertyManager* property_manager_;                 ///< The property manager to use to create properties
  CategoryPropertyWPtr parent_category_;                 ///< The parent category to use when creating properties
  BoolPropertyWPtr enabled_property_;

  DisplaySignal state_changed_;

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
