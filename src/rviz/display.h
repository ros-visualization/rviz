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
#ifndef DISPLAY_H
#define DISPLAY_H

#include <ros/ros.h>

#include "rviz/properties/status_property.h"
#include "rviz/properties/property.h"

namespace Ogre
{
class SceneManager;
}

namespace rviz
{

class StatusList;
class DisplayContext;

class Display: public Property
{
Q_OBJECT
public:
  Display();

  /** @brief Main initialization, called after constructor, before load() or setEnabled(). */
  void initialize( DisplayContext* context );

  /** @brief Override this function to do subclass-specific initialization.
   *
   * This is called after vis_manager_ and scene_manager_ are set, and before load() or setEnabled(). */
  virtual void onInitialize() {}

  /** @brief Return data appropriate for the given column (0 or 1) and
   * role for this Display.
   */
  virtual QVariant getViewData( int column, int role ) const;

  /** @brief Return item flags appropriate for the given column (0 or
   * 1) for this Display. */
  virtual Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const { return class_id_; }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id ) { class_id_ = class_id; }

  /** @brief Load the settings for this display from the given YAML
   * node, which must be a map.
   *
   * Overridden from Property::load().  This version just calls
   * loadChildren().
   *
   * load() is called after initialize(). */
  virtual void load( const YAML::Node& yaml_node );

  /** @brief Load the settings for this display from the given YAML
   * node, which must be a map.
   *
   * Overridden from Property::loadChildren() to load the Display's
   * name and enabled state, then call Property::loadChildren(). */
  virtual void loadChildren( const YAML::Node& yaml_node );

  /** @brief Write this display to the given YAML emitter.
   *
   * Overridden from Property::save().  This version just begins a
   * map, calls saveChildren(), and ends the map. */
  virtual void save( YAML::Emitter& emitter );

  /** @brief Write the contents of this display to the given YAML
   * emitter, which must be in a map context already. */
  virtual void saveChildren( YAML::Emitter& emitter );

  /** @brief Return true if this Display is enabled, false if not. */
  bool isEnabled() const;

  /** @brief Set the fixed frame in this display. */
  void setFixedFrame( const QString& fixed_frame );

  /** @brief Called periodically by the visualization manager.
   * @param wall_dt Wall-clock time, in seconds, since the last time the update list was run through.
   * @param ros_dt ROS time, in seconds, since the last time the update list was run through. */
  virtual void update( float wall_dt, float ros_dt ) {}

  /** @brief Called to tell the display to clear its state */
  virtual void reset();

  /** @brief Show status level and text.
   * @param level One of StatusProperty::Ok, StatusProperty::Warn, or StatusProperty::Error.
   * @param name The name of the child entry to set.
   * @param text Description of the child's state.
   *
   * Every Display has a StatusList to indicate how it is doing.  The
   * StatusList has StatusPropertychildren indicating the status of
   * various subcomponents of the Display.  Each child of the status
   * has a level, a name, and descriptive text.  The top-level
   * StatusList has a level which is set to the worst of all the
   * children's levels.
   */
  virtual void setStatus( StatusProperty::Level level, const QString& name, const QString& text );

public Q_SLOTS:
  /** @brief Enable or disable this Display.
   *
   * SetEnabled is called after initialize() and at the end of load(),
   * if the Display settings are being loaded from a file. */
  void setEnabled( bool enabled );

protected:
  /** @brief Derived classes override this to do the actual work of enabling themselves. */
  virtual void onEnable() {}

  /** @brief Derived classes override this to do the actual work of disabling themselves. */
  virtual void onDisable() {}

  /** @brief Delete all status children.
   *
   * This removes all status children and updates the top-level status. */
  virtual void clearStatuses();

  /** @brief Called by setFixedFrame().  Override to respond to changes to fixed_frame_. */
  virtual void fixedFrameChanged() {}

  DisplayContext* context_;
  Ogre::SceneManager* scene_manager_;    ///< The scene manager we're associated with
  ros::NodeHandle update_nh_;
  ros::NodeHandle threaded_nh_;
  QString fixed_frame_;

private Q_SLOTS:
  void onEnableChanged();

private:
  StatusList* status_;
  QString class_id_;
  bool initialized_;
};

} // end namespace rviz

#endif // DISPLAY_H
