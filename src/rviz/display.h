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

#include <string>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>
#endif

#include "rviz/properties/status_property.h"
#include "rviz/properties/bool_property.h"

#include <QIcon>
#include <QSet>

class QDockWidget;
class QWidget;

namespace Ogre
{
class SceneManager;
class SceneNode;
}

// needed for timeSignal
Q_DECLARE_METATYPE(ros::Time);

namespace rviz
{

class StatusList;
class DisplayContext;
class PanelDockWidget;

class Display: public BoolProperty
{
Q_OBJECT
public:
  Display();
  virtual ~Display();

  /** @brief Main initialization, called after constructor, before load() or setEnabled(). */
  void initialize( DisplayContext* context );

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

  /** @brief Load the settings for this display from the given Config
   * node, which must be a map.
   *
   * Overridden from Property::load() to load the Display's
   * name and enabled state, then call Property::load().
   *
   * load() is called after initialize(). */
  virtual void load( const Config& config );

  /** @brief Write this display to the given Config node.
   *
   * Overridden from Property::save(). */
  virtual void save( Config config ) const;

  /** @brief Set the ROS topic to listen to for this display.
   *
   *  By default, do nothing.  Subclasses should override this method if they
   *  subscribe to a single ROS topic.
   *
   *  setTopic() is used by the "New display by topic" window; it is called
   *  with a user selected topic and its type.
   *
   *  @param topic The published topic to be visualized.
   *  @param datatype The datatype of the topic.
   */
  virtual void setTopic( const QString &topic, const QString &datatype )
  {
    (void) topic;
    (void) datatype;
  }

  /** @brief Return true if this Display is enabled, false if not. */
  bool isEnabled() const;

  /** @brief Set the fixed frame in this display. */
  void setFixedFrame( const QString& fixed_frame );

  /** @brief Called periodically by the visualization manager.
   * @param wall_dt Wall-clock time, in seconds, since the last time the update list was run through.
   * @param ros_dt ROS time, in seconds, since the last time the update list was run through. */
  virtual void update( float wall_dt, float ros_dt )
  {
    (void) wall_dt;
    (void) ros_dt;
  }

  /** @brief Called to tell the display to clear its state */
  virtual void reset();

  /** @brief Show status level and text.  This is thread-safe.
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

  /** @brief Show status level and text, using a std::string.
   * Convenience function which converts std::string to QString
   * and calls setStatus().  This is thread-safe. */
  void setStatusStd( StatusProperty::Level level, const std::string& name, const std::string& text )
    {
      setStatus( level, QString::fromStdString( name ), QString::fromStdString( text ));
    }

  /** @brief Delete the status entry with the given name.  This is thread-safe. */
  virtual void deleteStatus( const QString& name );

  /** @brief Delete the status entry with the given std::string name.  This is thread-safe. */
  void deleteStatusStd( const std::string& name ) { deleteStatus( QString::fromStdString( name )); }

  /** Default is all bits ON. */
  void setVisibilityBits( uint32_t bits );
  void unsetVisibilityBits( uint32_t bits );
  uint32_t getVisibilityBits() { return visibility_bits_; }

  /** @brief Return the Ogre::SceneNode holding all 3D scene elements shown by this Display. */
  Ogre::SceneNode* getSceneNode() const { return scene_node_; }

  /** @brief Associate the given @a widget with this Display.
   *
   * Each Display can have one QWidget which is shown when the Display
   * is enabled and hidden when the Display is disabled.  If there is
   * a WindowManagerInterface registered with the
   * VisualizationManager, like if you are using a VisualizationFrame,
   * this also adds widget as a pane within it (with
   * WindowManagerInterface::addPane() ).
   *
   * Since there is only one slot for such a widget, this
   * dis-associates any previously associated widget.
   *
   * Call this with NULL to disassociate the current associated widget. */
  void setAssociatedWidget( QWidget* widget );

  /** @brief Return the current associated widget, or NULL if there is none.
   * @sa setAssociatedWidget() */
  QWidget* getAssociatedWidget() const { return associated_widget_; }

  /** @brief Return the panel containing the associated widget, or NULL if there is none.
   * @sa setAssociatedWidget() */
  PanelDockWidget* getAssociatedWidgetPanel() { return associated_widget_panel_; }

  /** @brief Overridden from Property to set associated widget title to the new name. */
  void setName( const QString& name );

  /** @brief Emit a time signal that other Displays can synchronize to. */
  void emitTimeSignal( ros::Time time );

Q_SIGNALS:

  void timeSignal( rviz::Display* display, ros::Time time );

public Q_SLOTS:
  /** @brief Enable or disable this Display.
   *
   * SetEnabled is called after initialize() and at the end of load(),
   * if the Display settings are being loaded from a file. */
  void setEnabled( bool enabled );

  /** @brief Convenience function which calls context_->queueRender(). */
  void queueRender();

  /** @brief Set the Display's icon. */
  virtual void setIcon( const QIcon& icon );

protected:

  /** @brief Override this function to do subclass-specific initialization.
   *
   * This is called after vis_manager_ and scene_manager_ are set, and
   * before load() or setEnabled().
   *
   * setName() may or may not have been called before this. */
  virtual void onInitialize() {}

  /** @brief Derived classes override this to do the actual work of enabling themselves. */
  virtual void onEnable() {}

  /** @brief Derived classes override this to do the actual work of disabling themselves. */
  virtual void onDisable() {}

  /** @brief Delete all status children.  This is thread-safe.
   *
   * This removes all status children and updates the top-level status. */
  virtual void clearStatuses();

  /** @brief Called by setFixedFrame().  Override to respond to changes to fixed_frame_. */
  virtual void fixedFrameChanged() {}

  /** @brief Returns true if the display has been initialized */
  bool initialized() const { return initialized_; }

  /** @brief This DisplayContext pointer is the main connection a
   * Display has into the rest of rviz.  This is how the FrameManager
   * is accessed, the SelectionManager, etc.  When a Display subclass
   * wants to signal that a new render should be done right away, call
   * context_->queueRender().
   *
   * This is set after the constructor and before onInitialize() is called. */
  DisplayContext* context_;

  /** @brief A convenience variable equal to context_->getSceneManager().
   *
   * This is set after the constructor and before onInitialize() is called. */
  Ogre::SceneManager* scene_manager_;

  /** @brief The Ogre::SceneNode to hold all 3D scene elements shown by this Display. */
  Ogre::SceneNode* scene_node_;

  /** @brief A NodeHandle whose CallbackQueue is run from the main GUI thread (the "update" thread).
   *
   * This is configured after the constructor and before onInitialize() is called. */
  ros::NodeHandle update_nh_;

  /** @brief A NodeHandle whose CallbackQueue is run from a different thread than the GUI.
   *
   * This is configured after the constructor and before onInitialize() is called. */
  ros::NodeHandle threaded_nh_;

  /** @brief A convenience variable equal to context_->getFixedFrame().
   *
   * This is set after the constructor and before onInitialize() is
   * called. Every time it is updated (via setFixedFrame()),
   * fixedFrameChanged() is called. */
  QString fixed_frame_;

public Q_SLOTS:
  virtual void onEnableChanged();

private Q_SLOTS:
  void setStatusInternal( int level, const QString& name, const QString& text );
  void deleteStatusInternal( const QString& name );
  void clearStatusesInternal();
  void associatedPanelVisibilityChange( bool visible );
  void disable();

private:
  StatusList* status_;
  QString class_id_;
  bool initialized_;
  uint32_t visibility_bits_;
  QWidget* associated_widget_;
  PanelDockWidget* associated_widget_panel_;
};

} // end namespace rviz

#endif // DISPLAY_H
