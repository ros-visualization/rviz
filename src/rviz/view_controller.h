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

#ifndef RVIZ_VIEW_CONTROLLER_H
#define RVIZ_VIEW_CONTROLLER_H

#include <string>

#include <QCursor>
#include <utility>


#include <OgrePrerequisites.h>

#include <rviz/properties/property.h>
#include <rviz/rviz_export.h>

class QKeyEvent;

namespace rviz
{
class DisplayContext;
class EnumProperty;
class RenderPanel;
class ViewportMouseEvent;
class FloatProperty;
class BoolProperty;

class RVIZ_EXPORT ViewController : public Property
{
  Q_OBJECT
public:
  ViewController();
  ~ViewController() override;

  /** @brief Do all setup that can't be done in the constructor.
   *
   * Creates camera_ and attaches it to the root scene node.
   *
   * Calls onInitialize() just before returning. */
  void initialize(DisplayContext* context);

  static QString formatClassId(const QString& class_id);

  /** @brief Overridden from Property to give a different background
   * color and bold font if this view is active. */
  QVariant getViewData(int column, int role) const override;

  /** @brief Overridden from Property to make this draggable if it is not active. */
  Qt::ItemFlags getViewFlags(int column) const override;

  /** @brief Called by RenderPanel when this view controller is about to be used.
   *
   * There is no deactivate() because ViewControllers leaving
   * "current" are destroyed.  Put any cleanup in the destructor. */
  void activate();

  /** @brief Called at 30Hz by ViewManager::update() while this view
   * is active. Override with code that needs to run repeatedly. */
  virtual void update(float dt, float ros_dt)
  {
    (void)dt;
    (void)ros_dt;
  }

  virtual void handleMouseEvent(ViewportMouseEvent& evt)
  {
    (void)evt;
  }

  /** @brief Called by MoveTool and InteractionTool when keyboard events are passed to them.
   *
   * The default implementation here handles the "F" (focus on object)
   * and "Z" (zero - reset) keys. */
  virtual void handleKeyEvent(QKeyEvent* event, RenderPanel* panel);

  /** @brief Convenience function which calls lookAt(Ogre::Vector3). */
  void lookAt(float x, float y, float z);

  /** @brief This should be implemented in each subclass to aim the
   * camera at the given point in space (relative to the fixed
   * frame). */
  virtual void lookAt(const Ogre::Vector3& point)
  {
    (void)point;
  }

  /** Reset the view controller to some sane initial state, like
   * looking at 0,0,0 from a few meters away. */
  virtual void reset() = 0;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera().
   *
   * This base class implementation does nothing. */
  virtual void mimic(ViewController* source_view)
  {
    (void)source_view;
  }

  /** @brief Called by ViewManager when this ViewController is being made current.
   * @param previous_view is the previous "current" view, and will not be NULL.
   *
   * This gives ViewController subclasses an opportunity to implement
   * a smooth transition from a previous viewpoint to the new
   * viewpoint.
   *
   * This base class implementation does nothing. */
  virtual void transitionFrom(ViewController* previous_view)
  {
    (void)previous_view;
  }

  /** @brief Subclasses should call this whenever a change is made which would change the results of
   * toString(). */
  void emitConfigChanged();

  Ogre::Camera* getCamera() const
  {
    return camera_;
  }

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const
  {
    return class_id_;
  }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId(const QString& class_id)
  {
    class_id_ = class_id;
  }

  void load(const Config& config) override;
  void save(Config config) const override;

  bool isActive() const
  {
    return is_active_;
  }

  /** @return A mouse cursor representing the current state */
  virtual QCursor getCursor()
  {
    return cursor_;
  }

Q_SIGNALS:
  void configChanged();

private Q_SLOTS:

  void updateNearClipDistance();
  void updateStereoProperties();
  void updateInvertZAxis();

protected:
  /** @brief Do subclass-specific initialization.  Called by
   * ViewController::initialize after context_ and camera_ are set.
   * Default implementation does nothing. */
  virtual void onInitialize()
  {
  }

  /** @brief called by activate().
   *
   * Override to implement view-specific activation.  This base
   * implementation does nothing. */
  virtual void onActivate()
  {
  }

  // choose a cursor from the standard set
  enum CursorType
  {
    Default,
    Rotate2D,
    Rotate3D,
    MoveXY,
    MoveZ,
    Zoom,
    Crosshair
  };
  void setCursor(CursorType cursor_type);

  // set a custom cursor
  void setCursor(QCursor cursor)
  {
    cursor_ = std::move(cursor);
  }

  DisplayContext* context_;
  Ogre::Camera* camera_;

  bool is_active_;

  // this cursor will be displayed when the mouse is within the
  // window controlled by this view controller
  // use SetCursor to modify.
  QCursor cursor_;

  FloatProperty* near_clip_property_;
  BoolProperty* stereo_enable_;
  BoolProperty* stereo_eye_swap_;
  FloatProperty* stereo_eye_separation_;
  FloatProperty* stereo_focal_distance_;
  BoolProperty* invert_z_;

  void setStatus(const QString& message);

private:
  EnumProperty* type_property_;
  QString class_id_;

  // Default cursors for the most common actions
  QMap<CursorType, QCursor> standard_cursors_;
};

} // end namespace rviz

#endif // RVIZ_VIEW_CONTROLLER_H
