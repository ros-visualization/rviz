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

#ifndef RVIZ_TOOL_H
#define RVIZ_TOOL_H

#include <QString>
#include <QIcon>
#include <QCursor>
#include <QObject>

#include <rviz/config.h>
#include <rviz/rviz_export.h>

class QMouseEvent;
class QKeyEvent;

namespace Ogre
{
class SceneManager;
}

namespace rviz
{
class DisplayContext;
class Property;
class RenderPanel;
class ViewportMouseEvent;

class RVIZ_EXPORT Tool : public QObject
{
  Q_OBJECT
public:
  /** Default constructor.  Pluginlib only instantiates classes via
   * default constructors.  Subclasses of Tool should shortcut_key_
   * field in their constructors.
   *
   * Properties to appear in the Tool Properties panel are typically
   * created in the constructor, as children of the property from
   * getPropertyContainer(), which is set up in this Tool
   * constructor. */
  Tool();
  ~Tool() override;

  /** Initialize the tool.  Sets the DisplayContext and calls
   * onInitialize(). */
  void initialize(DisplayContext* context);

  /** @brief Return the container for properties of this Tool. */
  virtual Property* getPropertyContainer() const
  {
    return property_container_;
  }

  char getShortcutKey()
  {
    return shortcut_key_;
  }

  bool accessAllKeys()
  {
    return access_all_keys_;
  }

  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void update(float wall_dt, float ros_dt)
  {
    (void)wall_dt;
    (void)ros_dt;
  }

  enum
  {
    Render = 1,
    Finished = 2
  };

  /** Process a mouse event.  This is the central function of all the
   * tools, as it defines how the mouse is used. */
  virtual int processMouseEvent(ViewportMouseEvent& event)
  {
    (void)event;
    return 0;
  }

  /** Process a key event.  Override if your tool should handle any
      other keypresses than the tool shortcuts, which are handled
      separately. */
  virtual int processKeyEvent(QKeyEvent* event, RenderPanel* panel)
  {
    (void)event;
    (void)panel;
    return 0;
  }

  QString getName() const
  {
    return name_;
  }

  /** @brief Set the name of the tool.
   *
   * This is called by ToolManager during tool initialization.  If you
   * want a different name than it gives you, call this from
   * onInitialize() (or thereafter). */
  void setName(const QString& name);

  /** @brief Set the description of the tool.  This is called by
   * ToolManager during tool initialization. */
  QString getDescription() const
  {
    return description_;
  }
  void setDescription(const QString& description);

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

  /** @brief Load properties from the given Config.
   *
   * Most tools won't need to override this, because any child
   * Properties of property_container_ are automatically loaded by
   * this function. */
  virtual void load(const Config& config);

  /** @brief Save this entire tool into the given Config node.
   *
   * Most tools won't need to override this, because any child
   * Properties of property_container_ are automatically saved by
   * this function. */
  virtual void save(Config config) const;

  /** @brief Set the toolbar icon for this tool (will also set its cursor). */
  void setIcon(const QIcon& icon);

  /** @brief Get the icon of this tool. */
  const QIcon& getIcon()
  {
    return icon_;
  }

  /** @brief Set the cursor for this tool. */
  void setCursor(const QCursor& cursor);

  /** @brief Get current cursor of this tool. */
  const QCursor& getCursor()
  {
    return cursor_;
  }

  void setStatus(const QString& message);

Q_SIGNALS:
  void close();

protected:
  /** Override onInitialize to do any setup needed after the
      DisplayContext has been set.  This is called by
      Tool::initialize().  The base implementation here does
      nothing. */
  virtual void onInitialize()
  {
  }

  Ogre::SceneManager* scene_manager_;
  DisplayContext* context_;

  char shortcut_key_;
  bool access_all_keys_;

  QIcon icon_;

  QCursor cursor_;

private:
  QString class_id_;
  Property* property_container_;
  QString name_;
  QString description_;
};

} // namespace rviz

#endif
