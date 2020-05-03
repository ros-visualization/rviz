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
#ifndef RVIZ_PANEL_H
#define RVIZ_PANEL_H

#include <QWidget>

#include <rviz/config.h>
#include <rviz/rviz_export.h>

namespace rviz
{
class VisualizationManager;

class RVIZ_EXPORT Panel : public QWidget
{
  Q_OBJECT
public:
  Panel(QWidget* parent = nullptr);
  ~Panel() override;

  /** Initialize the panel with a VisualizationManager.  Called by
   * VisualizationFrame during setup. */
  void initialize(VisualizationManager* manager);

  /**
   * Override to do initialization which depends on the
   * VisualizationManager being available.  This base implementation
   * does nothing.
   */
  virtual void onInitialize()
  {
  }

  virtual QString getName() const
  {
    return name_;
  }
  virtual void setName(const QString& name)
  {
    name_ = name;
  }

  /** @brief Return a description of this Panel. */
  virtual QString getDescription() const
  {
    return description_;
  }

  /** @brief Set a description of this Panel.  Called by the factory which creates it. */
  virtual void setDescription(const QString& description)
  {
    description_ = description;
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

  /** @brief Override to load configuration data.  This version loads the name of the panel. */
  virtual void load(const Config& config);

  /** @brief Override to save configuration data.  This version saves the name and class ID of the panel.
   */
  virtual void save(Config config) const;

Q_SIGNALS:
  /** @brief Subclasses must emit this whenever a configuration change
   *         happens.
   *
   * This is used to let the system know that changes have been made
   * since the last time the config was saved. */
  void configChanged();

protected:
  VisualizationManager* vis_manager_;

private:
  QString class_id_;
  QString name_;
  QString description_;
};

} // end namespace rviz

#endif // RVIZ_PANEL_H
