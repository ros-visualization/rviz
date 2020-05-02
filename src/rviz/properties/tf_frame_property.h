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
#ifndef TF_FRAME_PROPERTY_H
#define TF_FRAME_PROPERTY_H

#include <string>

#include "rviz/properties/editable_enum_property.h"
#include "rviz/rviz_export.h"

namespace rviz
{
class FrameManager;

class RVIZ_EXPORT TfFrameProperty : public EditableEnumProperty
{
  Q_OBJECT
public:
  TfFrameProperty(const QString& name = QString(),
                  const QString& default_value = QString(),
                  const QString& description = QString(),
                  Property* parent = nullptr,
                  FrameManager* frame_manager = nullptr,
                  bool include_fixed_frame_string = false,
                  const char* changed_slot = nullptr,
                  QObject* receiver = nullptr);

  /** @brief Override from Property to resolve the frame name on the way in. */
  bool setValue(const QVariant& new_value) override;

  QString getFrame() const;
  std::string getFrameStd() const;

  void setFrameManager(FrameManager* frame_manager);
  FrameManager* getFrameManager() const
  {
    return frame_manager_;
  }

  static const QString FIXED_FRAME_STRING;

private Q_SLOTS:
  void fillFrameList();

  /** @brief If this property is currently set to FIXED_FRAME_STRING,
   * this emits changed() to let users know that a call to getFrame()
   * will now return something different. */
  void handleFixedFrameChange();

private:
  FrameManager* frame_manager_;
  bool include_fixed_frame_string_;
};

} // end namespace rviz

#endif // TF_FRAME_PROPERTY_H
