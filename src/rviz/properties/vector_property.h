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
#ifndef VECTOR_PROPERTY_H
#define VECTOR_PROPERTY_H

#include <OgreVector3.h>

#include "rviz/properties/property.h"
#include "rviz/rviz_export.h"

namespace rviz
{
class RVIZ_EXPORT VectorProperty : public Property
{
  Q_OBJECT
public:
  VectorProperty(const QString& name = QString(),
                 const Ogre::Vector3& default_value = Ogre::Vector3::ZERO,
                 const QString& description = QString(),
                 Property* parent = nullptr,
                 const char* changed_slot = nullptr,
                 QObject* receiver = nullptr);

  virtual bool setVector(const Ogre::Vector3& vector);
  virtual Ogre::Vector3 getVector() const
  {
    return vector_;
  }
  bool add(const Ogre::Vector3& offset)
  {
    return setVector(getVector() + offset);
  }

  bool setValue(const QVariant& new_value) override;

  void load(const Config& config) override;
  void save(Config config) const override;

  /** @brief Overridden from Property to propagate read-only-ness to children. */
  void setReadOnly(bool read_only) override;

private Q_SLOTS:
  void updateFromChildren();
  void emitAboutToChange();

private:
  void updateString();

  Ogre::Vector3 vector_;
  Property* x_;
  Property* y_;
  Property* z_;

  bool ignore_child_updates_;
};

} // end namespace rviz

#endif // VECTOR_PROPERTY_H
