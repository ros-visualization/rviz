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

#include <QStringList>

#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/property_tree_model.h>

namespace rviz
{
QuaternionProperty::QuaternionProperty(const QString& name,
                                       const Ogre::Quaternion& default_value,
                                       const QString& description,
                                       Property* parent,
                                       const char* changed_slot,
                                       QObject* receiver)
  : Property(name, QVariant(), description, parent, changed_slot, receiver)
  , quaternion_(default_value)
  , ignore_child_updates_(false)
{
  x_ = new Property("X", quaternion_.x, "X coordinate", this);
  y_ = new Property("Y", quaternion_.y, "Y coordinate", this);
  z_ = new Property("Z", quaternion_.z, "Z coordinate", this);
  w_ = new Property("W", quaternion_.w, "W coordinate", this);
  updateString();
  connect(x_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(y_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(z_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(w_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(x_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
  connect(y_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
  connect(z_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
  connect(w_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
}

bool QuaternionProperty::setQuaternion(const Ogre::Quaternion& new_quaternion)
{
  if (new_quaternion != quaternion_)
  {
    Q_EMIT aboutToChange();
    quaternion_ = new_quaternion;
    ignore_child_updates_ = true;
    x_->setValue(quaternion_.x);
    y_->setValue(quaternion_.y);
    z_->setValue(quaternion_.z);
    w_->setValue(quaternion_.w);
    ignore_child_updates_ = false;
    updateString();
    Q_EMIT changed();
    if (model_)
      model_->emitDataChanged(this);
    return true;
  }
  return false;
}

bool QuaternionProperty::setValue(const QVariant& new_value)
{
  QStringList strings = new_value.toString().split(';');
  if (strings.size() >= 4)
  {
    bool x_ok = true;
    float x = strings[0].toFloat(&x_ok);
    bool y_ok = true;
    float y = strings[1].toFloat(&y_ok);
    bool z_ok = true;
    float z = strings[2].toFloat(&z_ok);
    bool w_ok = true;
    float w = strings[3].toFloat(&w_ok);
    if (x_ok && y_ok && z_ok && w_ok)
    {
      return setQuaternion(Ogre::Quaternion(w, x, y, z));
    }
  }
  return false;
}

void QuaternionProperty::updateFromChildren()
{
  if (!ignore_child_updates_)
  {
    quaternion_.x = x_->getValue().toFloat();
    quaternion_.y = y_->getValue().toFloat();
    quaternion_.z = z_->getValue().toFloat();
    quaternion_.w = w_->getValue().toFloat();
    updateString();
    Q_EMIT changed();
  }
}

void QuaternionProperty::emitAboutToChange()
{
  if (!ignore_child_updates_)
  {
    Q_EMIT aboutToChange();
  }
}

void QuaternionProperty::updateString()
{
  value_ = QString("%1; %2; %3; %4")
               .arg(quaternion_.x, 0, 'g', 5)
               .arg(quaternion_.y, 0, 'g', 5)
               .arg(quaternion_.z, 0, 'g', 5)
               .arg(quaternion_.w, 0, 'g', 5);
}

void QuaternionProperty::load(const Config& config)
{
  float x, y, z, w;
  if (config.mapGetFloat("X", &x) && config.mapGetFloat("Y", &y) && config.mapGetFloat("Z", &z) &&
      config.mapGetFloat("W", &w))
  {
    // Calling setQuaternion() once explicitly is better than letting
    // the Property class load the X, Y, Z, and W children
    // independently, which would result in at least 4 calls to
    // setQuaternion().
    setQuaternion(Ogre::Quaternion(w, x, y, z));
  }
}

void QuaternionProperty::save(Config config) const
{
  // Saving the child values explicitly avoids having Property::save()
  // save the summary string version of the property.
  if (getReadOnly())
    return;
  config.mapSetValue("X", x_->getValue());
  config.mapSetValue("Y", y_->getValue());
  config.mapSetValue("Z", z_->getValue());
  config.mapSetValue("W", w_->getValue());
}

void QuaternionProperty::setReadOnly(bool read_only)
{
  Property::setReadOnly(read_only);
  x_->setReadOnly(read_only);
  y_->setReadOnly(read_only);
  z_->setReadOnly(read_only);
  w_->setReadOnly(read_only);
}


} // end namespace rviz
