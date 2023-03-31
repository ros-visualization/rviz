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

#include <boost/bind/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreRibbonTrail.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>

#include "axes_display.h"

namespace rviz
{
AxesDisplay::AxesDisplay() : Display(), axes_(nullptr), trail_(nullptr)
{
  frame_property_ = new TfFrameProperty("Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
                                        "The TF frame these axes will use for their origin.", this,
                                        nullptr, true, [this] { resetTrail(); });

  length_property_ = new FloatProperty("Length", 1.0, "Length of each axis, in meters.", this,
                                       &AxesDisplay::updateShape);
  length_property_->setMin(0.0001);

  radius_property_ = new FloatProperty("Radius", 0.1, "Radius of each axis, in meters.", this,
                                       &AxesDisplay::updateShape);
  radius_property_->setMin(0.0001);

  trail_property_ =
      new Property("Show Trail", false, "Enable/disable a 2 meter \"ribbon\" which follows this frame.",
                   this, &AxesDisplay::updateTrail);

  alpha_property_ = new FloatProperty("Alpha", 1.0, "Alpha channel value of each axis.", this,
                                      &AxesDisplay::updateShape);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);
}

AxesDisplay::~AxesDisplay()
{
  if (trail_)
  {
    scene_manager_->destroyRibbonTrail(trail_);
  }

  delete axes_;
}

void AxesDisplay::onInitialize()
{
  frame_property_->setFrameManager(context_->getFrameManager());

  axes_ = new Axes(scene_manager_, nullptr, length_property_->getFloat(), radius_property_->getFloat(),
                   alpha_property_->getFloat());
  axes_->getSceneNode()->setVisible(isEnabled());
}

void AxesDisplay::reset()
{
  Display::reset();
  resetTrail(false);
}

void AxesDisplay::resetTrail(bool update)
{
  if (!trail_)
    return;
  if (update)
    this->update(0.0, 0.0);
  trail_->nodeUpdated(axes_->getSceneNode());
  trail_->clearAllChains();
}

void AxesDisplay::onEnable()
{
  axes_->getSceneNode()->setVisible(true);
  if (trail_)
    trail_->setVisible(true);
}

void AxesDisplay::onDisable()
{
  axes_->getSceneNode()->setVisible(false);
  if (trail_)
    trail_->setVisible(false);
}

void AxesDisplay::updateShape()
{
  axes_->set(length_property_->getFloat(), radius_property_->getFloat(), alpha_property_->getFloat());
  context_->queueRender();
}

void AxesDisplay::updateTrail()
{
  if (trail_property_->getValue().toBool())
  {
    if (!trail_)
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Trail for frame " << frame_property_->getFrame().toStdString() << count++;
      trail_ = scene_manager_->createRibbonTrail(ss.str());
      trail_->setMaxChainElements(100);
      trail_->setInitialWidth(0, 0.01f);
      trail_->setInitialColour(0, 1, 0, 0);
      trail_->addNode(axes_->getSceneNode());
      trail_->setTrailLength(2.0f);
      trail_->setVisible(isEnabled());
      axes_->getSceneNode()->getParentSceneNode()->attachObject(trail_);
    }
  }
  else
  {
    if (trail_)
    {
      scene_manager_->destroyRibbonTrail(trail_);
      trail_ = nullptr;
    }
  }
}

void AxesDisplay::update(float /*dt*/, float /*ros_dt*/)
{
  QString qframe = frame_property_->getFrame();
  std::string frame = qframe.toStdString();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
  {
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }
  else
  {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
    {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(StatusProperty::Error, "Transform",
                "Could not transform from [" + qframe + "] to Fixed Frame [" + fixed_frame_ +
                    "] for an unknown reason");
    }
  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::AxesDisplay, rviz::Display)
