/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
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

#ifndef COVARIANCE_PROPERTY_H
#define COVARIANCE_PROPERTY_H

#include <QColor>

#include <OgreColourValue.h>

#include <rviz/properties/bool_property.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
} // namespace Ogre

namespace rviz
{
class Property;
class ColorProperty;
class FloatProperty;
class EnumProperty;
class CovarianceVisual;

/** @brief Property specialized to provide getter for booleans. */
class CovarianceProperty : public rviz::BoolProperty
{
  Q_OBJECT
public:
  typedef boost::shared_ptr<CovarianceVisual> CovarianceVisualPtr;

  enum Frame
  {
    Local,
    Fixed,
  };

  enum ColorStyle
  {
    Unique,
    RGB,
  };

  CovarianceProperty(const QString& name = "Covariance",
                     bool default_value = false,
                     const QString& description = QString(),
                     rviz::Property* parent = nullptr,
                     const char* changed_slot = nullptr,
                     QObject* receiver = nullptr);

  ~CovarianceProperty() override;

  bool getPositionBool();
  bool getOrientationBool();

  // Methods to manage the deque of Covariance Visuals
  CovarianceVisualPtr createAndPushBackVisual(Ogre::SceneManager* scene_manager,
                                              Ogre::SceneNode* parent_node);
  void popFrontVisual();
  void clearVisual();
  size_t sizeVisual();

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateColorAndAlphaAndScaleAndOffset();
  void updateOrientationFrame();
  void updateColorStyleChoice();

private:
  void updateColorAndAlphaAndScaleAndOffset(const CovarianceVisualPtr& visual);
  void updateOrientationFrame(const CovarianceVisualPtr& visual);
  void updateVisibility(const CovarianceVisualPtr& visual);

  typedef std::deque<CovarianceVisualPtr> D_Covariance;
  D_Covariance covariances_;

  rviz::BoolProperty* position_property_;
  rviz::ColorProperty* position_color_property_;
  rviz::FloatProperty* position_alpha_property_;
  rviz::FloatProperty* position_scale_property_;
  rviz::BoolProperty* orientation_property_;
  rviz::EnumProperty* orientation_frame_property_;
  rviz::EnumProperty* orientation_colorstyle_property_;
  rviz::ColorProperty* orientation_color_property_;
  rviz::FloatProperty* orientation_alpha_property_;
  rviz::FloatProperty* orientation_offset_property_;
  rviz::FloatProperty* orientation_scale_property_;
};

} // end namespace rviz

#endif // COVARIANCE_PROPERTY_H
