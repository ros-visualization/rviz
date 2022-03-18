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

#include "grid.h"
#include "billboard_line.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <sstream>

namespace rviz
{
Grid::Grid(Ogre::SceneManager* scene_manager,
           Ogre::SceneNode* parent_node,
           Style style,
           uint32_t cell_count,
           float cell_length,
           float line_width,
           const Ogre::ColourValue& color)
  : scene_manager_(scene_manager)
  , style_(style)
  , cell_count_(cell_count)
  , cell_length_(cell_length)
  , line_width_(line_width)
  , height_(0)
  , color_(color)
{
  static uint32_t gridCount = 0;
  std::stringstream ss;
  ss << "Grid" << gridCount++;

  manual_object_ = scene_manager_->createManualObject(ss.str());

  if (!parent_node)
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  scene_node_->attachObject(manual_object_);

  billboard_line_ = new BillboardLine(scene_manager, scene_node_);

  ss << "Material";
  material_ = Ogre::MaterialManager::getSingleton().create(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);

  setColor(color_);
}

Grid::~Grid()
{
  delete billboard_line_;

  scene_manager_->destroySceneNode(scene_node_->getName());
  scene_manager_->destroyManualObject(manual_object_);

  material_->unload();
}

void Grid::setCellCount(uint32_t count)
{
  cell_count_ = count;

  create();
}

void Grid::setCellLength(float len)
{
  cell_length_ = len;

  create();
}

void Grid::setLineWidth(float width)
{
  line_width_ = width;

  create();
}

void Grid::setColor(const Ogre::ColourValue& color)
{
  color_ = color;

  if (color_.a < 0.9998)
  {
    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setDepthWriteEnabled(false);
  }
  else
  {
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(true);
  }

  create();
}

void Grid::setStyle(Style style)
{
  style_ = style;

  create();
}

void Grid::setHeight(uint32_t height)
{
  height_ = height;

  create();
}

void Grid::create()
{
  manual_object_->clear();
  billboard_line_->clear();

  float extent = (cell_length_ * ((double)cell_count_)) / 2;

  if (style_ == Billboards)
  {
    billboard_line_->setColor(color_.r, color_.g, color_.b, color_.a);
    billboard_line_->setLineWidth(line_width_);
    billboard_line_->setMaxPointsPerLine(2);
    billboard_line_->setNumLines((cell_count_ + 1) * 2 * (height_ + 1) +
                                 ((cell_count_ + 1) * (cell_count_ + 1)) * height_);
  }
  else
  {
    manual_object_->estimateVertexCount(cell_count_ * 4 * (height_ + 1) +
                                        ((cell_count_ + 1) * (cell_count_ + 1) * height_));
    manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST);
  }

  for (uint32_t h = 0; h <= height_; ++h)
  {
    float h_real = (height_ / 2.0f - (float)h) * cell_length_;
    for (uint32_t i = 0; i <= cell_count_; i++)
    {
      float inc = extent - (i * cell_length_);

      Ogre::Vector3 p1(inc, h_real, -extent);
      Ogre::Vector3 p2(inc, h_real, extent);
      Ogre::Vector3 p3(-extent, h_real, inc);
      Ogre::Vector3 p4(extent, h_real, inc);

      if (style_ == Billboards)
      {
        if (h != 0 || i != 0)
        {
          billboard_line_->newLine();
        }

        billboard_line_->addPoint(p1);
        billboard_line_->addPoint(p2);

        billboard_line_->newLine();

        billboard_line_->addPoint(p3);
        billboard_line_->addPoint(p4);
      }
      else
      {
        manual_object_->position(p1);
        manual_object_->colour(color_);
        manual_object_->position(p2);
        manual_object_->colour(color_);

        manual_object_->position(p3);
        manual_object_->colour(color_);
        manual_object_->position(p4);
        manual_object_->colour(color_);
      }
    }
  }

  if (height_ > 0)
  {
    for (uint32_t x = 0; x <= cell_count_; ++x)
    {
      for (uint32_t z = 0; z <= cell_count_; ++z)
      {
        float x_real = extent - x * cell_length_;
        float z_real = extent - z * cell_length_;

        float y_top = (height_ / 2.0f) * cell_length_;
        float y_bottom = -y_top;

        if (style_ == Billboards)
        {
          billboard_line_->newLine();

          billboard_line_->addPoint(Ogre::Vector3(x_real, y_bottom, z_real));
          billboard_line_->addPoint(Ogre::Vector3(x_real, y_top, z_real));
        }
        else
        {
          manual_object_->position(x_real, y_bottom, z_real);
          manual_object_->colour(color_);
          manual_object_->position(x_real, y_top, z_real);
          manual_object_->colour(color_);
        }
      }
    }
  }

  if (style_ == Lines)
  {
    manual_object_->end();
  }
}

void Grid::setUserData(const Ogre::Any& data)
{
  manual_object_->getUserObjectBindings().setUserAny(data);
}

} // namespace rviz
