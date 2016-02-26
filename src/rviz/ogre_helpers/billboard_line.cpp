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

#include "billboard_line.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreBillboardChain.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <sstream>

#include <ros/assert.h>

#define MAX_ELEMENTS (65536/4)

namespace rviz
{

BillboardLine::BillboardLine( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
: Object( scene_manager )
, width_( 0.1f )
, current_line_(0)
, total_elements_(0)
, num_lines_(1)
, max_points_per_line_(100)
, lines_per_chain_(0)
, current_chain_(0)
, elements_in_current_chain_(0)
{
  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "BillboardLineMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);

  setNumLines(num_lines_);
  setMaxPointsPerLine(max_points_per_line_);
}

BillboardLine::~BillboardLine()
{
  V_Chain::iterator it = chains_.begin();
  V_Chain::iterator end = chains_.end();
  for (;it != end; ++it)
  {
    scene_manager_->destroyBillboardChain(*it);
  }

  scene_manager_->destroySceneNode( scene_node_->getName() );

  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

Ogre::BillboardChain* BillboardLine::createChain()
{
  std::stringstream ss;
  static int count = 0;
  ss << "BillboardLine chain" << count++;
  Ogre::BillboardChain* chain = scene_manager_->createBillboardChain(ss.str());
  chain->setMaterialName( material_->getName() );
  scene_node_->attachObject( chain );

  chains_.push_back(chain);

  return chain;
}

void BillboardLine::clear()
{
  V_Chain::iterator it = chains_.begin();
  V_Chain::iterator end = chains_.end();
  for (; it != end; ++it)
  {
    (*it)->clearAllChains();
  }

  current_line_ = 0;
  total_elements_ = 0;
  current_chain_ = 0;
  elements_in_current_chain_ = 0;

  for (V_uint32::iterator it = num_elements_.begin(); it != num_elements_.end(); ++it)
  {
    *it = 0;
  }
}

void BillboardLine::setupChains()
{
  uint32_t total_points = max_points_per_line_ * num_lines_;
  uint32_t num_chains = total_points / MAX_ELEMENTS;
  if (total_points % MAX_ELEMENTS != 0)
  {
    ++num_chains;
  }

  for (uint32_t i = chains_.size(); i < num_chains; ++i)
  {
    createChain();
  }

  lines_per_chain_ = max_points_per_line_ > 0 ? MAX_ELEMENTS / max_points_per_line_ : 1;

  V_Chain::iterator it = chains_.begin();
  V_Chain::iterator end = chains_.end();
  for (;it != end; ++it)
  {
    (*it)->setMaxChainElements(max_points_per_line_);

    // shorten the number of chains in the last bbchain, to avoid memory wasteage
    if (it + 1 == end)
    {
      uint32_t lines_left = num_lines_ % lines_per_chain_;

      // Handle the case where num_lines_ is a multiple of lines_per_chain
      if (lines_left == 0) {
          (*it)->setNumberOfChains(lines_per_chain_);
      }
      else
      {
          (*it)->setNumberOfChains(lines_left);
      }
    }
    else
    {
      (*it)->setNumberOfChains(lines_per_chain_);
    }
  }
}

void BillboardLine::setMaxPointsPerLine(uint32_t max)
{
  max_points_per_line_ = max;

  setupChains();
}

void BillboardLine::setNumLines(uint32_t num)
{
  num_lines_ = num;

  setupChains();

  num_elements_.resize(num);

  for (V_uint32::iterator it = num_elements_.begin(); it != num_elements_.end(); ++it)
  {
    *it = 0;
  }
}

void BillboardLine::newLine()
{
  ++current_line_;

  ROS_ASSERT(current_line_ < num_lines_);
}

void BillboardLine::addPoint( const Ogre::Vector3& point )
{
  addPoint(point, color_);
}

void BillboardLine::addPoint( const Ogre::Vector3& point, const Ogre::ColourValue& color )
{
  ++num_elements_[current_line_];
  ++total_elements_;

  ROS_ASSERT(num_elements_[current_line_] <= max_points_per_line_);

  ++elements_in_current_chain_;
  if (elements_in_current_chain_ > MAX_ELEMENTS)
  {
    ++current_chain_;
    elements_in_current_chain_ = 1;
  }

  Ogre::BillboardChain::Element e;
  e.position = point;
  e.width = width_;
  e.colour = color;
  chains_[current_chain_]->addChainElement(current_line_ % lines_per_chain_ , e);
}

void BillboardLine::setLineWidth( float width )
{
  width_ = width;

  for (uint32_t line = 0; line < num_lines_; ++line)
  {
    uint32_t element_count = num_elements_[line];

    for ( uint32_t i = 0; i < element_count; ++i )
    {
      Ogre::BillboardChain* c = chains_[line / lines_per_chain_];
      Ogre::BillboardChain::Element e = c->getChainElement(line % lines_per_chain_, i);

      e.width = width_;
      c->updateChainElement(line % lines_per_chain_, i, e);
    }
  }
}

void BillboardLine::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void BillboardLine::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void BillboardLine::setScale( const Ogre::Vector3& scale )
{
  // Setting scale doesn't really make sense here
}

void BillboardLine::setColor( float r, float g, float b, float a )
{
  if ( a < 0.9998 )
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    material_->getTechnique(0)->setDepthWriteEnabled( true );
  }

  color_ = Ogre::ColourValue( r, g, b, a );

  for (uint32_t line = 0; line < num_lines_; ++line)
  {
    uint32_t element_count = num_elements_[line];

    for ( uint32_t i = 0; i < element_count; ++i )
    {
      Ogre::BillboardChain* c = chains_[line / lines_per_chain_];
      Ogre::BillboardChain::Element e = c->getChainElement(line % lines_per_chain_, i);

      e.colour = color_;
      c->updateChainElement(line % lines_per_chain_, i, e);
    }
  }
}

const Ogre::Vector3& BillboardLine::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& BillboardLine::getOrientation()
{
  return scene_node_->getOrientation();
}

} // namespace rviz
