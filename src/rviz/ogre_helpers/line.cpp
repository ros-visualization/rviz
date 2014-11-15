/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include "line.h"

#include <sstream>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

namespace rviz
{

Line::Line( Ogre::SceneManager* manager, Ogre::SceneNode* parent_node )
  : Object( manager )
{
  if (!parent_node)
  {
    parent_node = manager->getRootSceneNode();
  }
  manual_object_ =  manager->createManualObject();
  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "LineMaterial" << count++;

  // NOTE: The second parameter to the create method is the resource group the material will be added to.
  // If the group you name does not exist (in your resources.cfg file) the library will assert() and your program will crash
  manual_object_material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),"rviz");
  manual_object_material_->setReceiveShadows(false);
  manual_object_material_->getTechnique(0)->setLightingEnabled(true);
  manual_object_material_->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0);
  manual_object_material_->getTechnique(0)->getPass(0)->setAmbient(1,1,1);

  scene_node_->attachObject(manual_object_);
}

Line::~Line()
{
  if ( scene_node_->getParentSceneNode() )
  {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject( manual_object_ );
  Ogre::MaterialManager::getSingleton().remove(manual_object_material_->getName());
}

void Line::setPoints( Ogre::Vector3 start, Ogre::Vector3 end )
{
  manual_object_->clear();
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_LINE_LIST);
  manual_object_->position(start);
  manual_object_->position(end);
  manual_object_->end();
  setVisible(true);
}

void Line::setVisible( bool visible )
{
  scene_node_->setVisible(visible,true);
}


void Line::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void Line::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void Line::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( scale );
}

void Line::setColor( const Ogre::ColourValue& c )
{
  // this is consistent with the behaviour in the Shape class.

  manual_object_material_->getTechnique(0)->setAmbient( c * 0.5 );
  manual_object_material_->getTechnique(0)->setDiffuse( c );

  if ( c.a < 0.9998 )
  {
    manual_object_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    manual_object_material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    manual_object_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    manual_object_material_->getTechnique(0)->setDepthWriteEnabled( true );
  }
}

void Line::setColor( float r, float g, float b, float a )
{
  setColor(Ogre::ColourValue(r, g, b, a));
}

// where are the void Line::setColour(...) convenience methods??? ;)

const Ogre::Vector3& Line::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& Line::getOrientation()
{
  return scene_node_->getOrientation();
}

void Line::setUserData( const Ogre::Any& data )
{
  manual_object_->setUserAny( data );
}


}
