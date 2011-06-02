/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "mesh_resource_marker.h"

#include "marker_selection_handler.h"
#include "rviz/default_plugin/marker_display.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/visualization_manager.h"
#include "rviz/mesh_loader.h"
#include "marker_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

namespace rviz
{

MeshResourceMarker::MeshResourceMarker(MarkerDisplay* owner, VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(owner, manager, parent_node)
, entity_(0)
{
}

MeshResourceMarker::~MeshResourceMarker()
{
  reset();
}

void MeshResourceMarker::reset()
{
  //destroy entity
  if (entity_)
  {
    vis_manager_->getSceneManager()->destroyEntity( entity_ );
    entity_ = 0;
  }

  // destroy all the materials we've created
  S_MaterialPtr::iterator it;
  for ( it = materials_.begin(); it!=materials_.end(); it++ )
  {
    Ogre::MaterialPtr material = *it;
    if (!material.isNull())
    {
      for (size_t i = 0; i < material->getNumTechniques(); ++i)
      {
        Ogre::Technique* t = material->getTechnique(i);
        // hack hack hack, really need to do a shader-based way of picking, rather than
        // creating a texture for each object
        if (t->getSchemeName() == "Pick")
        {
          Ogre::TextureManager::getSingleton().remove(t->getPass(0)->getTextureUnitState(0)->getTextureName());
        }
      }

      material->unload();
      Ogre::MaterialManager::getSingleton().remove(material->getName());
    }
  }
  materials_.clear();
}

void MeshResourceMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::MESH_RESOURCE);

  scene_node_->setVisible(false);

  if (!entity_ || old_message->mesh_resource != new_message->mesh_resource)
  {
    reset();

    if (new_message->mesh_resource.empty())
    {
      return;
    }

    if (loadMeshFromResource(new_message->mesh_resource).isNull())
    {
      std::stringstream ss;
      ss << "Mesh resource marker [" << getStringID() << "] could not load [" << new_message->mesh_resource << "]";
      if ( owner_ )
      {
        owner_->setMarkerStatus(getID(), status_levels::Error, ss.str());
      }
      ROS_DEBUG("%s", ss.str().c_str());
      return;
    }

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "mesh_resource_marker_" << count++;
    std::string id = ss.str();
    entity_ = vis_manager_->getSceneManager()->createEntity(id, new_message->mesh_resource);
    scene_node_->attachObject(entity_);

    if ( new_message->mesh_use_embedded_materials )
    {
      // make clones of all embedded materials so selection works correctly
      S_MaterialPtr materials = getMaterials();

      S_MaterialPtr::iterator it;
      for ( it = materials.begin(); it!=materials.end(); it++ )
      {
        Ogre::MaterialPtr new_material = (*it)->clone( id + (*it)->getName() );
        materials_.insert( new_material );
      }

      // make sub-entities use cloned materials
      for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i)
      {
        std::string mat_name = entity_->getSubEntity(i)->getMaterialName();
        entity_->getSubEntity(i)->setMaterialName( id + mat_name );
      }
    }
    else
    {
      // create a new single material for all the sub-entities
      ss << "Material";
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create( ss.str(), ROS_PACKAGE_NAME );
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(true);
      material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );

      float r = new_message->color.r;
      float g = new_message->color.g;
      float b = new_message->color.b;
      float a = new_message->color.a;
      material->getTechnique(0)->setAmbient( r*0.5, g*0.5, b*0.5 );
      material->getTechnique(0)->setDiffuse( r, g, b, a );

      if ( a < 0.9998 )
      {
        material->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
        material->getTechnique(0)->setDepthWriteEnabled( false );
      }
      else
      {
        material->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
        material->getTechnique(0)->setDepthWriteEnabled( true );
      }
      entity_->setMaterial( material );
      materials_.insert( material );
    }

    coll_ = vis_manager_->getSelectionManager()->createCollisionForEntity(entity_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id))), coll_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);
  scene_node_->setScale(scale);
}

S_MaterialPtr MeshResourceMarker::getMaterials()
{
  S_MaterialPtr materials;
  extractMaterials( entity_, materials );
  return materials;
}

}

