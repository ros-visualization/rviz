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

  bool need_color = false;

  scene_node_->setVisible(false);

  if( !entity_ ||
      old_message->mesh_resource != new_message->mesh_resource ||
      old_message->mesh_use_embedded_materials != new_message->mesh_use_embedded_materials )
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
    need_color = true;

    // create a default material for any sub-entities which don't have their own.
    ss << "Material";
    Ogre::MaterialPtr default_material = Ogre::MaterialManager::getSingleton().create( ss.str(), ROS_PACKAGE_NAME );
    default_material->setReceiveShadows(false);
    default_material->getTechnique(0)->setLightingEnabled(true);
    default_material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
    materials_.insert( default_material );

    if ( new_message->mesh_use_embedded_materials )
    {
      // make clones of all embedded materials so selection works correctly
      S_MaterialPtr materials = getMaterials();

      S_MaterialPtr::iterator it;
      for ( it = materials.begin(); it!=materials.end(); it++ )
      {
        if( (*it)->getName() != "BaseWhiteNoLighting" )
        {
          Ogre::MaterialPtr new_material = (*it)->clone( id + (*it)->getName() );
          materials_.insert( new_material );
        }
      }

      // make sub-entities use cloned materials
      for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i)
      {
        std::string mat_name = entity_->getSubEntity(i)->getMaterialName();
        if( mat_name != "BaseWhiteNoLighting" )
        {
          entity_->getSubEntity(i)->setMaterialName( id + mat_name );
        }
        else
        {
          // BaseWhiteNoLighting is the default material Ogre uses
          // when it sees a mesh with no material.  Here we replace
          // that with our default_material which gets colored with
          // new_message->color.
          entity_->getSubEntity(i)->setMaterial( default_material );
        }
      }
    }
    else
    {
      entity_->setMaterial( default_material );
    }

    vis_manager_->getSelectionManager()->removeObject(coll_);
    coll_ = vis_manager_->getSelectionManager()->createCollisionForEntity(entity_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id))), coll_);
  }

  if( need_color ||
      old_message->color.r != new_message->color.r ||
      old_message->color.g != new_message->color.g ||
      old_message->color.b != new_message->color.b ||
      old_message->color.a != new_message->color.a )
  {
    float r = new_message->color.r;
    float g = new_message->color.g;
    float b = new_message->color.b;
    float a = new_message->color.a;

    // Old way was to ignore the color and alpha when using embedded
    // materials, which meant you could leave them unset, which means
    // 0.  Since we now USE the color and alpha values, leaving them
    // all 0 will mean the object will be invisible.  Therefore detect
    // the situation where RGBA are all 0 and treat that the same as
    // all 1 (full white).
    if( new_message->mesh_use_embedded_materials && r == 0 && g == 0 && b == 0 && a == 0 )
    {
      r = 1; g = 1; b = 1; a = 1;
    }

    Ogre::SceneBlendType blending;
    bool depth_write;

    if ( a < 0.9998 )
    {
      blending = Ogre::SBT_TRANSPARENT_ALPHA;
      depth_write = false;
    }
    else
    {
      blending = Ogre::SBT_REPLACE;
      depth_write = true;
    }

    S_MaterialPtr::iterator it;
    for( it = materials_.begin(); it != materials_.end(); it++ )
    {    
      Ogre::Technique* technique = (*it)->getTechnique( 0 );

      technique->setAmbient( r*0.5, g*0.5, b*0.5 );
      technique->setDiffuse( r, g, b, a );
      technique->setSceneBlending( blending );
      technique->setDepthWriteEnabled( depth_write );
    }
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  // In Ogre, mesh surface normals are not normalized if object is not
  // scaled.  This forces the surface normals to be renormalized by
  // invisibly tweaking the scale.
  if( scale.x == 1.0 && scale.y == 1.0 && scale.z == 1.0 )
  {
    scale.z = 1.0001;
  }
  scene_node_->setScale(scale);
}

S_MaterialPtr MeshResourceMarker::getMaterials()
{
  S_MaterialPtr materials;
  if( entity_ )
  {
    extractMaterials( entity_, materials );
  }
  return materials;
}

}

