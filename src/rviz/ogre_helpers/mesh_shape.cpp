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

#include "mesh_shape.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreManualObject.h>

#include <ros/console.h>
#include <boost/lexical_cast.hpp>

namespace rviz
{

MeshShape::MeshShape(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
: Shape( Shape::Mesh, scene_manager, parent_node )
, started_(false)
{
  static uint32_t count = 0;
  manual_object_ = scene_manager->createManualObject("MeshShape_ManualObject" + boost::lexical_cast<std::string>(count++));
}

MeshShape::~MeshShape()
{
  // destroy the entity first
  if (entity_)
  {
    scene_manager_->destroyEntity( entity_ );
    entity_ = NULL;
  }
  scene_manager_->destroyManualObject(manual_object_);
}

void MeshShape::estimateVertexCount(size_t vcount)
{
  if (entity_ == NULL && started_ == false)
    manual_object_->estimateVertexCount(vcount);
}

void MeshShape::beginTriangles()
{
  if (!started_ && entity_)
  {
    ROS_WARN("Cannot modify mesh once construction is complete");
    return;
  }
  
  if (!started_)
  {
    started_ = true;
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  }
}

void MeshShape::addVertex(const Ogre::Vector3& position)
{
  beginTriangles();
  manual_object_->position(position);
}

void MeshShape::addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal)
{
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
}

void MeshShape::addTriangle(const Ogre::Vector3 &p1, const Ogre::Vector3 &p2, const Ogre::Vector3 &p3)
{
  Ogre::Vector3 n = (p1-p2).crossProduct(p2-p3);
  n.normalise();
  addTriangle(p1, p2, p3, n);
}

void MeshShape::addTriangle(const Ogre::Vector3 &p1, const Ogre::Vector3 &p2, const Ogre::Vector3 &p3, const Ogre::Vector3 &normal)
{
  addVertex(p1, normal);
  addVertex(p2, normal);
  addVertex(p3, normal);  
}

void MeshShape::addTriangle(const Ogre::Vector3 &p1, const Ogre::Vector3 &p2, const Ogre::Vector3 &p3,
                            const Ogre::Vector3 &n1, const Ogre::Vector3 &n2, const Ogre::Vector3 &n3)
{
  addVertex(p1, n1);
  addVertex(p2, n2);
  addVertex(p3, n3);
}

void MeshShape::endTriangles()
{
  if (started_)
  {
    started_ = false;
    manual_object_->end();
    std::string name = "ConvertedMeshShape@" + boost::lexical_cast<std::string>(this);
    manual_object_->convertToMesh(name);
    entity_ = scene_manager_->createEntity(name);
    if (entity_)
    {
      entity_->setMaterialName(material_name_);
      offset_node_->attachObject(entity_);
    }
    else
      ROS_ERROR("Unable to construct triangle mesh");
  }
  else
    ROS_ERROR("No triangles added");
}

} // namespace rviz

