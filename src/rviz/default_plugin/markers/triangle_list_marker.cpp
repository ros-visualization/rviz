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

#include "triangle_list_marker.h"

#include "marker_selection_handler.h"
#include "rviz/default_plugin/marker_display.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/visualization_manager.h"
#include "rviz/mesh_loader.h"
#include "marker_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

namespace rviz
{

TriangleListMarker::TriangleListMarker(MarkerDisplay* owner, VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(owner, manager, parent_node)
, manual_object_(0)
{
  if (parent_node)
  {
    scene_node_ = parent_node->createChildSceneNode();
  }
  else
  {
    scene_node_ = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  }
}

TriangleListMarker::~TriangleListMarker()
{
  vis_manager_->getSceneManager()->destroySceneNode(scene_node_->getName());
  vis_manager_->getSceneManager()->destroyManualObject(manual_object_);

  for (size_t i = 0; i < material_->getNumTechniques(); ++i)
  {
    Ogre::Technique* t = material_->getTechnique(i);
    // hack hack hack, really need to do a shader-based way of picking, rather than
    // creating a texture for each object
    if (t->getSchemeName() == "Pick")
    {
      Ogre::TextureManager::getSingleton().remove(t->getPass(0)->getTextureUnitState(0)->getTextureName());
    }
  }

  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

void TriangleListMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::TRIANGLE_LIST);

  scene_node_->setVisible(false);

  if (!manual_object_)
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "Mesh Marker" << count++;
    manual_object_ = vis_manager_->getSceneManager()->createManualObject(ss.str());
    scene_node_->attachObject(manual_object_);

    ss << "Material";
    material_name_ = ss.str();
    material_ = Ogre::MaterialManager::getSingleton().create( material_name_, ROS_PACKAGE_NAME );
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(false);
    material_->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
    material_->setCullingMode(Ogre::CULL_NONE);

    SelectionManager* sel_man = vis_manager_->getSelectionManager();
    coll_ = sel_man->createHandle();
    sel_man->addPickTechnique(coll_, material_);
  }

  size_t num_points = new_message->points.size();
  if ((num_points % 3) != 0)
  {
    std::stringstream ss;
    ss << "TriMesh marker [" << getStringID() << "] has a point count which is not divisible by 3 [" << num_points <<"]";
    owner_->setMarkerStatus(getID(), status_levels::Error, ss.str());
    ROS_DEBUG("%s", ss.str().c_str());

    manual_object_->clear();
    return;
  }

  bool has_vertex_colors = new_message->colors.size() == num_points;
  if (has_vertex_colors)
  {
    material_->getTechnique(0)->setLightingEnabled(false);
  }
  else
  {
    material_->getTechnique(0)->setLightingEnabled(true);
  }

  // If we have the same number of tris as previously, just update the object
  if (old_message && num_points == old_message->points.size())
  {
    manual_object_->beginUpdate(0);
  }
  else // Otherwise clear it and begin anew
  {
    manual_object_->clear();
    manual_object_->estimateVertexCount(num_points);
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  }

  for (size_t i = 0; i < num_points; ++i)
  {
    manual_object_->position(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z);

    if (has_vertex_colors)
    {
      manual_object_->colour(new_message->colors[i].r, new_message->colors[i].g, new_message->colors[i].b);
    }
  }

  manual_object_->end();

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale, false);

  scene_node_->setVisible(true);
  scene_node_->setPosition(pos);
  scene_node_->setOrientation(orient);
  scene_node_->setScale(scale);

  float r = new_message->color.r;
  float g = new_message->color.g;
  float b = new_message->color.b;
  float a = new_message->color.a;
  material_->getTechnique(0)->setAmbient( r*0.5, g*0.5, b*0.5 );
  material_->getTechnique(0)->setDiffuse( r, g, b, a );

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
}

}

