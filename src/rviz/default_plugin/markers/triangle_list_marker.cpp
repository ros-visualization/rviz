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
#include "rviz/uniform_string_stream.h"

#include "rviz/display_context.h"
#include "rviz/mesh_loader.h"
#include "marker_display.h"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

namespace rviz
{

TriangleListMarker::TriangleListMarker(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
: MarkerBase(owner, context, parent_node)
, manual_object_(0)
{
}

TriangleListMarker::~TriangleListMarker()
{
  context_->getSceneManager()->destroyManualObject(manual_object_);
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

void TriangleListMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::TRIANGLE_LIST);

  size_t num_points = new_message->points.size();
  if( (num_points % 3) != 0 || num_points == 0 )
  {
    std::stringstream ss;
    if( num_points == 0 )
    {
      ss << "TriMesh marker [" << getStringID() << "] has no points.";
    }
    else
    {
      ss << "TriMesh marker [" << getStringID() << "] has a point count which is not divisible by 3 [" << num_points <<"]";
    }
    if ( owner_ )
    {
      owner_->setMarkerStatus(getID(), StatusProperty::Error, ss.str());
    }
    ROS_DEBUG("%s", ss.str().c_str());

    scene_node_->setVisible( false );
    return;
  }
  else
  {
    scene_node_->setVisible( true );
  }

  if (!manual_object_)
  {
    static uint32_t count = 0;
    UniformStringStream ss;
    ss << "Triangle List Marker" << count++;
    manual_object_ = context_->getSceneManager()->createManualObject(ss.str());
    scene_node_->attachObject(manual_object_);

    ss << "Material";
    material_name_ = ss.str();
    material_ = Ogre::MaterialManager::getSingleton().create( material_name_, ROS_PACKAGE_NAME );
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setCullingMode(Ogre::CULL_NONE);

    handler_.reset( new MarkerSelectionHandler( this, MarkerID( new_message->ns, new_message->id ), context_ ));
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale))
  {    
    ROS_DEBUG("Unable to transform marker message");
    scene_node_->setVisible( false );
    return;
  }
  
  if ( owner_ &&  (new_message->scale.x * new_message->scale.y * new_message->scale.z == 0.0f) )
  {
    owner_->setMarkerStatus(getID(), StatusProperty::Warn, "Scale of 0 in one of x/y/z");
  }

  setPosition(pos);
  setOrientation(orient);
  scene_node_->setScale(scale);

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

  bool has_vertex_colors = new_message->colors.size() == num_points;
  bool has_face_colors = new_message->colors.size() == num_points / 3;
  bool any_vertex_has_alpha = false;

  if (has_vertex_colors)
  {
    for (size_t i = 0; i < num_points; ++i)
    {
      manual_object_->position(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z);
      any_vertex_has_alpha = any_vertex_has_alpha || (new_message->colors[i].a < 0.9998);
      manual_object_->colour(new_message->colors[i].r, new_message->colors[i].g, new_message->colors[i].b, new_message->color.a * new_message->colors[i].a);
    }
  }
  else if (has_face_colors)
  {
    for (size_t i = 0; i < num_points; ++i)
    {
      manual_object_->position(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z);
      any_vertex_has_alpha = any_vertex_has_alpha || (new_message->colors[i/3].a < 0.9998);
      manual_object_->colour(new_message->colors[i/3].r, new_message->colors[i/3].g, new_message->colors[i/3].b, new_message->color.a * new_message->colors[i/3].a);
    }
  }
  else
  {
    for (size_t i = 0; i < num_points; ++i)
    {
      manual_object_->position(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z);
    }
  }

  manual_object_->end();

  if (has_vertex_colors || has_face_colors)
  {
    material_->getTechnique(0)->setLightingEnabled(false);
  }
  else
  {
    material_->getTechnique(0)->setLightingEnabled(true);
    float r,g,b,a;
    r = new_message->color.r;
    g = new_message->color.g;
    b = new_message->color.b;
    a = new_message->color.a;
    material_->getTechnique(0)->setAmbient( r,g,b );
    material_->getTechnique(0)->setDiffuse( 0,0,0,a );
  }

  if( (!has_vertex_colors && new_message->color.a < 0.9998) || (has_vertex_colors && any_vertex_has_alpha))
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    material_->getTechnique(0)->setDepthWriteEnabled( true );
  }

  handler_->addTrackedObject( manual_object_ );
}

S_MaterialPtr TriangleListMarker::getMaterials()
{
  S_MaterialPtr materials;
  materials.insert( material_ );
  return materials;
}


}

