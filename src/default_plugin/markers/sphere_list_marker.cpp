/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "sphere_list_marker.h"
#include "marker_selection_handler.h"
#include "rviz/common.h"

#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"

#include <ogre_tools/shape.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreStaticGeometry.h>
#include <OGRE/OgreEntity.h>

namespace rviz
{

SphereListMarker::SphereListMarker(VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(manager, parent_node)
, geometry_(0)
{
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "SphereListMarker" << count++;

  geometry_ = vis_manager_->getSceneManager()->createStaticGeometry(ss.str());

  ss << "Material";
  material_name_ = ss.str();
  material_ = Ogre::MaterialManager::getSingleton().create( material_name_, ROS_PACKAGE_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );


}

SphereListMarker::~SphereListMarker()
{
  vis_manager_->getSceneManager()->destroyStaticGeometry(geometry_);
}

void SphereListMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::SPHERE_LIST);

  if (!old_message)
  {
    SelectionManager* sel_manager = vis_manager_->getSelectionManager();
    coll_ = sel_manager->createHandle();
    sel_manager->addPickTechnique(coll_, material_);
    sel_manager->addObject(coll_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id))));
  }

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

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  Ogre::SceneManager* scene_manager = vis_manager_->getSceneManager();
  Ogre::Entity* entity = scene_manager->createEntity("SphereListMarker Temp", "ogre_tools_sphere.mesh");

  entity->setMaterialName(material_name_);
  geometry_->reset();

  tf::TransformListener* tf_client = vis_manager_->getTFClient();
  std::string fixed_frame = vis_manager_->getFixedFrame();

  tf::Stamped<btTransform> transform;
  try
  {
    tf_client->lookupTransform (fixed_frame, new_message->header.frame_id, new_message->header.stamp, transform);
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Error looking up transform for marker [%s/%d]: %s", new_message->ns.c_str(), new_message->id, e.what());
  }

  std::vector<geometry_msgs::Point>::const_iterator it = new_message->points.begin();
  std::vector<geometry_msgs::Point>::const_iterator end = new_message->points.end();
  for ( ; it != end; ++it )
  {
    const geometry_msgs::Point& p = *it;

    Ogre::Vector3 point(p.x, p.y, p.z);
    robotToOgre(point);

    point = pos + (orient * point);

    geometry_->addEntity(entity, point, orient, scale);
  }

  geometry_->build();

  scene_manager->destroyEntity(entity);
}

}
