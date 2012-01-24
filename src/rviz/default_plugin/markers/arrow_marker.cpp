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

#include "arrow_marker.h"
#include "marker_selection_handler.h"
#include "rviz/default_plugin/marker_display.h"

#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

namespace rviz
{

ArrowMarker::ArrowMarker(MarkerDisplay* owner, VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(owner, manager, parent_node)
, arrow_(0)
{
  child_scene_node_ = scene_node_->createChildSceneNode();
}

ArrowMarker::~ArrowMarker()
{
  delete arrow_;
  vis_manager_->getSceneManager()->destroySceneNode( child_scene_node_ );
}

void ArrowMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::ARROW);

  if (!new_message->points.empty() && new_message->points.size() < 2)
  {
    std::stringstream ss;
    ss << "Arrow marker [" << getStringID() << "] only specified one point of a point to point arrow.";
    if ( owner_ )
    {
      owner_->setMarkerStatus(getID(), status_levels::Error, ss.str());
    }
    ROS_DEBUG("%s", ss.str().c_str());

    delete arrow_;
    arrow_ = 0;

    return;
  }

  if (!arrow_)
  {
    arrow_ = new Arrow(vis_manager_->getSceneManager(), child_scene_node_);
    vis_manager_->getSelectionManager()->removeObject(coll_);
    coll_ = vis_manager_->getSelectionManager()->createCollisionForObject(arrow_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id))), coll_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);
  setPosition(pos);
  setOrientation( orient );

  arrow_->setColor(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);

  // compute translation & rotation from the two points
  if (new_message->points.size() == 2)
  {
    Ogre::Vector3 point1( new_message->points[0].x, new_message->points[0].y, new_message->points[0].z );
    Ogre::Vector3 point2( new_message->points[1].x, new_message->points[1].y, new_message->points[1].z );

    Ogre::Vector3 direction = point2 - point1;
    float distance = direction.length();

    float head_length = 0.1*distance;
    if ( new_message->scale.z != 0.0 )
    {
      head_length = new_message->scale.z;
    }
    float shaft_length = distance - head_length;

    arrow_->set(shaft_length, new_message->scale.x, head_length, new_message->scale.y);

    direction.normalise();

    // for some reason the arrow goes into the y direction by default
    Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );

    arrow_->setPosition(point1);
    arrow_->setOrientation( orient );
  }
  else
  {
    if ( owner_ && (new_message->scale.x * new_message->scale.y * new_message->scale.z == 0.0f) )
    {
      owner_->setMarkerStatus(getID(), status_levels::Warn, "Scale of 0 in one of x/y/z");
    }
    arrow_->setScale(scale);

    Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( Ogre::Vector3(1,0,0) );
    arrow_->setOrientation( orient );
  }
}

S_MaterialPtr ArrowMarker::getMaterials()
{
  S_MaterialPtr materials;
  extractMaterials( arrow_->getHead()->getEntity(), materials );
  extractMaterials( arrow_->getShaft()->getEntity(), materials );
  return materials;
}

}
