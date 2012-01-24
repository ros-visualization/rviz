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

#include "shape_marker.h"
#include "marker_selection_handler.h"
#include "rviz/default_plugin/marker_display.h"

#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"

#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreMatrix3.h>

namespace rviz
{

ShapeMarker::ShapeMarker( MarkerDisplay* owner, VisualizationManager* manager,
    Ogre::SceneNode* parent_node ) :
  MarkerBase(owner, manager, parent_node), shape_(0)
{
}

ShapeMarker::~ShapeMarker()
{
  delete shape_;
}

void ShapeMarker::onNewMessage( const MarkerConstPtr& old_message,
    const MarkerConstPtr& new_message )
{
  if (!shape_ || old_message->type != new_message->type)
  {
    delete shape_;
    shape_ = 0;

    switch (new_message->type)
    {
      case visualization_msgs::Marker::CUBE:
      {
        shape_ = new Shape(Shape::Cube,
            vis_manager_->getSceneManager(), scene_node_);
      }
        break;

      case visualization_msgs::Marker::CYLINDER:
      {
        shape_ = new Shape(Shape::Cylinder,
            vis_manager_->getSceneManager(), scene_node_);
      }
        break;

      case visualization_msgs::Marker::SPHERE:
      {
        shape_ = new Shape(Shape::Sphere,
            vis_manager_->getSceneManager(), scene_node_);
      }
        break;

      default:
        ROS_BREAK();
        break;
    }

    vis_manager_->getSelectionManager()->removeObject(coll_);
    coll_ = vis_manager_->getSelectionManager()->createCollisionForObject(
        shape_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(
            new_message->ns, new_message->id))), coll_);
  }

  Ogre::Vector3 pos, scale, scale_correct;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  if (owner_ && (new_message->scale.x * new_message->scale.y
      * new_message->scale.z == 0.0f))
  {
    owner_->setMarkerStatus(getID(), status_levels::Warn,
        "Scale of 0 in one of x/y/z");
  }

  setPosition(pos);
  setOrientation( orient * Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ) );

  scale_correct = Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ) * scale;

  shape_->setScale(scale_correct);

  shape_->setColor(new_message->color.r, new_message->color.g,
      new_message->color.b, new_message->color.a);
}

S_MaterialPtr ShapeMarker::getMaterials()
{
  S_MaterialPtr materials;
  extractMaterials(shape_->getEntity(), materials);
  return materials;
}

}
