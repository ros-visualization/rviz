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

#include "tf_link_updater.h"
#include "common.h"

#include <tf/tf.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz
{

TFLinkUpdater::TFLinkUpdater(tf::Transformer* tf, const std::string& target_frame_)
: tf_(tf)
, target_frame_(target_frame_)
{
}

bool TFLinkUpdater::getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                                      Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation, bool& apply_offset_transforms) const
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), link_name );

  if (tf_->canTransform(target_frame_, link_name, ros::Time()))
  {
    try
    {
      tf_->transformPose( target_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'\n", link_name.c_str(), target_frame_.c_str() );
      return false;
    }
  }
  else
  {
    return false;
  }

  Ogre::Vector3 position( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX( yaw, pitch, roll );

  Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( yaw, pitch, roll ) );

  // Collision/visual transforms are the same in this case
  visual_position = position;
  visual_orientation = orientation;
  collision_position = position;
  collision_orientation = orientation;
  apply_offset_transforms = true;

  return true;
}

}
