/*
 * Copyright (c) 2019, Bielefeld University
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
 *     * Neither the name of Bielefeld University nor the names of its
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

#include "frame_view_controller.h"
#include <rviz/display_context.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>

namespace rviz
{

void FrameViewController::reset()
{
  camera_->setPosition(Ogre::Vector3(0, 0, 0));
  camera_->lookAt(0, 0, -1);
  setPropertiesFromCamera(camera_);
}

void FrameViewController::onTargetFrameChanged(const Ogre::Vector3& /* old_reference_position */,
                                               const Ogre::Quaternion& /* old_reference_orientation */)
{
  // don't adapt the camera pose to the old reference position, but just jump to new frame
}

void FrameViewController::updateTargetSceneNode()
{
  if ( getNewTransform() )
  {
    // track both, position and orientation of reference frame
    target_scene_node_->setPosition( reference_position_ );
    target_scene_node_->setOrientation( reference_orientation_ );
    context_->queueRender();
  }
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::FrameViewController, rviz::ViewController )
