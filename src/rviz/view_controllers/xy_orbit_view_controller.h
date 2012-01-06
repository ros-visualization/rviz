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

#ifndef RVIZ_SIMPLE_ORBIT_VIEW_CONTROLLER_H
#define RVIZ_SIMPLE_ORBIT_VIEW_CONTROLLER_H

#include "rviz/view_controllers/orbit_view_controller.h"

#include <OGRE/OgreVector3.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

/**
 * \brief Like the orbit view controller, but focal point moves only in the x-y plane.
 */
class XYOrbitViewController : public OrbitViewController
{
public:
  XYOrbitViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node);

  virtual void handleMouseEvent(ViewportMouseEvent& evt);

  static std::string getClassNameStatic() { return "rviz::XYOrbitViewController"; }
  virtual std::string getClassName() { return getClassNameStatic(); }
  virtual void lookAt( const Ogre::Vector3& point );

protected:
  virtual void onActivate();
  virtual void updateCamera();

  bool intersectGroundPlane( Ogre::Ray mouse_ray, Ogre::Vector3 &intersection_3d );
};

}

#endif // RVIZ_VIEW_CONTROLLER_H
