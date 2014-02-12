/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <OgreRay.h>
#include <OgrePlane.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "geometry.h"

namespace rviz
{

/** Given a viewport and an x,y position in window-pixel coordinates,
 *  find the point on a plane directly behind it, if any.
 * @return true if the intersection exists, false if it does not. */
bool getPointOnPlaneFromWindowXY( Ogre::Viewport* viewport,
                                  Ogre::Plane& plane,
                                  int window_x, int window_y,
                                  Ogre::Vector3& intersection_out )
{
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay( (float)window_x / (float)width,
                                                                       (float)window_y / (float)height );
  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects( plane );
  if ( !intersection.first )
  {
    return false;
  }
  intersection_out = mouse_ray.getPoint( intersection.second );

  return true;
}

float mapAngleTo0_2Pi( float angle )
{
  angle = fmod( angle, Ogre::Math::TWO_PI );

  if( angle < 0.0f )
  {
    angle = Ogre::Math::TWO_PI + angle;
  }
  return angle;
}

Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view, const Ogre::Vector3& pos)
{
  Ogre::Camera* cam = view->getCamera();
  Ogre::Vector3 pos2D = cam->getProjectionMatrix() * (cam->getViewMatrix() * pos);

  Ogre::Real x = ((pos2D.x * 0.5) + 0.5);
  Ogre::Real y = 1 - ((pos2D.y * 0.5) + 0.5);

  return  Ogre::Vector2(x * view->getActualWidth(), y * view->getActualHeight());
}

} // end namespace rviz
