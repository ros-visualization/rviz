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

#ifndef GEOMETRY_H
#define GEOMETRY_H

namespace Ogre
{
class Plane;
class Vector3;
class Vector2;
}

namespace rviz
{

/** @brief Given a viewport and an x,y position in window-pixel coordinates,
 *  find the point on a plane directly behind it, if any.
 * @return true if the intersection exists, false if it does not. */
bool getPointOnPlaneFromWindowXY( Ogre::Viewport* viewport,
                                  Ogre::Plane& plane,
                                  int window_x, int window_y,
                                  Ogre::Vector3& intersection_out );


/** @brief Return the input angle mapped back to the range 0 to 2*PI. */
float mapAngleTo0_2Pi( float angle );

/** @brief Given a viewport and a 3D position in world coordinates,
 *  project that point into the view plane.
 * @return The 2D floating-point pixel position of the projection. */
Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view, const Ogre::Vector3& pos);

} // end namespace rviz

#endif
