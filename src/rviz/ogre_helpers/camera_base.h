/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef OGRE_TOOLS_CAMERA_BASE_H_
#define OGRE_TOOLS_CAMERA_BASE_H_

#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace Ogre
{
  class Camera;
  class SceneNode;
  class SceneManager;
}

namespace rviz
{

/**
 * \class CameraBase
 * \brief Generic interface for a camera
 *
 * Provides a interface that a camera can override, providing interchangeability between different camera types.
 * Specific implementation is left to the child class.
 */
class CameraBase
{
public:
  /**
   * \brief Constructor
   * @param scene_manager Scene manager this camera is displaying
   */
  CameraBase( Ogre::SceneManager* scene_manager );
  virtual ~CameraBase();

  /**
   * \brief Get the Ogre camera associated with this camera object
   * @return The Ogre camera associated with this camera object
   */
  Ogre::Camera* getOgreCamera() { return camera_; }

  /**
   * \brief Set a scene node that all camera transformations should be relative to
   * @param node The node
   */
  void setRelativeNode( Ogre::SceneNode* node );
  /**
   * \brief Called when the relative node changes
   */
  virtual void relativeNodeChanged() {}

  virtual void update() = 0;

  /**
   * \brief Set the position of the camera
   */
  virtual void setPosition( const Ogre::Vector3& position );
  /**
   * \brief Set the orientation of the camera
   */
  virtual void setOrientation( const Ogre::Quaternion& orientation );

  /**
   * \brief Yaw the camera.
   *
   * Calls to yaw are cumulative, so:
   *   yaw(PI);
   *   yaw(PI);
   *
   * is equivalent to
   *  yaw(2*PI);
   *
   * @param angle Angle to yaw, in radians
   */
  virtual void yaw( float angle ) = 0;
  /**
     * \brief Pitch the camera.
     *
     * Calls to pitch are cumulative, so:
     *   pitch(PI);
     *   pitch(PI);
     *
     * is equivalent to
     *  pitch(2*PI);
     *
     * @param angle Angle to pitch, in radians
     */
  virtual void pitch( float angle ) = 0;
  /**
       * \brief Roll the camera.
       *
       * Calls to roll are cumulative, so:
       *   roll(PI);
       *   roll(PI);
       *
       * is equivalent to
       *  roll(2*PI);
       *
       * @param angle Angle to roll, in radians
       */
  virtual void roll( float angle ) = 0;

  /**
   * \brief Set the orientation of the camera from a quaternion
   */
  virtual void setOrientation( float x, float y, float z, float w ) = 0;
  /**
   * \brief Set the position of the camera
   */
  virtual void setPosition( float x, float y, float z ) = 0;

  /**
   * \brief Set the position/orientation of this camera from another camera.
   *
   * @param camera The camera to set from
   */
  virtual void setFrom( CameraBase* camera ) = 0;

  /**
   * \brief Get the position of this camera
   * @return The position of this camera
   */
  virtual Ogre::Vector3 getPosition() = 0;
  /**
   * \brief Get the orientation of this camera
   * @return The orientation of this camera
   */
  virtual Ogre::Quaternion getOrientation() = 0;

  /**
   * \brief Point the camera at the specified point
   * @param point The point to look at
   */
  virtual void lookAt( const Ogre::Vector3& point ) = 0;

  /**
   * \brief Move the camera relative to its orientation
   *
   * @param x Distance to move along the X-axis
   * @param y Distance to move along the Y-axis
   * @param z Distance to move along the Z-axis
   */
  virtual void move( float x, float y, float z ) = 0;

  virtual void mouseLeftDown( int x, int y ) {}
  virtual void mouseMiddleDown( int x, int y ) {}
  virtual void mouseRightDown( int x, int y ) {}
  virtual void mouseLeftUp( int x, int y ) {}
  virtual void mouseMiddleUp( int x, int y ) {}
  virtual void mouseRightUp( int x, int y ) {}

  /**
   * \brief Handle a left mouse button drag
   *
   * @param diff_x Pixels the mouse has moved in the (window space) x direction
   * @param diff_y Pixels the mouse has moved in the (window space) y direction
   */
  virtual void mouseLeftDrag( int diff_x, int diff_y, bool ctrl, bool alt, bool shift ) = 0;
  /**
   * \brief Handle a middle mouse button drag
   *
   * @param diff_x Pixels the mouse has moved in the (window space) x direction
   * @param diff_y Pixels the mouse has moved in the (window space) y direction
   */
  virtual void mouseMiddleDrag( int diff_x, int diff_y, bool ctrl, bool alt, bool shift ) = 0;
  /**
   * \brief Handle a right mouse button drag
   *
   * @param diff_x Pixels the mouse has moved in the (window space) x direction
   * @param diff_y Pixels the mouse has moved in the (window space) y direction
   */
  virtual void mouseRightDrag( int diff_x, int diff_y, bool ctrl, bool alt, bool shift ) = 0;
  /**
   * \brief Handle a scrollwheel change
   *
   * @param diff Number of "units" the scrollwheel has moved
   * @todo Probably need to pass in how many units there are in a "click" of the wheel
   */
  virtual void scrollWheel( int diff, bool ctrl, bool alt, bool shift ) = 0;

  /**
   * \brief Loads the camera's configure from the supplied string (generated through toString())
   * @param str The string to load from
   */
  virtual void fromString(const std::string& str) = 0;
  /**
   * \brief Returns a string representation of the camera's configuration
   */
  virtual std::string toString() = 0;

protected:
  Ogre::Camera* camera_;                  ///< Ogre camera associated with this camera object
  Ogre::SceneManager* scene_manager_;     ///< Scene manager this camera is part of

  Ogre::SceneNode* relative_node_;
};

} // namespace rviz

#endif /*OGRE_TOOLS_CAMERA_BASE_H_*/
