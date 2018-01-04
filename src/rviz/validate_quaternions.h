/*
 * Copyright (c) 2017, Stefan Fabian
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

#ifndef RVIZ_VALIDATE_QUATERNIONS_H
#define RVIZ_VALIDATE_QUATERNIONS_H

#include <geometry_msgs/PoseStamped.h>
#include <OgreQuaternion.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>

#include <boost/array.hpp>

namespace rviz
{

inline float quaternionNorm2(float w, float x, float y, float z)
{
  return w * w + x * x + y * y + z * z;
}

inline bool validateQuaternion(float w, float x, float y, float z)
{
  if ( 0.0f == x && 0.0f == y && 0.0f == z && 0.0f == w )
  {
    // Allow null quaternions to pass because they are common in uninitialized ROS messages.
    return true;
  }
  float norm2 = quaternionNorm2(w, x, y, z); 
  // use same threshold as in tf
  bool is_normalized = std::abs( norm2 - 1.0f ) < 10e-3f;
  ROS_DEBUG_COND_NAMED( !is_normalized, "quaternions", "Quaternion [x: %.3f, y: %.3f, z: %.3f, w: %.3f] not normalized. "
                        "Magnitude: %.3f", x, y, z, w, std::sqrt(norm2) );
  return is_normalized;
}

inline bool normalizeQuaternion(float& w, float& x, float& y, float& z)
{
  float norm2 = quaternionNorm2(w, x, y, z);
  if (norm2 < 10e-3f)
    return false;
  norm2 = std::sqrt(norm2);
  w /= norm2;
  x /= norm2;
  y /= norm2;
  z /= norm2;
  return true;
}


inline double quaternionNorm2(double w, double x, double y, double z)
{
  return w * w + x * x + y * y + z * z;
}

inline bool validateQuaternion(double w, double x, double y, double z)
{
  if ( 0.0 == x && 0.0 == y && 0.0 == z && 0.0 == w )
  {
    // Allow null quaternions to pass because they are common in uninitialized ROS messages.
    return true;
  }
  double norm2 = quaternionNorm2(w, x, y, z); 
  // use same threshold as in tf
  bool is_normalized = std::abs( norm2 - 1.0 ) < 10e-3;
  ROS_DEBUG_COND_NAMED( !is_normalized, "quaternions", "Quaternion [x: %.3f, y: %.3f, z: %.3f, w: %.3f] not normalized. "
                        "Magnitude: %.3f", x, y, z, w, std::sqrt(norm2) );
  return is_normalized;
}

inline bool normalizeQuaternion(double& w, double& x, double& y, double& z)
{
  float norm2 = quaternionNorm2(w, x, y, z);
  if (norm2 < 10e-3f)
    return false;
  norm2 = std::sqrt(norm2);
  w /= norm2;
  x /= norm2;
  y /= norm2;
  z /= norm2;
  return true;
}


inline bool validateQuaternion(const Ogre::Quaternion& quaternion)
{
  return validateQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

inline bool validateQuaternion(const tf::Quaternion& quaternion)
{
  return validateQuaternion(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

inline bool validateQuaternion(const geometry_msgs::Quaternion &msg)
{
  return validateQuaternion(msg.w, msg.x, msg.y, msg.z);
}

inline bool validateQuaternion(const geometry_msgs::Pose &msg)
{
  return validateQuaternion(msg.orientation);
}

inline bool validateQuaternion(const geometry_msgs::PoseStamped &msg)
{
  return validateQuaternion(msg.pose.orientation);
}


inline bool normalizeQuaternion(Ogre::Quaternion& quaternion)
{
  if (normalizeQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z))
    return true;

  quaternion.w = 1.0;
  quaternion.x = quaternion.y = quaternion.z = 0.0;
  return false;
}

inline bool normalizeQuaternion(const geometry_msgs::Quaternion &msg, Ogre::Quaternion &q)
{
  q.w = msg.w;
  q.x = msg.x;
  q.y = msg.y;
  q.z = msg.z;
  return normalizeQuaternion(q);
}


template<typename T>
inline bool validateQuaternions(const T &vec)
{
  for (typename T::const_iterator it = vec.begin(), end = vec.end() ; it != end; ++it)
  {
    if ( !validateQuaternions( *it ))
    {
      return false;
    }
  }

  return true;
}

} // namespace rviz

#endif // RVIZ_VALIDATE_QUATERNIONS_H
