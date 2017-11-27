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
#include <tf/LinearMath/Quaternion.h>

#include <boost/array.hpp>

namespace rviz
{

inline bool validateQuaternions( float w, float x, float y, float z )
{
  return std::abs( w * w + x * x + y * y + z * z - 1.0f ) < 10e-3f;
}

inline bool validateQuaternions( double w, double x, double y, double z )
{
  return std::abs( w * w + x * x + y * y + z * z - 1.0 ) < 10e-3;
}

inline bool validateQuaternions( Ogre::Quaternion quaternion )
{
  return validateQuaternions( quaternion.w, quaternion.x, quaternion.y, quaternion.z );
}

inline bool validateQuaternions( tf::Quaternion quaternion )
{
  return validateQuaternions( quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

inline bool validateQuaternions( const geometry_msgs::Quaternion &msg )
{
  return validateQuaternions( msg.w, msg.x, msg.y, msg.z );
}

inline bool validateQuaternions( const geometry_msgs::Pose &msg )
{
  return validateQuaternions( msg.orientation );
}

inline bool validateQuaternions( const geometry_msgs::PoseStamped &msg )
{
  return validateQuaternions( msg.pose );
}

template<typename T>
inline bool validateQuaternions( const std::vector<T> &vec )
{
  typedef std::vector<T> VecType;
  typename VecType::const_iterator it = vec.begin();
  typename VecType::const_iterator end = vec.end();
  for ( ; it != end; ++it )
  {
    if ( !validateQuaternions( *it ))
    {
      return false;
    }
  }

  return true;
}

template<typename T, size_t N>
inline bool validateQuaternions( const boost::array<T, N> &arr )
{
  typedef boost::array<T, N> ArrType;
  typename ArrType::const_iterator it = arr.begin();
  typename ArrType::const_iterator end = arr.end();
  for ( ; it != end; ++it )
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
