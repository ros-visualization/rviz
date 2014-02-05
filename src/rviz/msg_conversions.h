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
#ifndef MSG_CONVERSIONS_H
#define MSG_CONVERSIONS_H

#include "OgreVector3.h"
#include "OgreQuaternion.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace rviz {

// This file contains some convenience functions for Ogre / geometry_msgs conversions.

// pointMsgToOgre  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
static inline Ogre::Vector3 pointMsgToOgre(const geometry_msgs::Point& m)
{
  return Ogre::Vector3(m.x, m.y, m.z);
}

static inline void pointMsgToOgre(const geometry_msgs::Point &m, Ogre::Vector3& o)
{
  o.x = m.x; o.y = m.y; o.z = m.z;
}

// vector3MsgToOgre  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
static inline Ogre::Vector3 vector3MsgToOgre(const geometry_msgs::Vector3& m)
{
  return Ogre::Vector3(m.x, m.y, m.z);
}


static inline void vector3MsgToOgre(const geometry_msgs::Vector3 &m, Ogre::Vector3& o)
{
  o.x = m.x; o.y = m.y; o.z = m.z;
}

// quaternionMsgToOgre  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
static inline Ogre::Quaternion quaternionMsgToOgre(const geometry_msgs::Quaternion& m)
{
  return Ogre::Quaternion(m.w, m.x, m.y, m.z);
}

static inline void quaternionMsgToOgre(const geometry_msgs::Quaternion &m, Ogre::Quaternion& o)
{
  o.w = m.w; o.x = m.x; o.y = m.y; o.z = m.z;
}

// pointOgreToMsg  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
static inline void pointOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Point &m)
{
  m.x = o.x; m.y = o.y; m.z = o.z;
}

static inline geometry_msgs::Point pointOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Point m;
  pointOgreToMsg(o, m);
  return m;
}

// vectorOgreToMsg  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
static inline void vector3OgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Vector3 &m)
{
  m.x = o.x; m.y = o.y; m.z = o.z;
}

static inline geometry_msgs::Vector3 vector3OgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Vector3 m;
  vector3OgreToMsg(o, m);
  return m;
}

// quaternionOgreToMsg  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
static inline void quaternionOgreToMsg(const Ogre::Quaternion &o, geometry_msgs::Quaternion &m)
{
  m.w = o.w; m.x = o.x; m.y = o.y; m.z = o.z;
}

static inline geometry_msgs::Quaternion quaternionOgreToMsg(const Ogre::Quaternion &o)
{
  geometry_msgs::Quaternion m;
  quaternionOgreToMsg(o, m);
  return m;
}


} // namespace rviz

#endif
