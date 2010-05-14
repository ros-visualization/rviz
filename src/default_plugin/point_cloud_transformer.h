/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef RVIZ_POINT_CLOUD_TRANSFORMER_H
#define RVIZ_POINT_CLOUD_TRANSFORMER_H

#include <string>

#include <ros/message_forward.h>
#include <rviz/properties/forwards.h>

namespace Ogre
{
class Matrix4;
}

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(PointCloud2);
}

namespace rviz
{

struct PointCloudPoint
{
  Ogre::Vector3 position;
  Ogre::ColourValue color;
};
typedef std::vector<PointCloudPoint> V_PointCloudPoint;

struct PointCloud
{
  V_PointCloudPoint points;
};

class PointCloudTransformer
{
public:
  enum SupportLevel
  {
    Support_None = 0,
    Support_XYZ = 1 << 1,
    Support_Color = 1 << 2,
    Support_Both = Support_XYZ|Support_Color,
  };
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) = 0;
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out) = 0;
  virtual const std::string& getName() = 0;

  virtual void reset() {}
  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBase& out_props) {}
};

} // namespace rviz

#endif
