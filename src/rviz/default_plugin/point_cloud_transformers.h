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

#ifndef POINT_CLOUD_TRANSFORMERS_H
#define POINT_CLOUD_TRANSFORMERS_H

#include <sensor_msgs/PointCloud2.h>

#include "point_cloud_transformer.h"

namespace rviz
{

class BoolProperty;
class ColorProperty;
class EditableEnumProperty;
class EnumProperty;
class FloatProperty;

typedef std::vector<std::string> V_string; 

inline int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& channel)
{
  for (size_t i = 0; i < cloud->fields.size(); ++i)
  {
    if (cloud->fields[i].name == channel)
    {
      return i;
    }
  }

  return -1;
}

template<typename T>
inline T valueFromCloud(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t offset, uint8_t type, uint32_t point_step, uint32_t index)
{
  const uint8_t* data = &cloud->data[(point_step * index) + offset];
  T ret = 0;

  switch (type)
  {
  case sensor_msgs::PointField::INT8:
  case sensor_msgs::PointField::UINT8:
    {
      uint8_t val = *reinterpret_cast<const uint8_t*>(data);
      ret = static_cast<T>(val);
      break;
    }

  case sensor_msgs::PointField::INT16:
  case sensor_msgs::PointField::UINT16:
    {
      uint16_t val = *reinterpret_cast<const uint16_t*>(data);
      ret = static_cast<T>(val);
      break;
    }

  case sensor_msgs::PointField::INT32:
  case sensor_msgs::PointField::UINT32:
    {
      uint32_t val = *reinterpret_cast<const uint32_t*>(data);
      ret = static_cast<T>(val);
      break;
    }

  case sensor_msgs::PointField::FLOAT32:
    {
      float val = *reinterpret_cast<const float*>(data);
      ret = static_cast<T>(val);
      break;
    }

  case sensor_msgs::PointField::FLOAT64:
    {
      double val = *reinterpret_cast<const double*>(data);
      ret = static_cast<T>(val);
      break;
    }
  default:
    break;
  }

  return ret;
}

class IntensityPCTransformer : public PointCloudTransformer
{
Q_OBJECT
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                         uint32_t mask,
                         const Ogre::Matrix4& transform,
                         V_PointCloudPoint& points_out);
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual void createProperties( Property* parent_property, uint32_t mask, QList<Property*>& out_props );
  void updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud); 

private Q_SLOTS:
  void updateUseRainbow();
  void updateAutoComputeIntensityBounds();

private:
  V_string available_channels_;

  ColorProperty* min_color_property_;
  ColorProperty* max_color_property_;
  BoolProperty* auto_compute_intensity_bounds_property_;
  BoolProperty* use_rainbow_property_;
  BoolProperty* invert_rainbow_property_;
  FloatProperty* min_intensity_property_;
  FloatProperty* max_intensity_property_;
  EditableEnumProperty* channel_name_property_;
};

class XYZPCTransformer : public PointCloudTransformer
{
Q_OBJECT
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class RGB8PCTransformer : public PointCloudTransformer
{
Q_OBJECT
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class MONO8PCTransformer : public RGB8PCTransformer
{
Q_OBJECT
public:
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class RGBF32PCTransformer : public PointCloudTransformer
{
Q_OBJECT
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class FlatColorPCTransformer : public PointCloudTransformer
{
Q_OBJECT
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
  virtual void createProperties( Property* parent_property, uint32_t mask, QList<Property*>& out_props );
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);

private:
  ColorProperty* color_property_;
};

class AxisColorPCTransformer : public PointCloudTransformer
{
Q_OBJECT
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
  virtual void createProperties( Property* parent_property, uint32_t mask, QList<Property*>& out_props );
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);

  enum Axis
  {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
  };

private Q_SLOTS:
  void updateAutoComputeBounds();

private:
  BoolProperty* auto_compute_bounds_property_;
  FloatProperty* min_value_property_;
  FloatProperty* max_value_property_;
  EnumProperty* axis_property_;
  BoolProperty* use_fixed_frame_property_;
};

}

#endif // POINT_CLOUD_TRANSFORMERS_H
