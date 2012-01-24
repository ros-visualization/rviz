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

#include "point_cloud_transformer.h"
#include <rviz/helpers/color.h>
#include <sensor_msgs/PointCloud2.h>

namespace rviz
{

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
public:
  IntensityPCTransformer()
  : min_color_( 0.0f, 0.0f, 0.0f )
  , max_color_( 1.0f, 1.0f, 1.0f )
  , min_intensity_(0.0f)
  , max_intensity_(4096.0f)
  , use_full_rgb_colors_(false) 
  , selected_channel_("intensity") 
  {
    setAutoComputeIntensityBounds(true);
  }

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                         uint32_t mask,
                         const Ogre::Matrix4& transform,
                         V_PointCloudPoint& points_out);
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual void reset();
  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props);

  void setMinColor( const Color& color );
  void setMaxColor( const Color& color );
  const Color& getMaxColor() { return max_color_; }
  const Color& getMinColor() { return min_color_; }
  void setMinIntensity(float val);
  void setMaxIntensity(float val);
  float getMinIntensity() { return min_intensity_; }
  float getMaxIntensity() { return max_intensity_; }
  void setAutoComputeIntensityBounds(bool compute);
  bool getAutoComputeIntensityBounds() { return auto_compute_intensity_bounds_; }
  void setUseFullRGBColors(bool full_rgb); 
  bool getUseFullRGBColors() { return use_full_rgb_colors_; } 
  const std::string& getChannelName() { return selected_channel_; } 
  void setChannelName(const std::string& channel); 
  void updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud); 

private:
  Color min_color_;
  Color max_color_;
  float min_intensity_;
  float max_intensity_;
  bool auto_compute_intensity_bounds_;
  bool use_full_rgb_colors_;
  bool intensity_bounds_changed_;
  std::string selected_channel_;
  V_string available_channels_;

  ColorPropertyWPtr min_color_property_;
  ColorPropertyWPtr max_color_property_;
  BoolPropertyWPtr auto_compute_intensity_bounds_property_;
  BoolPropertyWPtr use_full_rgb_colors_property_;
  FloatPropertyWPtr min_intensity_property_;
  FloatPropertyWPtr max_intensity_property_;
  EditEnumPropertyWPtr channel_name_property_;

  RetransformFunc retransform_func_;
};

class XYZPCTransformer : public PointCloudTransformer
{
public:
  XYZPCTransformer()
  {}

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class RGB8PCTransformer : public PointCloudTransformer
{
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class RGBF32PCTransformer : public PointCloudTransformer
{
public:
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
};



class FlatColorPCTransformer : public PointCloudTransformer
{
public:
  FlatColorPCTransformer()
  : color_(1.0, 1.0, 1.0)
  {}

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props);
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void setColor(const Color& color);
  const Color& getColor() { return color_; }

private:
  Color color_;
  ColorPropertyWPtr color_property_;
};

class AxisColorPCTransformer : public PointCloudTransformer
{
public:
  AxisColorPCTransformer()
    : min_value_(-10.0f)
    , max_value_(10.0f)
    , use_fixed_frame_(true)
    , axis_(AXIS_Z)
    {
      setAutoComputeBounds(true);
    }

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out);
  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props);
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void setMinValue(float val);
  void setMaxValue(float val);
  float getMinValue() { return min_value_; }
  float getMaxValue() { return max_value_; }
  void setAutoComputeBounds(bool compute);
  bool getAutoComputeBounds() { return auto_compute_bounds_; }

  void setUseFixedFrame(bool use);
  bool getUseFixedFrame() { return use_fixed_frame_; }

  enum Axis
  {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
  };

  void setAxis(int axis);
  int getAxis() { return axis_; }

private:

  float min_value_;
  float max_value_;

  bool auto_compute_bounds_;
  bool use_fixed_frame_;

  int axis_;

  BoolPropertyWPtr auto_compute_bounds_property_;
  FloatPropertyWPtr min_value_property_;
  FloatPropertyWPtr max_value_property_;
  EnumPropertyWPtr axis_property_;
  BoolPropertyWPtr use_fixed_frame_property_;
};


}

#endif // POINT_CLOUD_TRANSFORMERS_H
