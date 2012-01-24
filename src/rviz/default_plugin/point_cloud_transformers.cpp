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

#include "point_cloud_transformers.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/validate_floats.h"
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix4.h>

namespace rviz
{

static void getRainbowColor(float value, Ogre::ColourValue& color)
{
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if ( !(i&1) ) f = 1 - f; // if i is even
  float n = 1 - f;

  if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
}

uint8_t IntensityPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  updateChannels(cloud);
  return Support_Color;
}

uint8_t IntensityPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return 255;
}

bool IntensityPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t index = findChannelIndex(cloud, selected_channel_);

  if (index == -1)
  {
    if (selected_channel_ == "intensity")
    {
      index = findChannelIndex(cloud, "intensities");
      if (index == -1)
      {
	return false;
      }
    }
    else
    {
      return false;
    }
  }

  const uint32_t offset = cloud->fields[index].offset;
  const uint8_t type = cloud->fields[index].datatype;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;

  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;
  if (auto_compute_intensity_bounds_)
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      min_intensity = std::min(val, min_intensity);
      max_intensity = std::max(val, max_intensity);
    }

    min_intensity = std::max(-999999.0f, min_intensity);
    max_intensity = std::min(999999.0f, max_intensity);
    min_intensity_ = min_intensity;
    max_intensity_ = max_intensity;
  }
  else
  {
    min_intensity = min_intensity_;
    max_intensity = max_intensity_;
  }
  float diff_intensity = max_intensity - min_intensity;
  Color max_color = max_color_;
  Color min_color = min_color_;

  for (uint32_t i = 0; i < num_points; ++i)
  {
    float val = valueFromCloud<float>(cloud, offset, type, point_step, i);

    if (use_full_rgb_colors_) {
      float range = std::max(diff_intensity, 0.001f);
      float value = 1.0 - (val - min_intensity)/range;
      getRainbowColor(value, points_out[i].color);
    }
    else {
      float normalized_intensity = diff_intensity > 0.0f ? ( val - min_intensity ) / diff_intensity : 1.0f;
      normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
      points_out[i].color.r = max_color.r_*normalized_intensity + min_color.r_*(1.0f - normalized_intensity);
      points_out[i].color.g = max_color.g_*normalized_intensity + min_color.g_*(1.0f - normalized_intensity);
      points_out[i].color.b = max_color.b_*normalized_intensity + min_color.b_*(1.0f - normalized_intensity);
    }
  }

  return true;
}

void IntensityPCTransformer::reset()
{
  min_intensity_ = 0.0f;
  max_intensity_ = 4096.0f;
  selected_channel_ = std::string("intensity");
  available_channels_.clear();
}

void IntensityPCTransformer::createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props)
{
  if (mask & Support_Color)
  {
    channel_name_property_ = property_man->createProperty<EditEnumProperty>("Channel Name", prefix, boost::bind( &IntensityPCTransformer::getChannelName, this),
                                                                            boost::bind( &IntensityPCTransformer::setChannelName, this, _1), parent, this);
    setPropertyHelpText(channel_name_property_, "Select the channel to use to compute the intensity");

    use_full_rgb_colors_property_ = property_man->createProperty<BoolProperty>( "Use full RGB spectrum",
										prefix, boost::bind( &IntensityPCTransformer::getUseFullRGBColors, this),
										boost::bind( &IntensityPCTransformer::setUseFullRGBColors, this, _1),
										parent, this);
    setPropertyHelpText(use_full_rgb_colors_property_, "Whether to interpolate strictly between min and max color or use the full RGB color spectrum for intensities");

    min_color_property_ = property_man->createProperty<ColorProperty>( "Min Color", prefix, boost::bind( &IntensityPCTransformer::getMinColor, this ),
								       boost::bind( &IntensityPCTransformer::setMinColor, this, _1 ), parent, this );
    setPropertyHelpText(min_color_property_, "Color to assign the points with the minimum intensity.  Actual color is interpolated between this and Max Color.");
    max_color_property_ = property_man->createProperty<ColorProperty>( "Max Color", prefix, boost::bind( &IntensityPCTransformer::getMaxColor, this ),
								       boost::bind( &IntensityPCTransformer::setMaxColor, this, _1 ), parent, this );
    setPropertyHelpText(max_color_property_, "Color to assign the points with the maximum intensity.  Actual color is interpolated between this and Min Color.");
    ColorPropertyPtr color_prop = max_color_property_.lock();
    // legacy "Color" support... convert it to max color
    color_prop->addLegacyName("Color");

    auto_compute_intensity_bounds_property_ = property_man->createProperty<BoolProperty>( "Autocompute Intensity Bounds", prefix, boost::bind( &IntensityPCTransformer::getAutoComputeIntensityBounds, this ),
											  boost::bind( &IntensityPCTransformer::setAutoComputeIntensityBounds, this, _1 ), parent, this );
    setPropertyHelpText(auto_compute_intensity_bounds_property_, "Whether to automatically compute the intensity min/max values.");
    min_intensity_property_ = property_man->createProperty<FloatProperty>( "Min Intensity", prefix, boost::bind( &IntensityPCTransformer::getMinIntensity, this ),
									   boost::bind( &IntensityPCTransformer::setMinIntensity, this, _1 ), parent, this );
    setPropertyHelpText(min_intensity_property_, "Minimum possible intensity value, used to interpolate from Min Color to Max Color for a point.");
    max_intensity_property_ = property_man->createProperty<FloatProperty>( "Max Intensity", prefix, boost::bind( &IntensityPCTransformer::getMaxIntensity, this ),
									   boost::bind( &IntensityPCTransformer::setMaxIntensity, this, _1 ), parent, this );
    setPropertyHelpText(max_intensity_property_, "Maximum possible intensity value, used to interpolate from Min Color to Max Color for a point.");

    out_props.push_back(channel_name_property_);
    out_props.push_back(use_full_rgb_colors_property_);
    out_props.push_back(min_color_property_);
    out_props.push_back(max_color_property_);
    out_props.push_back(auto_compute_intensity_bounds_property_);
    out_props.push_back(min_intensity_property_);
    out_props.push_back(max_intensity_property_);

    if (auto_compute_intensity_bounds_)
    {
      hideProperty(min_intensity_property_);
      hideProperty(max_intensity_property_);
    }
    else
    {
      showProperty(min_intensity_property_);
      showProperty(max_intensity_property_);
    }

    if (use_full_rgb_colors_)
    {
      hideProperty(min_color_property_);
      hideProperty(max_color_property_);
    }
    else
    {
      showProperty(min_color_property_);
      showProperty(max_color_property_);
    }
  }
}

void IntensityPCTransformer::setChannelName(const std::string& channel)
{
  // If we validate channel here to be in the list of
  // available_channels_ it will always fail at load time, since
  // available_channels_ is populated dynamically as point cloud
  // messages arrive.  Therefore we don't validate it and we live with
  // the consequences at runtime.
  selected_channel_ = channel;

  propertyChanged(channel_name_property_);

  causeRetransform();
}

void IntensityPCTransformer::updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  V_string channels;
  for(size_t i = 0; i < cloud->fields.size(); ++i)
  {
    channels.push_back(cloud->fields[i].name);
  }
  std::sort(channels.begin(), channels.end());

  EditEnumPropertyPtr channel_prop = channel_name_property_.lock();
  ROS_ASSERT(channel_prop);

  if (channels != available_channels_)
  {
    channel_prop->clear();
    for(V_string::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      const std::string& channel = *it;
      if (channel.empty())
      {
	continue;
      }
      channel_prop->addOption(channel);
    }
    available_channels_ = channels;
  }
}

void IntensityPCTransformer::setMaxColor( const Color& color )
{
  max_color_ = color;

  propertyChanged(max_color_property_);

  causeRetransform();
}

void IntensityPCTransformer::setMinColor( const Color& color )
{
  min_color_ = color;

  propertyChanged(min_color_property_);

  causeRetransform();
}

void IntensityPCTransformer::setMinIntensity( float val )
{
  min_intensity_ = val;
  if (min_intensity_ > max_intensity_)
  {
    min_intensity_ = max_intensity_;
  }

  propertyChanged(min_intensity_property_);

  causeRetransform();
}

void IntensityPCTransformer::setMaxIntensity( float val )
{
  max_intensity_ = val;
  if (max_intensity_ < min_intensity_)
  {
    max_intensity_ = min_intensity_;
  }

  propertyChanged(max_intensity_property_);

  causeRetransform();
}

void IntensityPCTransformer::setAutoComputeIntensityBounds(bool compute)
{
  auto_compute_intensity_bounds_ = compute;

  if (auto_compute_intensity_bounds_)
  {
    hideProperty(min_intensity_property_);
    hideProperty(max_intensity_property_);
  }
  else
  {
    showProperty(min_intensity_property_);
    showProperty(max_intensity_property_);
  }

  propertyChanged(auto_compute_intensity_bounds_property_);

  causeRetransform();
}

void IntensityPCTransformer::setUseFullRGBColors(bool full_rgb)
{
  use_full_rgb_colors_ = full_rgb;

  if (use_full_rgb_colors_)
  {
    hideProperty(min_color_property_);
    hideProperty(max_color_property_);
  }
  else
  {
    showProperty(min_color_property_);
    showProperty(max_color_property_);
  }

  propertyChanged(use_full_rgb_colors_property_);

  causeRetransform();
}

uint8_t XYZPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return Support_None;
  }

  if (cloud->fields[xi].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_XYZ;
  }

  return Support_None;
}

bool XYZPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_XYZ))
  {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    float x = *reinterpret_cast<const float*>(point + xoff);
    float y = *reinterpret_cast<const float*>(point + yoff);
    float z = *reinterpret_cast<const float*>(point + zoff);

    Ogre::Vector3 pos(x, y, z);
    pos = transform * pos;
    points_out[i].position = pos;
  }

  return true;
}

uint8_t RGB8PCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t index = findChannelIndex(cloud, "rgb");
  if (index == -1)
  {
    return Support_None;
  }

  if (cloud->fields[index].datatype == sensor_msgs::PointField::INT32 ||
      cloud->fields[index].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGB8PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t index = findChannelIndex(cloud, "rgb");

  const uint32_t off = cloud->fields[index].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    uint32_t rgb = *reinterpret_cast<const uint32_t*>(point + off);
    float r = ((rgb >> 16) & 0xff) / 255.0f;
    float g = ((rgb >> 8) & 0xff) / 255.0f;
    float b = (rgb & 0xff) / 255.0f;
    points_out[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

uint8_t RGBF32PCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");
  if (ri == -1 || gi == -1 || bi == -1)
  {
    return Support_None;
  }

  if (cloud->fields[ri].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGBF32PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");

  const uint32_t roff = cloud->fields[ri].offset;
  const uint32_t goff = cloud->fields[gi].offset;
  const uint32_t boff = cloud->fields[bi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    float r = *reinterpret_cast<const float*>(point + roff);
    float g = *reinterpret_cast<const float*>(point + goff);
    float b = *reinterpret_cast<const float*>(point + boff);
    points_out[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

uint8_t FlatColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return Support_Color;
}

uint8_t FlatColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return 0;
}

bool FlatColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  const uint32_t num_points = cloud->width * cloud->height;
  for (uint32_t i = 0; i < num_points; ++i)
  {
    points_out[i].color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_);
  }

  return true;
}

void FlatColorPCTransformer::setColor(const Color& c)
{
  color_ = c;
  propertyChanged(color_property_);
  causeRetransform();
}

void FlatColorPCTransformer::createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props)
{
  if (mask & Support_Color)
  {
    color_property_ = property_man->createProperty<ColorProperty>("Color", prefix, boost::bind( &FlatColorPCTransformer::getColor, this ),
                                                                  boost::bind( &FlatColorPCTransformer::setColor, this, _1 ), parent, this);
    setPropertyHelpText(color_property_, "Color to assign to every point.");

    out_props.push_back(color_property_);
  }
}

uint8_t AxisColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return Support_Color;
}

uint8_t AxisColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return 255;
}

bool AxisColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& points_out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();

  // compute bounds

  float min_value_current = 9999.0f;
  float max_value_current = -9999.0f;
  std::vector<float> values;
  values.reserve(num_points);

  for (uint32_t i = 0; i < num_points; ++i, point += point_step)
  {
    float x = *reinterpret_cast<const float*>(point + xoff);
    float y = *reinterpret_cast<const float*>(point + yoff);
    float z = *reinterpret_cast<const float*>(point + zoff);

    Ogre::Vector3 pos(x, y, z);

    if (use_fixed_frame_)
    {
      pos = transform * pos;
    }

    float val = pos[axis_];
    min_value_current = std::min(min_value_current, val);
    max_value_current = std::max(max_value_current, val);

    values.push_back(val);
  }

  if (auto_compute_bounds_)
  {
    min_value_ = min_value_current;
    max_value_ = max_value_current;
  }

  for (uint32_t i = 0; i < num_points; ++i)
  {
    float range = std::max(max_value_ - min_value_, 0.001f);
    float value = 1.0 - (values[i] - min_value_)/range;
    getRainbowColor(value, points_out[i].color);
  }

  return true;
}

void AxisColorPCTransformer::createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBaseWPtr& out_props)
{
  if (mask & Support_Color)
  {
    axis_property_ = property_man->createProperty<EnumProperty>("Axis", prefix, boost::bind(&AxisColorPCTransformer::getAxis, this), boost::bind(&AxisColorPCTransformer::setAxis, this, _1),
                                                                parent, this);
    EnumPropertyPtr prop = axis_property_.lock();
    prop->addOption("X", AXIS_X);
    prop->addOption("Y", AXIS_Y);
    prop->addOption("Z", AXIS_Z);
    setPropertyHelpText(axis_property_, "The axis to interpolate the color along.");
    auto_compute_bounds_property_ = property_man->createProperty<BoolProperty>( "Autocompute Value Bounds", prefix, boost::bind( &AxisColorPCTransformer::getAutoComputeBounds, this ),
                                                                              boost::bind( &AxisColorPCTransformer::setAutoComputeBounds, this, _1 ), parent, this );

    setPropertyHelpText(auto_compute_bounds_property_, "Whether to automatically compute the value min/max values.");
    min_value_property_ = property_man->createProperty<FloatProperty>( "Min Value", prefix, boost::bind( &AxisColorPCTransformer::getMinValue, this ),
                                                                              boost::bind( &AxisColorPCTransformer::setMinValue, this, _1 ), parent, this );
    setPropertyHelpText(min_value_property_, "Minimum value value, used to interpolate the color of a point.");
    max_value_property_ = property_man->createProperty<FloatProperty>( "Max Value", prefix, boost::bind( &AxisColorPCTransformer::getMaxValue, this ),
                                                                            boost::bind( &AxisColorPCTransformer::setMaxValue, this, _1 ), parent, this );
    setPropertyHelpText(max_value_property_, "Maximum value value, used to interpolate the color of a point.");

    use_fixed_frame_property_ = property_man->createProperty<BoolProperty>( "Use Fixed Frame", prefix, boost::bind( &AxisColorPCTransformer::getUseFixedFrame, this ),
                                                                            boost::bind( &AxisColorPCTransformer::setUseFixedFrame, this, _1 ), parent, this );
    setPropertyHelpText(use_fixed_frame_property_, "Whether to color the cloud based on its fixed frame position or its local frame position.");

    out_props.push_back(axis_property_);
    out_props.push_back(auto_compute_bounds_property_);
    out_props.push_back(min_value_property_);
    out_props.push_back(max_value_property_);
    out_props.push_back(use_fixed_frame_property_);

    if (auto_compute_bounds_)
    {
      hideProperty(min_value_property_);
      hideProperty(max_value_property_);
    }
    else
    {
      showProperty(min_value_property_);
      showProperty(max_value_property_);
    }
  }
}

void AxisColorPCTransformer::setUseFixedFrame(bool use)
{
  use_fixed_frame_ = use;
  propertyChanged(use_fixed_frame_property_);
  causeRetransform();
}

void AxisColorPCTransformer::setAxis(int axis)
{
  axis_ = axis;
  propertyChanged(axis_property_);
  causeRetransform();
}

void AxisColorPCTransformer::setMinValue( float val )
{
  min_value_ = val;
  if (min_value_ > max_value_)
  {
    min_value_ = max_value_;
  }

  propertyChanged(min_value_property_);

  causeRetransform();
}

void AxisColorPCTransformer::setMaxValue( float val )
{
  max_value_ = val;
  if (max_value_ < min_value_)
  {
    max_value_ = min_value_;
  }

  propertyChanged(max_value_property_);

  causeRetransform();
}

void AxisColorPCTransformer::setAutoComputeBounds(bool compute)
{
  auto_compute_bounds_ = compute;

  if (auto_compute_bounds_)
  {
    hideProperty(min_value_property_);
    hideProperty(max_value_property_);
  }
  else
  {
    showProperty(min_value_property_);
    showProperty(max_value_property_);
  }

  propertyChanged(auto_compute_bounds_property_);

  causeRetransform();
}

}
