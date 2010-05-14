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

#include "point_cloud_base.h"
#include "point_cloud_transformer.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/frame_manager.h"

#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreWireBoundingBox.h>

namespace rviz
{

template<typename T>
T getValue(const T& val)
{
  return val;
}

class PointCloudSelectionHandler : public SelectionHandler
{
public:
  PointCloudSelectionHandler(PointCloudBase* display);
  virtual ~PointCloudSelectionHandler();

  virtual void createProperties(const Picked& obj, PropertyManager* property_manager);
  virtual void destroyProperties(const Picked& obj, PropertyManager* property_manager);

  virtual bool needsAdditionalRenderPass(uint32_t pass)
  {
    if (pass < 2)
    {
      return true;
    }

    return false;
  }

  virtual void preRenderPass(uint32_t pass);
  virtual void postRenderPass(uint32_t pass);

  virtual void onSelect(const Picked& obj);
  virtual void onDeselect(const Picked& obj);

  virtual void getAABBs(const Picked& obj, V_AABB& aabbs);

private:
  void getCloudAndLocalIndexByGlobalIndex(int global_index, sensor_msgs::PointCloud2Ptr& cloud_out, int& index_out);

  PointCloudBase* display_;
};

PointCloudSelectionHandler::PointCloudSelectionHandler(PointCloudBase* display)
: display_(display)
{
}

PointCloudSelectionHandler::~PointCloudSelectionHandler()
{
}

void PointCloudSelectionHandler::preRenderPass(uint32_t pass)
{
  SelectionHandler::preRenderPass(pass);

  if (pass == 1)
  {
    display_->cloud_->setColorByIndex(true);
  }
}

void PointCloudSelectionHandler::postRenderPass(uint32_t pass)
{
  SelectionHandler::postRenderPass(pass);

  if (pass == 1)
  {
    display_->cloud_->setColorByIndex(false);
  }
}

void PointCloudSelectionHandler::getCloudAndLocalIndexByGlobalIndex(int global_index, sensor_msgs::PointCloud2Ptr& cloud_out, int& index_out)
{
  boost::mutex::scoped_lock lock(display_->clouds_mutex_);

  int count = 0;

  PointCloudBase::D_CloudInfo::iterator cloud_it = display_->clouds_.begin();
  PointCloudBase::D_CloudInfo::iterator cloud_end = display_->clouds_.end();
  for (;cloud_it != cloud_end; ++cloud_it)
  {
    const PointCloudBase::CloudInfoPtr& info = *cloud_it;

    if (global_index < count + (int)info->num_points_)
    {
      index_out = global_index - count;
      cloud_out = info->message_;

      return;
    }

    count += info->message_->width * info->message_->height;
  }
}

void PointCloudSelectionHandler::createProperties(const Picked& obj, PropertyManager* property_manager)
{
  typedef std::set<int> S_int;
  S_int indices;
  {
    S_uint64::const_iterator it = obj.extra_handles.begin();
    S_uint64::const_iterator end = obj.extra_handles.end();
    for (; it != end; ++it)
    {
      uint64_t handle = *it;
      indices.insert((handle & 0xffffffff) - 1);
    }
  }

  {
    S_int::iterator it = indices.begin();
    S_int::iterator end = indices.end();
    for (; it != end; ++it)
    {
      int global_index = *it;
      int index = 0;
      sensor_msgs::PointCloud2Ptr message;

      getCloudAndLocalIndexByGlobalIndex(global_index, message, index);

      if (!message)
      {
        continue;
      }

      std::stringstream prefix;
      prefix << "Point " << index << " [cloud " << message.get() << "]";

      if (property_manager->hasProperty(prefix.str(), ""))
      {
        continue;
      }

#if 0
      CategoryPropertyWPtr cat = property_manager->createCategory(prefix.str(), "");

      Ogre::Vector3 pos(message->points[index].x, message->points[index].y, message->points[index].z);
      property_manager->createProperty<Vector3Property>("Position", prefix.str(), boost::bind(getValue<Ogre::Vector3>, pos), Vector3Property::Setter(), cat);

      for (int channel = 0; channel < (int)message->channels.size(); ++channel)
      {
        sensor_msgs::ChannelFloat32& c = message->channels[channel];
        const std::string& name = c.name;

        std::stringstream ss;
        ss << "Channel " << channel << " [" << name << "]";
        property_manager->createProperty<FloatProperty>(ss.str(), prefix.str(), boost::bind(getValue<float>, c.values[index]), FloatProperty::Setter(), cat);
      }
#endif
    }
  }
}

void PointCloudSelectionHandler::destroyProperties(const Picked& obj, PropertyManager* property_manager)
{
  typedef std::set<int> S_int;
  S_int indices;
  {
    S_uint64::const_iterator it = obj.extra_handles.begin();
    S_uint64::const_iterator end = obj.extra_handles.end();
    for (; it != end; ++it)
    {
      uint64_t handle = *it;
      indices.insert((handle & 0xffffffff) - 1);
    }
  }

  {
    S_int::iterator it = indices.begin();
    S_int::iterator end = indices.end();
    for (; it != end; ++it)
    {
      int global_index = *it;
      int index = 0;
      sensor_msgs::PointCloud2Ptr message;

      getCloudAndLocalIndexByGlobalIndex(global_index, message, index);

      if (!message)
      {
        continue;
      }

      std::stringstream prefix;
      prefix << "Point " << index << " [cloud " << message.get() << "]";

      if (property_manager->hasProperty(prefix.str(), ""))
      {
        property_manager->deleteProperty(prefix.str(), "");
      }
    }
  }
}

void PointCloudSelectionHandler::getAABBs(const Picked& obj, V_AABB& aabbs)
{
  S_uint64::iterator it = obj.extra_handles.begin();
  S_uint64::iterator end = obj.extra_handles.end();
  for (; it != end; ++it)
  {
    M_HandleToBox::iterator find_it = boxes_.find(std::make_pair(obj.handle, *it - 1));
    if (find_it != boxes_.end())
    {
      Ogre::WireBoundingBox* box = find_it->second.second;

      aabbs.push_back(box->getWorldBoundingBox());
    }
  }
}

void PointCloudSelectionHandler::onSelect(const Picked& obj)
{
  S_uint64::iterator it = obj.extra_handles.begin();
  S_uint64::iterator end = obj.extra_handles.end();
  for (; it != end; ++it)
  {
    int global_index = (*it & 0xffffffff) - 1;

    int index = 0;
    sensor_msgs::PointCloud2Ptr message;

    getCloudAndLocalIndexByGlobalIndex(global_index, message, index);

    if (!message)
    {
      continue;
    }

#if 0
    Ogre::Vector3 pos(message->points[index].x, message->points[index].y, message->points[index].z);
    robotToOgre(pos);

    float size = 0.002;
    if (display_->style_ != PointCloudBase::Points)
    {
      size = display_->billboard_size_ / 2.0;
    }

    Ogre::AxisAlignedBox aabb(pos - size, pos + size);

    createBox(std::make_pair(obj.handle, global_index), aabb, "RVIZ/Cyan");
#endif
  }
}

void PointCloudSelectionHandler::onDeselect(const Picked& obj)
{
  S_uint64::iterator it = obj.extra_handles.begin();
  S_uint64::iterator end = obj.extra_handles.end();
  for (; it != end; ++it)
  {
    int global_index = (*it & 0xffffffff) - 1;

    destroyBox(std::make_pair(obj.handle, global_index));
  }
}

int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& channel)
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
  IntensityPCTransformer(PointCloudBase* parent)
  : name_("Intensity")
  , min_color_( 0.0f, 0.0f, 0.0f )
  , max_color_( 1.0f, 1.0f, 1.0f )
  , min_intensity_(0.0f)
  , max_intensity_(4096.0f)
  , parent_(parent)
  {
    setAutoComputeIntensityBounds(true);
  }

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
  virtual void reset();
  virtual const std::string& getName() { return name_; }
  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBase& out_props);

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

private:
  std::string name_;

  Color min_color_;
  Color max_color_;
  float min_intensity_;
  float max_intensity_;
  bool auto_compute_intensity_bounds_;
  bool intensity_bounds_changed_;

  ColorPropertyWPtr min_color_property_;
  ColorPropertyWPtr max_color_property_;
  BoolPropertyWPtr auto_compute_intensity_bounds_property_;
  FloatPropertyWPtr min_intensity_property_;
  FloatPropertyWPtr max_intensity_property_;

  PointCloudBase* parent_;
};

uint8_t IntensityPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t index = findChannelIndex(cloud, "intensity");
  if (index == -1)
  {
    index = findChannelIndex(cloud, "intensities");
  }

  if (index == -1)
  {
    return Support_None;
  }

  return Support_Color;
}

bool IntensityPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  int32_t index = findChannelIndex(cloud, "intensity");
  if (index == -1)
  {
    index = findChannelIndex(cloud, "intensities");
  }

  if (index == -1)
  {
    return false;
  }

  const uint32_t offset = cloud->fields[index].offset;
  const uint8_t type = cloud->fields[index].datatype;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;

  float min_intensity = 999999.0f;
  float max_intensity = 0.0f;
  if (auto_compute_intensity_bounds_)
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      min_intensity = std::min(val, min_intensity);
      max_intensity = std::max(val, max_intensity);
    }

    min_intensity = std::max(0.0f, min_intensity);
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

    float normalized_intensity = diff_intensity > 0.0f ? ( val - min_intensity ) / diff_intensity : 1.0f;
    normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
    out.points[i].color.r = max_color.r_*normalized_intensity + min_color.r_*(1.0f - normalized_intensity);
    out.points[i].color.g = max_color.g_*normalized_intensity + min_color.g_*(1.0f - normalized_intensity);
    out.points[i].color.b = max_color.b_*normalized_intensity + min_color.b_*(1.0f - normalized_intensity);
  }

  return true;
}

void IntensityPCTransformer::reset()
{
  min_intensity_ = 0.0f;
  max_intensity_ = 4096.0f;
}

void IntensityPCTransformer::createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBase& out_props)
{
  if (mask & Support_Color)
  {
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

    out_props.push_back(min_color_property_.lock());
    out_props.push_back(max_color_property_.lock());
    out_props.push_back(auto_compute_intensity_bounds_property_.lock());
    out_props.push_back(min_intensity_property_.lock());
    out_props.push_back(max_intensity_property_.lock());

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
  }
}

void IntensityPCTransformer::setMaxColor( const Color& color )
{
  max_color_ = color;

  propertyChanged(max_color_property_);

  parent_->causeRetransform();
}

void IntensityPCTransformer::setMinColor( const Color& color )
{
  min_color_ = color;

  propertyChanged(min_color_property_);

  parent_->causeRetransform();
}

void IntensityPCTransformer::setMinIntensity( float val )
{
  min_intensity_ = val;
  if (min_intensity_ > max_intensity_)
  {
    min_intensity_ = max_intensity_;
  }

  propertyChanged(min_intensity_property_);

  parent_->causeRetransform();
}

void IntensityPCTransformer::setMaxIntensity( float val )
{
  max_intensity_ = val;
  if (max_intensity_ < min_intensity_)
  {
    max_intensity_ = min_intensity_;
  }

  propertyChanged(max_intensity_property_);

  parent_->causeRetransform();
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

  parent_->causeRetransform();
}

class XYZPCTransformer : public PointCloudTransformer
{
public:
  XYZPCTransformer()
  : name_("XYZ")
  {}

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
  virtual const std::string& getName() { return name_; }

private:
  std::string name_;
};

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

bool XYZPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
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
    out.points[i].position = pos;
  }

  return true;
}

class RGB8PCTransformer : public PointCloudTransformer
{
public:
  RGB8PCTransformer()
  : name_("RGB8")
  {}

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
  virtual const std::string& getName() { return name_; }

private:
  std::string name_;
};

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

bool RGB8PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
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
    out.points[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

class RGBF32PCTransformer : public PointCloudTransformer
{
public:
  RGBF32PCTransformer()
  : name_("RGBF32")
  {}

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
  virtual const std::string& getName() { return name_; }

private:
  std::string name_;
};

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

bool RGBF32PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
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
    out.points[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

class FlatColorPCTransformer : public PointCloudTransformer
{
public:
  FlatColorPCTransformer(PointCloudBase* parent)
  : name_("Flat Color")
  , color_(1.0, 1.0, 1.0)
  , parent_(parent)
  {}

  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out);
  virtual const std::string& getName() { return name_; }
  virtual void createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBase& out_props);

  void setColor(const Color& color);
  const Color& getColor() { return color_; }

private:
  std::string name_;
  Color color_;
  ColorPropertyWPtr color_property_;
  PointCloudBase* parent_;
};

uint8_t FlatColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  return Support_Color;
}

bool FlatColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, PointCloud& out)
{
  if (!(mask & Support_Color))
  {
    return false;
  }

  const uint32_t num_points = cloud->width * cloud->height;
  for (uint32_t i = 0; i < num_points; ++i)
  {
    out.points[i].color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_);
  }

  return true;
}

void FlatColorPCTransformer::setColor(const Color& c)
{
  color_ = c;
  propertyChanged(color_property_);
  parent_->causeRetransform();
}

void FlatColorPCTransformer::createProperties(PropertyManager* property_man, const CategoryPropertyWPtr& parent, const std::string& prefix, uint32_t mask, V_PropertyBase& out_props)
{
  if (mask & Support_Color)
  {
    color_property_ = property_man->createProperty<ColorProperty>("Color", prefix, boost::bind( &FlatColorPCTransformer::getColor, this ),
                                                                  boost::bind( &FlatColorPCTransformer::setColor, this, _1 ), parent, this);
    setPropertyHelpText(color_property_, "Color to assign to every point.");

    out_props.push_back(color_property_.lock());
  }
}

PointCloudBase::CloudInfo::CloudInfo(VisualizationManager* manager)
: time_(0.0f)
, transform_(Ogre::Matrix4::ZERO)
, num_points_(0)
, vis_manager_(manager)
{}

PointCloudBase::CloudInfo::~CloudInfo()
{
}

PointCloudBase::PointCloudBase( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, new_cloud_(false)
, new_xyz_transformer_(false)
, new_color_transformer_(false)
, needs_retransform_(false)
, style_( Billboards )
, billboard_size_( 0.01 )
, point_decay_time_(0.0f)
, selectable_(false)
, coll_handle_(0)
, messages_received_(0)
, total_point_count_(0)
{
  cloud_ = new ogre_tools::PointCloud();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->attachObject(cloud_);
  coll_handler_ = PointCloudSelectionHandlerPtr(new PointCloudSelectionHandler(this));

  setStyle( style_ );
  setBillboardSize( billboard_size_ );
  setAlpha(1.0f);

  setSelectable(true);

  transformers_.push_back(PointCloudTransformerPtr(new XYZPCTransformer));
  transformers_.push_back(PointCloudTransformerPtr(new IntensityPCTransformer(this)));
  transformers_.push_back(PointCloudTransformerPtr(new RGB8PCTransformer));
  transformers_.push_back(PointCloudTransformerPtr(new RGBF32PCTransformer));
  transformers_.push_back(PointCloudTransformerPtr(new FlatColorPCTransformer(this)));
  xyz_transformer_ = -1;
  color_transformer_ = -1;
}

PointCloudBase::~PointCloudBase()
{
  if (coll_handle_)
  {
    SelectionManager* sel_manager = vis_manager_->getSelectionManager();
    sel_manager->removeObject(coll_handle_);
  }

  scene_manager_->destroySceneNode(scene_node_->getName());
  delete cloud_;
}

void PointCloudBase::setAlpha( float alpha )
{
  alpha_ = alpha;

  cloud_->setAlpha(alpha_);

  propertyChanged(alpha_property_);
}

void PointCloudBase::setSelectable( bool selectable )
{
  if (selectable_ != selectable)
  {
    SelectionManager* sel_manager = vis_manager_->getSelectionManager();

    if (selectable)
    {
      coll_handle_ = sel_manager->createHandle();

      sel_manager->addObject(coll_handle_, coll_handler_);

      // Break out coll handle into r/g/b/a floats
      float r = ((coll_handle_ >> 16) & 0xff) / 255.0f;
      float g = ((coll_handle_ >> 8) & 0xff) / 255.0f;
      float b = (coll_handle_ & 0xff) / 255.0f;
      Ogre::ColourValue col(r, g, b, 1.0f);
      cloud_->setPickColor(col);
    }
    else
    {
      sel_manager->removeObject(coll_handle_);
      coll_handle_ = 0;
      cloud_->setPickColor(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
    }
  }

  selectable_ = selectable;

  propertyChanged(selectable_property_);
}

void PointCloudBase::setDecayTime( float time )
{
  point_decay_time_ = time;

  propertyChanged(decay_time_property_);

  causeRender();
}

void PointCloudBase::setStyle( int style )
{
  ROS_ASSERT( style < StyleCount );

  style_ = style;

  ogre_tools::PointCloud::RenderMode mode = ogre_tools::PointCloud::RM_POINTS;
  if (style == Billboards)
  {
    mode = ogre_tools::PointCloud::RM_BILLBOARDS;
  }
  else if (style == BillboardSpheres)
  {
    mode = ogre_tools::PointCloud::RM_BILLBOARD_SPHERES;
  }
  else if (style == Boxes)
  {
    mode = ogre_tools::PointCloud::RM_BOXES;
  }

  if (style == Points)
  {
    hideProperty(billboard_size_property_);
  }
  else
  {
    showProperty(billboard_size_property_);
  }

  cloud_->setRenderMode(mode);

  propertyChanged(style_property_);

  causeRender();
}

void PointCloudBase::setBillboardSize( float size )
{
  billboard_size_ = size;

  cloud_->setDimensions( size, size, size );

  propertyChanged(billboard_size_property_);

  causeRender();
}

void PointCloudBase::onEnable()
{
}

void PointCloudBase::onDisable()
{
  clouds_.clear();
  cloud_->clear();
  messages_received_ = 0;
  total_point_count_ = 0;
}

void PointCloudBase::causeRetransform()
{
  boost::mutex::scoped_lock lock(clouds_mutex_);
  needs_retransform_ = true;
}

void PointCloudBase::update(float wall_dt, float ros_dt)
{
  {
    boost::mutex::scoped_lock lock(clouds_mutex_);

    if (needs_retransform_)
    {
      retransform();
      needs_retransform_ = false;
    }

    D_CloudInfo::iterator cloud_it = clouds_.begin();
    D_CloudInfo::iterator cloud_end = clouds_.end();
    for (;cloud_it != cloud_end; ++cloud_it)
    {
      const CloudInfoPtr& info = *cloud_it;

      info->time_ += ros_dt;
    }

    if (point_decay_time_ > 0.0f)
    {
      bool removed = false;
      uint32_t points_to_pop = 0;
      while (!clouds_.empty() && clouds_.front()->time_ > point_decay_time_)
      {
        total_point_count_ -= clouds_.front()->num_points_;
        points_to_pop += clouds_.front()->num_points_;
        clouds_.pop_front();
        removed = true;
      }

      if (removed)
      {
        cloud_->popPoints(points_to_pop);
        causeRender();
      }
    }
  }

  if (new_cloud_)
  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);

    if (point_decay_time_ == 0.0f)
    {
      clouds_.clear();
      cloud_->clear();

      ROS_ASSERT(!new_points_.empty());
      ROS_ASSERT(!new_clouds_.empty());
      V_Point& points = new_points_.back();
      cloud_->addPoints(&points.front(), points.size());
      clouds_.push_back(new_clouds_.back());

      total_point_count_ = points.size();
    }
    else
    {
      {
        VV_Point::iterator it = new_points_.begin();
        VV_Point::iterator end = new_points_.end();
        for (; it != end; ++it)
        {
          V_Point& points = *it;
          total_point_count_ += points.size();
          cloud_->addPoints( &points.front(), points.size() );
        }
      }

      {
        V_CloudInfo::iterator it = new_clouds_.begin();
        V_CloudInfo::iterator end = new_clouds_.end();
        for (; it != end; ++it)
        {
          clouds_.push_back(*it);
        }
      }
    }

    new_clouds_.clear();
    new_points_.clear();
    new_cloud_ = false;
  }

  {
    boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
    if (new_xyz_transformer_)
    {
      V_PropertyBase::iterator it = xyz_props_.begin();
      V_PropertyBase::iterator end = xyz_props_.end();
      for (; it != end; ++it)
      {
        property_manager_->deleteProperty(*it);
      }

      xyz_props_.clear();
      if (xyz_transformer_ >= 0)
      {
        transformers_[xyz_transformer_]->createProperties(property_manager_, parent_category_, property_prefix_, PointCloudTransformer::Support_XYZ, xyz_props_);
      }
    }

    if (new_color_transformer_)
    {
      V_PropertyBase::iterator it = color_props_.begin();
      V_PropertyBase::iterator end = color_props_.end();
      for (; it != end; ++it)
      {
        property_manager_->deleteProperty(*it);
      }

      color_props_.clear();
      if (color_transformer_ >= 0)
      {
        transformers_[color_transformer_]->createProperties(property_manager_, parent_category_, property_prefix_, PointCloudTransformer::Support_Color, color_props_);
      }
    }

    new_xyz_transformer_ = false;
    new_color_transformer_ = false;
  }

  updateStatus();
}

void PointCloudBase::updateTransformers(const sensor_msgs::PointCloud2Ptr& cloud)
{
  EnumPropertyPtr xyz_prop = xyz_transformer_property_.lock();
  if (xyz_prop)
  {
    xyz_prop->clear();
  }

  EnumPropertyPtr color_prop = color_transformer_property_.lock();
  if (color_prop)
  {
    color_prop->clear();
  }

  // Get the channels that we could potentially render
  int32_t xyz_index = getXYZTransformer();
  int32_t color_index = getColorTransformer();

  typedef std::set<int32_t> S_int32;
  S_int32 valid_xyz, valid_color;
  uint32_t index = 0;
  V_PointCloudTransformer::iterator trans_it = transformers_.begin();
  V_PointCloudTransformer::iterator trans_end = transformers_.end();
  for(;trans_it != trans_end; ++trans_it, ++index)
  {
    const PointCloudTransformerPtr& trans = *trans_it;
    uint32_t mask = trans->supports(cloud);
    if (mask & PointCloudTransformer::Support_XYZ)
    {
      valid_xyz.insert(index);

      if (xyz_prop)
      {
        xyz_prop->addOption(trans->getName(), index);
      }
    }

    if (mask & PointCloudTransformer::Support_Color)
    {
      valid_color.insert(index);

      if (color_prop)
      {
        color_prop->addOption(trans->getName(), index);
      }
    }
  }

  if (valid_xyz.find(xyz_index) == valid_xyz.end())
  {
    xyz_index = -1;
    if (!valid_xyz.empty())
    {
      xyz_index = *valid_xyz.begin();
    }
  }

  if (valid_color.find(color_index) == valid_color.end())
  {
    color_index = -1;
    if (!valid_color.empty())
    {
      color_index = *valid_color.begin();
    }
  }

  if (xyz_prop)
  {
    xyz_prop->changed();
  }

  if (color_prop)
  {
    color_prop->changed();
  }

  setXYZTransformer(xyz_index);
  setColorTransformer(color_index);
}

void PointCloudBase::updateStatus()
{
  if (messages_received_ == 0)
  {
    setStatus(status_levels::Warn, "Topic", "No messages received");
  }
  else
  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(status_levels::Ok, "Topic", ss.str());
  }

  {
    std::stringstream ss;
    ss << "Showing [" << total_point_count_ << "] points from [" << clouds_.size() << "] messages";
    setStatus(status_levels::Ok, "Points", ss.str());
  }
}

void PointCloudBase::processMessage(const sensor_msgs::PointCloud2Ptr& cloud)
{
  CloudInfoPtr info(new CloudInfo(vis_manager_));
  info->message_ = cloud;
  info->time_ = 0;

  V_Point points;
  if (transformCloud(info, points))
  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);

    new_clouds_.push_back(info);
    new_points_.push_back(V_Point());
    new_points_.back().swap(points);

    new_cloud_ = true;
  }
}

void PointCloudBase::setXYZTransformer(int32_t index)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (xyz_transformer_ == index)
  {
    return;
  }

  if (index >= (int32_t)transformers_.size())
  {
    return;
  }

  xyz_transformer_ = index;
  new_xyz_transformer_ = true;
  propertyChanged(xyz_transformer_property_);

  causeRetransform();
}

void PointCloudBase::setColorTransformer(int32_t index)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (color_transformer_ == index)
  {
    return;
  }

  if (index >= (int32_t)transformers_.size())
  {
    return;
  }

  color_transformer_ = index;
  new_color_transformer_ = true;
  propertyChanged(color_transformer_property_);

  causeRetransform();
}

PointCloudTransformerPtr PointCloudBase::getXYZTransformer(const sensor_msgs::PointCloud2Ptr& cloud)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (xyz_transformer_ >= 0 && xyz_transformer_ < (int32_t)transformers_.size())
  {
    const PointCloudTransformerPtr& trans = transformers_[xyz_transformer_];
    if (trans->supports(cloud) & PointCloudTransformer::Support_XYZ)
    {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

PointCloudTransformerPtr PointCloudBase::getColorTransformer(const sensor_msgs::PointCloud2Ptr& cloud)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (color_transformer_ >= 0 && color_transformer_ < (int32_t)transformers_.size())
  {
    const PointCloudTransformerPtr& trans = transformers_[color_transformer_];
    if (trans->supports(cloud) & PointCloudTransformer::Support_Color)
    {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

void PointCloudBase::retransform()
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

  cloud_->clear();

  D_CloudInfo::iterator it = clouds_.begin();
  D_CloudInfo::iterator end = clouds_.end();
  for (; it != end; ++it)
  {
    const CloudInfoPtr& cloud = *it;
    V_Point points;
    transformCloud(cloud, points);
    if (!points.empty())
    {
      cloud_->addPoints(&points.front(), points.size());
    }
  }
}

bool PointCloudBase::transformCloud(const CloudInfoPtr& info, V_Point& points)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

  Ogre::Matrix4 transform = info->transform_;

  if (transform == Ogre::Matrix4::ZERO)
  {
    Ogre::Vector3 pos;
    Ogre::Quaternion orient;
    if (!vis_manager_->getFrameManager()->getTransform(info->message_->header, pos, orient, false))
    {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << info->message_->header.frame_id << "] to frame [" << vis_manager_->getFrameManager()->getFixedFrame() << "]";
      setStatus(status_levels::Error, "Message", ss.str());
      return false;
    }

    transform = Ogre::Matrix4(orient);
    transform.setTrans(pos);
    info->transform_ = transform;
  }

  PointCloud cloud;
  size_t size = info->message_->width * info->message_->height;
  info->num_points_ = size;
  PointCloudPoint default_pt;
  default_pt.color = Ogre::ColourValue(1, 1, 1);
  default_pt.position = Ogre::Vector3::ZERO;
  cloud.points.resize(size, default_pt);

  updateTransformers(info->message_);
  PointCloudTransformerPtr xyz_trans = getXYZTransformer(info->message_);
  PointCloudTransformerPtr color_trans = getColorTransformer(info->message_);

  if (!xyz_trans)
  {
    std::stringstream ss;
    ss << "No position transformer available for cloud";
    setStatus(status_levels::Error, "Message", ss.str());
    return false;
  }

  if (!color_trans)
  {
    std::stringstream ss;
    ss << "No color transformer available for cloud";
    setStatus(status_levels::Error, "Message", ss.str());
    return false;
  }

  xyz_trans->transform(info->message_, PointCloudTransformer::Support_XYZ, transform, cloud);
  color_trans->transform(info->message_, PointCloudTransformer::Support_Color, transform, cloud);

  points.resize(size);
  for (size_t i = 0; i < size; ++i)
  {
    Ogre::Vector3 pos = cloud.points[i].position;
    Ogre::ColourValue color = cloud.points[i].color;
    if (validateFloats(pos))
    {
      points[i].x = pos.x;
      points[i].y = pos.y;
      points[i].z = pos.z;
    }
    else
    {
      points[i].x = 999999.0f;
      points[i].y = 999999.0f;
      points[i].z = 999999.0f;
    }
    points[i].setColor(color.r, color.g, color.b);
  }

  return true;
}

bool convertPointCloudToPointCloud2(const sensor_msgs::PointCloud& input, sensor_msgs::PointCloud2& output)
{
  output.header = input.header;
  output.width  = input.points.size ();
  output.height = 1;
  output.fields.resize (3 + input.channels.size ());
  // Convert x/y/z to fields
  output.fields[0].name = "x"; output.fields[1].name = "y"; output.fields[2].name = "z";
  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < output.fields.size (); ++d, offset += 4)
  {
    output.fields[d].offset = offset;
    output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }
  output.point_step = offset;
  output.row_step   = output.point_step * output.width;
  // Convert the remaining of the channels to fields
  for (size_t d = 0; d < input.channels.size (); ++d)
    output.fields[3 + d].name = input.channels[d].name;
  output.data.resize (input.points.size () * output.point_step);
  output.is_bigendian = false;  // @todo ?
  output.is_dense     = false;

  // Copy the data points
  for (size_t cp = 0; cp < input.points.size (); ++cp)
  {
    memcpy (&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x, sizeof (float));
    memcpy (&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y, sizeof (float));
    memcpy (&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z, sizeof (float));
    for (size_t d = 0; d < input.channels.size (); ++d)
      memcpy (&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof (float));
  }
  return (true);
}

void PointCloudBase::addMessage(const sensor_msgs::PointCloudConstPtr& cloud)
{
  sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
  convertPointCloudToPointCloud2(*cloud, *out);
  addMessage(out);
}

void PointCloudBase::addMessage(const sensor_msgs::PointCloud2Ptr& cloud)
{
  ++messages_received_;

  if (cloud->width * cloud->height == 0)
  {
    return;
  }

  processMessage(cloud);
}

void PointCloudBase::fixedFrameChanged()
{
  reset();
}

void PointCloudBase::createProperties()
{
  selectable_property_ = property_manager_->createProperty<BoolProperty>( "Selectable", property_prefix_, boost::bind( &PointCloudBase::getSelectable, this ),
                                                                          boost::bind( &PointCloudBase::setSelectable, this, _1 ), parent_category_, this );
  setPropertyHelpText(selectable_property_, "Whether or not the points in this point cloud are selectable.");

  style_property_ = property_manager_->createProperty<EnumProperty>( "Style", property_prefix_, boost::bind( &PointCloudBase::getStyle, this ),
                                                                     boost::bind( &PointCloudBase::setStyle, this, _1 ), parent_category_, this );
  setPropertyHelpText(style_property_, "Rendering mode to use, in order of computational complexity.");
  EnumPropertyPtr enum_prop = style_property_.lock();
  enum_prop->addOption( "Points", Points );
  enum_prop->addOption( "Billboards", Billboards );
  enum_prop->addOption( "Billboard Spheres", BillboardSpheres );
  enum_prop->addOption( "Boxes", Boxes );

  billboard_size_property_ = property_manager_->createProperty<FloatProperty>( "Billboard Size", property_prefix_, boost::bind( &PointCloudBase::getBillboardSize, this ),
                                                                                boost::bind( &PointCloudBase::setBillboardSize, this, _1 ), parent_category_, this );
  setPropertyHelpText(billboard_size_property_, "Length, in meters, of the side of each billboard (or face if using the Boxes style).");
  FloatPropertyPtr float_prop = billboard_size_property_.lock();
  float_prop->setMin( 0.0001 );

  setPropertyHelpText(color_transformer_property_, "Set the transformer to use to set the color of the points.");
  enum_prop = color_transformer_property_.lock();

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &PointCloudBase::getAlpha, this ),
                                                                          boost::bind( &PointCloudBase::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the points.  Note that this is experimental and does not always look correct.");
  decay_time_property_ = property_manager_->createProperty<FloatProperty>( "Decay Time", property_prefix_, boost::bind( &PointCloudBase::getDecayTime, this ),
                                                                           boost::bind( &PointCloudBase::setDecayTime, this, _1 ), parent_category_, this );
  setPropertyHelpText(decay_time_property_, "Duration, in seconds, to keep the incoming points.  0 means only show the latest points.");

  xyz_transformer_property_ = property_manager_->createProperty<EnumProperty>( "Position Transformer", property_prefix_, boost::bind( &PointCloudBase::getXYZTransformer, this ),
                                                                     boost::bind( &PointCloudBase::setXYZTransformer, this, _1 ), parent_category_, this );
  setPropertyHelpText(xyz_transformer_property_, "Set the transformer to use to set the position of the points.");
  enum_prop = xyz_transformer_property_.lock();

  color_transformer_property_ = property_manager_->createProperty<EnumProperty>( "Color Transformer", property_prefix_, boost::bind( &PointCloudBase::getColorTransformer, this ),
                                                                     boost::bind( &PointCloudBase::setColorTransformer, this, _1 ), parent_category_, this );

  if (getXYZTransformer() != -1)
  {
    transformers_[getXYZTransformer()]->createProperties(property_manager_, parent_category_, property_prefix_, PointCloudTransformer::Support_XYZ, xyz_props_);
  }

  if (getColorTransformer() != -1)
  {
    transformers_[getColorTransformer()]->createProperties(property_manager_, parent_category_, property_prefix_, PointCloudTransformer::Support_Color, color_props_);
  }
}

void PointCloudBase::reset()
{
  Display::reset();

  clouds_.clear();
  cloud_->clear();
  messages_received_ = 0;
  total_point_count_ = 0;
}

} // namespace rviz
