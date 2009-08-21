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
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

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
  void getCloudAndLocalIndexByGlobalIndex(int global_index, boost::shared_ptr<sensor_msgs::PointCloud>& cloud_out, int& index_out);

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

void PointCloudSelectionHandler::getCloudAndLocalIndexByGlobalIndex(int global_index, boost::shared_ptr<sensor_msgs::PointCloud>& cloud_out, int& index_out)
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

    count += info->message_->points.size();
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
      boost::shared_ptr<sensor_msgs::PointCloud> message;

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
      boost::shared_ptr<sensor_msgs::PointCloud> message;

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
    boost::shared_ptr<sensor_msgs::PointCloud> message;

    getCloudAndLocalIndexByGlobalIndex(global_index, message, index);

    if (!message)
    {
      continue;
    }

    Ogre::Vector3 pos(message->points[index].x, message->points[index].y, message->points[index].z);
    robotToOgre(pos);

    float size = 0.002;
    if (display_->style_ != PointCloudBase::Points)
    {
      size = display_->billboard_size_ / 2.0;
    }

    Ogre::AxisAlignedBox aabb(pos - size, pos + size);

    createBox(std::make_pair(obj.handle, global_index), aabb, "RVIZ/Cyan");
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


PointCloudBase::CloudInfo::CloudInfo(VisualizationManager* manager)
: time_(0.0f)
, num_points_(0)
, vis_manager_(manager)
{}

PointCloudBase::CloudInfo::~CloudInfo()
{
}

PointCloudBase::PointCloudBase( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, new_cloud_(false)
, min_color_( 0.0f, 0.0f, 0.0f )
, max_color_( 1.0f, 1.0f, 1.0f )
, min_intensity_(0.0f)
, max_intensity_(4096.0f)
, auto_compute_intensity_bounds_(true)
, intensity_bounds_changed_(false)
, style_( Billboards )
, channel_color_idx_( -1 )
, billboard_size_( 0.01 )
, point_decay_time_(0.0f)
, selectable_(false)
, coll_handle_(0)
{
  cloud_ = new ogre_tools::PointCloud();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->attachObject(cloud_);
  coll_handler_ = PointCloudSelectionHandlerPtr(new PointCloudSelectionHandler(this));

  setStyle( style_ );
  setBillboardSize( billboard_size_ );
  setChannelColorIndex ( channel_color_idx_ );
  setAlpha(1.0f);

  setSelectable(true);
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

void PointCloudBase::setMaxColor( const Color& color )
{
  max_color_ = color;

  propertyChanged(max_color_property_);

  causeRender();
}

void PointCloudBase::setMinColor( const Color& color )
{
  min_color_ = color;

  propertyChanged(min_color_property_);

  causeRender();
}

void PointCloudBase::setMinIntensity( float val )
{
  min_intensity_ = val;
  if (min_intensity_ > max_intensity_)
  {
    min_intensity_ = max_intensity_;
  }

  propertyChanged(min_intensity_property_);

  causeRender();
}

void PointCloudBase::setMaxIntensity( float val )
{
  max_intensity_ = val;
  if (max_intensity_ < min_intensity_)
  {
    max_intensity_ = min_intensity_;
  }

  propertyChanged(max_intensity_property_);

  causeRender();
}

void PointCloudBase::setDecayTime( float time )
{
  point_decay_time_ = time;

  propertyChanged(decay_time_property_);

  causeRender();
}

void PointCloudBase::setAutoComputeIntensityBounds(bool compute)
{
  auto_compute_intensity_bounds_ = compute;

  propertyChanged(auto_compute_intensity_bounds_property_);

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

  cloud_->setRenderMode(mode);

  propertyChanged(style_property_);

  causeRender();
}

/** \brief Set the channel color index. Called through the \a channel_property_ callback.
  * \param channel_color_idx the index of the channel to be rendered
  */
void PointCloudBase::setChannelColorIndex (int channel_color_idx)
{
  ROS_ASSERT (channel_color_idx < ChannelRenderCount);

  channel_color_idx_ = channel_color_idx;

  propertyChanged(channel_property_);
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
}

void PointCloudBase::update(float wall_dt, float ros_dt)
{
  if (intensity_bounds_changed_)
  {
    setMinIntensity(min_intensity_);
    setMaxIntensity(max_intensity_);
    intensity_bounds_changed_ = false;
  }

  {
    boost::mutex::scoped_lock lock(clouds_mutex_);

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
      while (!clouds_.empty() && clouds_.front()->time_ > point_decay_time_)
      {
        cloud_->popPoints(clouds_.front()->num_points_);
        clouds_.pop_front();
        removed = true;
      }

      if (removed)
      {
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
    }
    else
    {
      {
        VV_Point::iterator it = new_points_.begin();
        VV_Point::iterator end = new_points_.end();
        for (; it != end; ++it)
        {
          V_Point& points = *it;
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

    if (!clouds_.empty())
    {
      const boost::shared_ptr<sensor_msgs::PointCloud>& cloud = clouds_.front()->message_;

      // Get the channels that we could potentially render
      int channel_color_idx = getChannelColorIndex ();

      EnumPropertyPtr channel_prop = channel_property_.lock();

      if (channel_prop)
      {
        channel_prop->clear ();
      }

      typedef std::set<int32_t> S_int32;
      S_int32 valid_chans;
      typedef std::vector<sensor_msgs::ChannelFloat32> V_Chan;
      V_Chan::iterator chan_it = cloud->channels.begin();
      V_Chan::iterator chan_end = cloud->channels.end();
      uint32_t index = 0;
      for ( ; chan_it != chan_end; ++chan_it, ++index )
      {
        sensor_msgs::ChannelFloat32& chan = *chan_it;
        if (chan.name == "intensity" || chan.name == "intensities")
        {
          valid_chans.insert(Intensity);

          if (channel_prop)
          {
            channel_prop->addOption ("Intensity", Intensity);
          }
        }
        else if (chan.name == "rgb" || chan.name == "r")
        {
          valid_chans.insert(ColorRGBSpace);

          if (channel_prop)
          {
            channel_prop->addOption ("Color (RGB)", ColorRGBSpace);
          }
        }
        else if (chan.name == "nx")
        {
          valid_chans.insert(NormalSphere);

          if (channel_prop)
          {
            channel_prop->addOption ("Normal Sphere", NormalSphere);
          }
        }
        else if (chan.name == "curvature" || chan.name == "curvatures")
        {
          valid_chans.insert(Curvature);

          if (channel_prop)
          {
            channel_prop->addOption ("Curvature", Curvature);
          }
        }
      }

      if (channel_color_idx == -1 || valid_chans.find(channel_color_idx) == valid_chans.end())
      {
        channel_color_idx = -1;
        if (!valid_chans.empty())
        {
          channel_color_idx = *valid_chans.begin();
        }
      }

      if ( channel_prop )
      {
        channel_prop->changed();
      }

      setChannelColorIndex (channel_color_idx);
    }
  }
}

void transformIntensity( float val, Color& color, const Color& min_color, const Color& max_color, float min_intensity, float max_intensity, float diff_intensity )
{
  float normalized_intensity = diff_intensity > 0.0f ? ( val - min_intensity ) / diff_intensity : 1.0f;
  normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
  color.r_ = max_color.r_*normalized_intensity + min_color.r_*(1.0f - normalized_intensity);
  color.g_ = max_color.g_*normalized_intensity + min_color.g_*(1.0f - normalized_intensity);
  color.b_ = max_color.b_*normalized_intensity + min_color.b_*(1.0f - normalized_intensity);
}

void transformRGB( float val, Color& color, const Color&, const Color&, float, float, float )
{
  int rgb = *reinterpret_cast<int*>(&val);
  color.r_ = ((rgb >> 16) & 0xff) / 255.0f;
  color.g_ = ((rgb >> 8) & 0xff) / 255.0f;
  color.b_ = (rgb & 0xff) / 255.0f;
}

void transformR( float val, Color& color, const Color&, const Color&, float, float, float )
{
  color.r_ = val;
}

void transformG( float val, Color& color, const Color&, const Color&, float, float, float )
{
  color.g_ = val;
}

void transformB( float val, Color& color, const Color&, const Color&, float, float, float )
{
  color.b_ = val;
}

void PointCloudBase::processMessage(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  CloudInfoPtr info(new CloudInfo(vis_manager_));
  info->message_ = sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud(*cloud));
  info->time_ = 0;

  V_Point points;
  transformCloud(info, points);

  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);

    new_clouds_.push_back(info);
    new_points_.push_back(V_Point());
    new_points_.back().swap(points);

    new_cloud_ = true;
  }
}

void PointCloudBase::transformCloud(const CloudInfoPtr& info, V_Point& points)
{
  const boost::shared_ptr<sensor_msgs::PointCloud>& cloud = info->message_;

  std::string frame_id = cloud->header.frame_id;
  if ( frame_id.empty() )
  {
    frame_id = fixed_frame_;
  }

  try
  {
    vis_manager_->getThreadedTFClient()->transformPointCloud( fixed_frame_, *cloud, *cloud );
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming point cloud '%s' from frame '%s' to frame '%s'\n", name_.c_str(), frame_id.c_str(), fixed_frame_.c_str() );
  }

  typedef std::vector<sensor_msgs::ChannelFloat32> V_Chan;
  typedef std::vector<bool> V_bool;

  V_bool valid_channels(cloud->channels.size());
  uint32_t point_count = cloud->get_points_size();
  V_Chan::iterator chan_it = cloud->channels.begin();
  V_Chan::iterator chan_end = cloud->channels.end();
  uint32_t index = 0;

  info->num_points_ = point_count;

  bool use_normals_as_coordinates = false;

  for ( ; chan_it != chan_end; ++chan_it, ++index )
  {
    sensor_msgs::ChannelFloat32& chan = *chan_it;
    uint32_t val_count = chan.values.size();
    bool channel_size_correct = val_count == point_count;
    ROS_ERROR_COND(!channel_size_correct, "Point cloud '%s' has channel with fewer values than points (%d values, %d points)", name_.c_str(), val_count, point_count);

    valid_channels[index] = channel_size_correct;

    // Check for intensities
    if ( auto_compute_intensity_bounds_ && channel_size_correct && ( chan.name == "intensity" || chan.name == "intensities" ) &&
         channel_color_idx_ == Intensity )
    {
      min_intensity_ = 999999.0f;
      max_intensity_ = -999999.0f;
      for(uint32_t i = 0; i < point_count; i++)
      {
        float& intensity = chan.values[i];
        // arbitrarily cap to 4096 for now
        intensity = std::min( intensity, 4096.0f );
        min_intensity_ = std::min( min_intensity_, intensity );
        max_intensity_ = std::max( max_intensity_, intensity );
      }

      intensity_bounds_changed_ = true;
    }

    // Check for curvatures
    else if ( auto_compute_intensity_bounds_ && channel_size_correct && ( chan.name == "curvature" || chan.name == "curvatures" ) &&
              channel_color_idx_ == Curvature )
    {
      min_intensity_ = 999999.0f;
      max_intensity_ = -999999.0f;
      for(uint32_t i = 0; i < point_count; i++)
      {
        float& intensity = chan.values[i];
        // arbitrarily cap to 4096 for now
        intensity = std::min( intensity, 4096.0f );
        min_intensity_ = std::min( min_intensity_, intensity );
        max_intensity_ = std::max( max_intensity_, intensity );
      }

      intensity_bounds_changed_ = true;
    }
    else if ( chan.name == "nx" && channel_color_idx_ == NormalSphere )
    {
      use_normals_as_coordinates = true;
    }
  }

  // Look for point normals
  int nx_idx = getROSCloudChannelIndex (cloud, std::string ("nx"));
  int ny_idx = getROSCloudChannelIndex (cloud, std::string ("ny"));
  int nz_idx = getROSCloudChannelIndex (cloud, std::string ("nz"));
  if ( use_normals_as_coordinates && (ny_idx == -1 || nz_idx == -1) )
  {
    ROS_WARN ("Normal information requested via 'nx', but 'ny' and 'nz' channels are not present!");
    use_normals_as_coordinates = false;
  }

  float diff_intensity = max_intensity_ - min_intensity_;

  points.resize( point_count );
  for(uint32_t i = 0; i < point_count; i++)
  {
    ogre_tools::PointCloud::Point& current_point = points[ i ];

    if (use_normals_as_coordinates && nx_idx != -1 && ny_idx != -1 && nz_idx != -1)
    {
      current_point.x = cloud->channels[nx_idx].values[i];
      current_point.y = cloud->channels[ny_idx].values[i];
      current_point.z = cloud->channels[nz_idx].values[i];
    }
    else          // Use normal 3D x-y-z coordinates
    {
      current_point.x = cloud->points[i].x;
      current_point.y = cloud->points[i].y;
      current_point.z = cloud->points[i].z;
    }

    Ogre::Vector3 position( current_point.x, current_point.y, current_point.z );
    robotToOgre( position );
    current_point.x = position.x;
    current_point.y = position.y;
    current_point.z = position.z;

    current_point.color = 0;
  }

  index = 0;
  enum ChannelType
  {
    CT_INTENSITY,
    CT_RGB,
    CT_R,
    CT_G,
    CT_B,

    CT_COUNT
  };
  ChannelType type = CT_INTENSITY;
  typedef void (*TransformFunc)(float, Color&, const Color&, const Color&, float, float, float);
  TransformFunc funcs[CT_COUNT] =
  {
    transformIntensity,
    transformRGB,
    transformR,
    transformG,
    transformB
  };

  Ogre::Root* root = Ogre::Root::getSingletonPtr();
  chan_it = cloud->channels.begin();
  for ( ; chan_it != chan_end; ++chan_it, ++index )
  {
    if ( !valid_channels[index] )
    {
      continue;
    }

    sensor_msgs::ChannelFloat32& chan = *chan_it;

    if ( chan.name == "intensity" || chan.name == "intensities" || chan.name == "curvatures" || chan.name == "curvature" )
    {
      type = CT_INTENSITY;
    }
    else if ( chan.name == "rgb" )
    {
      type = CT_RGB;
    }
    else if ( chan.name == "r" )
    {
      type = CT_R;
    }
    else if ( chan.name == "g" )
    {
      type = CT_G;
    }
    else if ( chan.name == "b" )
    {
      type = CT_B;
    }
    else
    {
      continue;
    }

    // Color all points
    if ( ( channel_color_idx_ == Intensity && (chan.name == "intensity" || chan.name == "intensities") ) ||
           ( channel_color_idx_ == Curvature && (chan.name == "curvature" || chan.name == "curvatures") ) ||
           ( channel_color_idx_ == ColorRGBSpace && (chan.name == "rgb" || chan.name == "r" || chan.name == "g" || chan.name == "b") )
         )
    {
      for (uint32_t i = 0; i < point_count; i++)
      {
        ogre_tools::PointCloud::Point& current_point = points[ i ];

        Color c;
        c.r_ = 0.0f;
        c.g_ = 0.0f;
        c.b_ = 0.0f;
        funcs[type]( chan.values[i], c, min_color_, max_color_, min_intensity_, max_intensity_, diff_intensity );
        uint32_t color;
        root->convertColourValue(Ogre::ColourValue(c.r_, c.g_, c.b_), &color);
        current_point.color |= color;
      }
    }
  }
}

void PointCloudBase::addMessage(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  if (cloud->points.empty())
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

  style_property_ = property_manager_->createProperty<EnumProperty>( "Style", property_prefix_, boost::bind( &PointCloudBase::getStyle, this ),
                                                                     boost::bind( &PointCloudBase::setStyle, this, _1 ), parent_category_, this );
  EnumPropertyPtr enum_prop = style_property_.lock();
  enum_prop->addOption( "Points", Points );
  enum_prop->addOption( "Billboards", Billboards );
  enum_prop->addOption( "Billboard Spheres", BillboardSpheres );
  enum_prop->addOption( "Boxes", Boxes );

  channel_property_ = property_manager_->createProperty<EnumProperty>( "Channel", property_prefix_, boost::bind( &PointCloudBase::getChannelColorIndex, this ),
                                                                     boost::bind( &PointCloudBase::setChannelColorIndex, this, _1 ), parent_category_, this );

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &PointCloudBase::getAlpha, this ),
                                                                        boost::bind( &PointCloudBase::setAlpha, this, _1 ), parent_category_, this );

  min_color_property_ = property_manager_->createProperty<ColorProperty>( "Min Color", property_prefix_, boost::bind( &PointCloudBase::getMinColor, this ),
                                                                            boost::bind( &PointCloudBase::setMinColor, this, _1 ), parent_category_, this );
  max_color_property_ = property_manager_->createProperty<ColorProperty>( "Max Color", property_prefix_, boost::bind( &PointCloudBase::getMaxColor, this ),
                                                                        boost::bind( &PointCloudBase::setMaxColor, this, _1 ), parent_category_, this );
  ColorPropertyPtr color_prop = max_color_property_.lock();
  // legacy "Color" support... convert it to max color
  color_prop->addLegacyName("Color");

  billboard_size_property_ = property_manager_->createProperty<FloatProperty>( "Billboard Size", property_prefix_, boost::bind( &PointCloudBase::getBillboardSize, this ),
                                                                                boost::bind( &PointCloudBase::setBillboardSize, this, _1 ), parent_category_, this );
  FloatPropertyPtr float_prop = billboard_size_property_.lock();
  float_prop->setMin( 0.0001 );

  auto_compute_intensity_bounds_property_ = property_manager_->createProperty<BoolProperty>( "Autocompute Intensity Bounds", property_prefix_, boost::bind( &PointCloudBase::getAutoComputeIntensityBounds, this ),
                                                                            boost::bind( &PointCloudBase::setAutoComputeIntensityBounds, this, _1 ), parent_category_, this );
  min_intensity_property_ = property_manager_->createProperty<FloatProperty>( "Min Intensity", property_prefix_, boost::bind( &PointCloudBase::getMinIntensity, this ),
                                                                            boost::bind( &PointCloudBase::setMinIntensity, this, _1 ), parent_category_, this );
  max_intensity_property_ = property_manager_->createProperty<FloatProperty>( "Max Intensity", property_prefix_, boost::bind( &PointCloudBase::getMaxIntensity, this ),
                                                                          boost::bind( &PointCloudBase::setMaxIntensity, this, _1 ), parent_category_, this );

  decay_time_property_ = property_manager_->createProperty<FloatProperty>( "Decay Time", property_prefix_, boost::bind( &PointCloudBase::getDecayTime, this ),
                                                                           boost::bind( &PointCloudBase::setDecayTime, this, _1 ), parent_category_, this );
}

void PointCloudBase::reset()
{
  clouds_.clear();
  cloud_->clear();
}

} // namespace rviz
