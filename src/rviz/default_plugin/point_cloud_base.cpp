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

#include <pluginlib/class_loader.h>

#include "point_cloud_base.h"
#include "point_cloud_transformer.h"
#include "point_cloud_transformers.h"
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/frame_manager.h"
#include "rviz/uniform_string_stream.h"

#include <ros/time.h>
#include "rviz/ogre_helpers/point_cloud.h"

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
  void getCloudAndLocalIndexByGlobalIndex(int global_index, PointCloudBase::CloudInfoPtr& cloud_out, int& index_out);

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

void PointCloudSelectionHandler::getCloudAndLocalIndexByGlobalIndex(int global_index, PointCloudBase::CloudInfoPtr& cloud_out, int& index_out)
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
      cloud_out = info;

      return;
    }

    count += info->message_->width * info->message_->height;
  }
}

Ogre::Vector3 pointFromCloud(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t index)
{
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint8_t type = cloud->fields[xi].datatype;
  const uint32_t point_step = cloud->point_step;
  float x = valueFromCloud<float>(cloud, xoff, type, point_step, index);
  float y = valueFromCloud<float>(cloud, yoff, type, point_step, index);
  float z = valueFromCloud<float>(cloud, zoff, type, point_step, index);
  return Ogre::Vector3(x, y, z);
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
      PointCloudBase::CloudInfoPtr cloud;

      getCloudAndLocalIndexByGlobalIndex(global_index, cloud, index);

      if (!cloud)
      {
        continue;
      }

      const sensor_msgs::PointCloud2ConstPtr& message = cloud->message_;

      UniformStringStream prefix;
      prefix << "Point " << index << " [cloud " << message.get() << "]";

      if (property_manager->hasProperty(prefix.str(), ""))
      {
        continue;
      }

      CategoryPropertyWPtr cat = property_manager->createCategory(prefix.str(), "");

      // Do xyz first, from the transformed xyz
      {
        UniformStringStream ss;
        ss << "Position";
        Ogre::Vector3 pos(cloud->transformed_points_[index].position);
        property_manager->createProperty<Vector3Property>(ss.str(), prefix.str(), boost::bind(getValue<Ogre::Vector3>, pos), Vector3Property::Setter(), cat);
      }

      for (size_t field = 0; field < message->fields.size(); ++field)
      {
        const sensor_msgs::PointField& f = message->fields[field];
        const std::string& name = f.name;

        if (name == "x" || name == "y" || name == "z" || name == "X" || name == "Y" || name == "Z")
        {
          continue;
        }

        float val = valueFromCloud<float>(message, f.offset, f.datatype, message->point_step, index);

        UniformStringStream ss;
        ss << field << ": " << name;
        property_manager->createProperty<FloatProperty>(ss.str(), prefix.str(), boost::bind(getValue<float>, val), FloatProperty::Setter(), cat);
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
      PointCloudBase::CloudInfoPtr cloud;

      getCloudAndLocalIndexByGlobalIndex(global_index, cloud, index);

      if (!cloud)
      {
        continue;
      }

      const sensor_msgs::PointCloud2ConstPtr& message = cloud->message_;

      UniformStringStream prefix;
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
    PointCloudBase::CloudInfoPtr cloud;

    getCloudAndLocalIndexByGlobalIndex(global_index, cloud, index);

    if (!cloud)
    {
      continue;
    }

    sensor_msgs::PointCloud2ConstPtr message = cloud->message_;

    Ogre::Vector3 pos = cloud->transform_ * pointFromCloud(message, index);

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

PointCloudBase::CloudInfo::CloudInfo()
: time_(0.0f)
, transform_(Ogre::Matrix4::ZERO)
, num_points_(0)
{}

PointCloudBase::CloudInfo::~CloudInfo()
{
}

PointCloudBase::PointCloudBase()
: Display()
, spinner_(1, &cbqueue_)
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
, transformer_class_loader_( new pluginlib::ClassLoader<PointCloudTransformer>( "rviz", "rviz::PointCloudTransformer" ))
, hidden_(false)
{
  cloud_ = new PointCloud();
}

void PointCloudBase::onInitialize()
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->attachObject(cloud_);
  coll_handler_ = PointCloudSelectionHandlerPtr(new PointCloudSelectionHandler(this));

  setStyle( style_ );
  setBillboardSize( billboard_size_ );
  setAlpha(1.0f);

  setSelectable(true);

  loadTransformers();

  threaded_nh_.setCallbackQueue(&cbqueue_);
  spinner_.start();
}

void PointCloudBase::hideVisible()
{
  if (!hidden_)
  {
    hidden_ = true;
    scene_manager_->getRootSceneNode()->removeChild(scene_node_);
    scene_node_->detachObject(cloud_);
  }
}

void PointCloudBase::restoreVisible()
{
  if (hidden_)
  {
    hidden_ = false;
    scene_manager_->getRootSceneNode()->addChild(scene_node_);
    scene_node_->attachObject(cloud_);
  }
}

void deleteProperties(PropertyManager* man, V_PropertyBaseWPtr& props)
{
  V_PropertyBaseWPtr::iterator prop_it = props.begin();
  V_PropertyBaseWPtr::iterator prop_end = props.end();
  for (; prop_it != prop_end; ++prop_it)
  {
    man->deleteProperty(prop_it->lock());
  }

  props.clear();
}

PointCloudBase::~PointCloudBase()
{
  spinner_.stop();

  if (coll_handle_)
  {
    SelectionManager* sel_manager = vis_manager_->getSelectionManager();
    sel_manager->removeObject(coll_handle_);
  }

  scene_manager_->destroySceneNode(scene_node_->getName());
  delete cloud_;

  if (property_manager_)
  {
    M_TransformerInfo::iterator it = transformers_.begin();
    M_TransformerInfo::iterator end = transformers_.end();
    for (; it != end; ++it)
    {
      deleteProperties( property_manager_, it->second.xyz_props );
      deleteProperties( property_manager_, it->second.color_props );
    }
  }

  delete transformer_class_loader_;
}

void PointCloudBase::loadTransformers()
{
  std::vector<std::string> classes = transformer_class_loader_->getDeclaredClasses();
  std::vector<std::string>::iterator ci;
  
  for( ci = classes.begin(); ci != classes.end(); ci++ )
  {
    const std::string& lookup_name = *ci;
    std::string name = transformer_class_loader_->getName( lookup_name );

    if( transformers_.count( name ) > 0 )
    {
      ROS_ERROR( "Transformer type [%s] is already loaded.", name.c_str() );
      continue;
    }

    PointCloudTransformerPtr trans( transformer_class_loader_->createClassInstance( lookup_name, true ));
    trans->init( boost::bind( &PointCloudBase::causeRetransform, this ));
    TransformerInfo info;
    info.transformer = trans;
    info.readable_name = name;
    info.lookup_name = lookup_name;
    transformers_[ name ] = info;

    if( property_manager_ )
    {
      info.transformer->createProperties( property_manager_, parent_category_,
                                          property_prefix_ + "." + name,
                                          PointCloudTransformer::Support_XYZ, info.xyz_props );
      info.transformer->createProperties( property_manager_, parent_category_,
                                          property_prefix_ + "." + name,
                                          PointCloudTransformer::Support_Color, info.color_props );
    }
  }
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

  PointCloud::RenderMode mode = PointCloud::RM_POINTS;
  if (style == Billboards)
  {
    mode = PointCloud::RM_BILLBOARDS;
  }
  else if (style == BillboardSpheres)
  {
    mode = PointCloud::RM_BILLBOARD_SPHERES;
  }
  else if (style == Boxes)
  {
    mode = PointCloud::RM_BOXES;
  }

  if (style == Points)
  {
    hideProperty( billboard_size_property_ );
  }
  else
  {
    showProperty( billboard_size_property_ );
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
    boost::recursive_mutex::scoped_try_lock lock( transformers_mutex_ );

    if( lock.owns_lock() )
    {
      if( new_xyz_transformer_ || new_color_transformer_ )
      {
        M_TransformerInfo::iterator it = transformers_.begin();
        M_TransformerInfo::iterator end = transformers_.end();
        for (; it != end; ++it)
        {
          const std::string& name = it->first;
          TransformerInfo& info = it->second;

          if( name == getXYZTransformer() )
          {
            std::for_each( info.xyz_props.begin(), info.xyz_props.end(), showProperty<PropertyBase> );
          }
          else
          {
            std::for_each( info.xyz_props.begin(), info.xyz_props.end(), hideProperty<PropertyBase> );
          }

          if( name == getColorTransformer() )
          {
            std::for_each( info.color_props.begin(), info.color_props.end(), showProperty<PropertyBase> );
          }
          else
          {
            std::for_each( info.color_props.begin(), info.color_props.end(), hideProperty<PropertyBase> );
          }
        }
      }
    }

    new_xyz_transformer_ = false;
    new_color_transformer_ = false;
  }
  updateStatus();
}

void PointCloudBase::updateTransformers(const sensor_msgs::PointCloud2ConstPtr& cloud, bool fully_update)
{
  EditEnumPropertyPtr xyz_prop = xyz_transformer_property_.lock();
  if (xyz_prop)
  {
    xyz_prop->clear();
  }

  EditEnumPropertyPtr color_prop = color_transformer_property_.lock();
  if (color_prop)
  {
    color_prop->clear();
  }

  // Get the channels that we could potentially render
  std::string xyz_name = getXYZTransformer();
  std::string color_name = getColorTransformer();

  typedef std::set<std::pair<uint8_t, std::string> > S_string;
  S_string valid_xyz, valid_color;
  bool cur_xyz_valid = false;
  bool cur_color_valid = false;
  M_TransformerInfo::iterator trans_it = transformers_.begin();
  M_TransformerInfo::iterator trans_end = transformers_.end();
  for(;trans_it != trans_end; ++trans_it)
  {
    const std::string& name = trans_it->first;
    const PointCloudTransformerPtr& trans = trans_it->second.transformer;
    uint32_t mask = trans->supports(cloud);
    if (mask & PointCloudTransformer::Support_XYZ)
    {
      valid_xyz.insert(std::make_pair(trans->score(cloud), name));
      if (name == xyz_name)
      {
        cur_xyz_valid = true;
      }

      if (xyz_prop)
      {
        xyz_prop->addOption(name);
      }
    }

    if (mask & PointCloudTransformer::Support_Color)
    {
      valid_color.insert(std::make_pair(trans->score(cloud), name));

      if (name == color_name)
      {
        cur_color_valid = true;
      }

      if (color_prop)
      {
        color_prop->addOption(name);
      }
    }
  }

  if (!cur_xyz_valid)
  {
    if (!valid_xyz.empty())
    {
      if (fully_update)
      {
        setXYZTransformer(valid_xyz.rbegin()->second);
      }
      else
      {
        xyz_transformer_ = valid_xyz.rbegin()->second;
      }
    }
  }

  if (!cur_color_valid)
  {
    if (!valid_color.empty())
    {
      if (fully_update)
      {
        setColorTransformer(valid_color.rbegin()->second);
      }
      else
      {
        color_transformer_ = valid_color.rbegin()->second;
      }
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

void PointCloudBase::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  CloudInfoPtr info(new CloudInfo);
  info->message_ = cloud;
  info->time_ = 0;

  V_Point points;
  if (transformCloud(info, points, true))
  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);

    new_clouds_.push_back(info);
    new_points_.push_back(V_Point());
    new_points_.back().swap(points);

    new_cloud_ = true;
  }
}

void PointCloudBase::setXYZTransformer(const std::string& name)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (xyz_transformer_ == name)
  {
    return;
  }

  if (transformers_.count(name) == 0)
  {
    return;
  }

  xyz_transformer_ = name;
  new_xyz_transformer_ = true;
  propertyChanged(xyz_transformer_property_);

  causeRetransform();
}

void PointCloudBase::setColorTransformer(const std::string& name)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (color_transformer_ == name)
  {
    return;
  }

  if (transformers_.count(name) == 0)
  {
    return;
  }

  color_transformer_ = name;
  new_color_transformer_ = true;
  propertyChanged(color_transformer_property_);

  causeRetransform();
}

PointCloudTransformerPtr PointCloudBase::getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  M_TransformerInfo::iterator it = transformers_.find(xyz_transformer_);
  if (it != transformers_.end())
  {
    const PointCloudTransformerPtr& trans = it->second.transformer;
    if (trans->supports(cloud) & PointCloudTransformer::Support_XYZ)
    {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

PointCloudTransformerPtr PointCloudBase::getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  M_TransformerInfo::iterator it = transformers_.find(color_transformer_);
  if (it != transformers_.end())
  {
    const PointCloudTransformerPtr& trans = it->second.transformer;
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

  // transformCloud can change the transformers, store them off so we can reset them afterwards
  std::string xyz_trans = xyz_transformer_;
  std::string color_trans = color_transformer_;

  D_CloudInfo::iterator it = clouds_.begin();
  D_CloudInfo::iterator end = clouds_.end();
  for (; it != end; ++it)
  {
    const CloudInfoPtr& cloud = *it;
    V_Point points;
    transformCloud(cloud, points, false);
    if (!points.empty())
    {
      cloud_->addPoints(&points.front(), points.size());
    }
  }

  xyz_transformer_ = xyz_trans;
  color_transformer_ = color_trans;
}

bool PointCloudBase::transformCloud(const CloudInfoPtr& info, V_Point& points, bool fully_update_transformers)
{
  Ogre::Matrix4 transform = info->transform_;

  if (transform == Ogre::Matrix4::ZERO)
  {
    Ogre::Vector3 pos;
    Ogre::Quaternion orient;
    if (!vis_manager_->getFrameManager()->getTransform(info->message_->header, pos, orient))
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

  V_PointCloudPoint& cloud_points = info->transformed_points_;
  cloud_points.clear();

  size_t size = info->message_->width * info->message_->height;
  info->num_points_ = size;
  PointCloudPoint default_pt;
  default_pt.color = Ogre::ColourValue(1, 1, 1);
  default_pt.position = Ogre::Vector3::ZERO;
  cloud_points.resize(size, default_pt);

  {
    boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
    updateTransformers(info->message_, fully_update_transformers);
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

    xyz_trans->transform(info->message_, PointCloudTransformer::Support_XYZ, transform, cloud_points);
    color_trans->transform(info->message_, PointCloudTransformer::Support_Color, transform, cloud_points);
  }

  points.resize(size);
  for (size_t i = 0; i < size; ++i)
  {
    Ogre::Vector3 pos = cloud_points[i].position;
    Ogre::ColourValue color = cloud_points[i].color;
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
    {
      if (input.channels[d].values.size() == input.points.size())
      {
        memcpy (&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof (float));
      }
    }
  }
  return (true);
}

void PointCloudBase::addMessage(const sensor_msgs::PointCloudConstPtr& cloud)
{
  sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
  convertPointCloudToPointCloud2(*cloud, *out);
  addMessage(out);
}

void PointCloudBase::addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
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

void PointCloudBase::onTransformerOptions(V_string& ops, uint32_t mask)
{
  boost::mutex::scoped_lock clock(clouds_mutex_);

  if (clouds_.empty())
  {
    return;
  }

  boost::recursive_mutex::scoped_lock tlock(transformers_mutex_);

  const sensor_msgs::PointCloud2ConstPtr& msg = clouds_.front()->message_;

  M_TransformerInfo::iterator it = transformers_.begin();
  M_TransformerInfo::iterator end = transformers_.end();
  for (; it != end; ++it)
  {
    const PointCloudTransformerPtr& trans = it->second.transformer;
    if ((trans->supports(msg) & mask) == mask)
    {
      ops.push_back(it->first);
    }
  }
}

void PointCloudBase::createProperties()
{
  selectable_property_ = property_manager_->createProperty<BoolProperty>( "Selectable", property_prefix_,
                                                                          boost::bind( &PointCloudBase::getSelectable, this ),
                                                                          boost::bind( &PointCloudBase::setSelectable, this, _1 ),
                                                                          parent_category_, this );
  setPropertyHelpText( selectable_property_, "Whether or not the points in this point cloud are selectable." );

  style_property_ = property_manager_->createProperty<EnumProperty>( "Style", property_prefix_,
                                                                     boost::bind( &PointCloudBase::getStyle, this ),
                                                                     boost::bind( &PointCloudBase::setStyle, this, _1 ),
                                                                     parent_category_, this );
  setPropertyHelpText( style_property_, "Rendering mode to use, in order of computational complexity." );
  EnumPropertyPtr enum_prop = style_property_.lock();
  enum_prop->addOption( "Points", Points );
  enum_prop->addOption( "Billboards", Billboards );
  enum_prop->addOption( "Billboard Spheres", BillboardSpheres );
  enum_prop->addOption( "Boxes", Boxes );

  billboard_size_property_ = property_manager_->createProperty<FloatProperty>( "Billboard Size", property_prefix_,
                                                                               boost::bind( &PointCloudBase::getBillboardSize, this ),
                                                                               boost::bind( &PointCloudBase::setBillboardSize, this, _1 ),
                                                                               parent_category_, this );
  setPropertyHelpText( billboard_size_property_, "Length, in meters, of the side of each billboard (or face if using the Boxes style)." );
  FloatPropertyPtr float_prop = billboard_size_property_.lock();
  float_prop->setMin( 0.0001 );

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_,
                                                                      boost::bind( &PointCloudBase::getAlpha, this ),
                                                                      boost::bind( &PointCloudBase::setAlpha, this, _1 ),
                                                                      parent_category_, this );
  setPropertyHelpText( alpha_property_,
                       "Amount of transparency to apply to the points.  Note that this is experimental and does not always look correct." );
  decay_time_property_ = property_manager_->createProperty<FloatProperty>( "Decay Time", property_prefix_,
                                                                           boost::bind( &PointCloudBase::getDecayTime, this ),
                                                                           boost::bind( &PointCloudBase::setDecayTime, this, _1 ),
                                                                           parent_category_, this );
  setPropertyHelpText( decay_time_property_, "Duration, in seconds, to keep the incoming points.  0 means only show the latest points." );

  xyz_transformer_property_ =
    property_manager_->createProperty<EditEnumProperty>( "Position Transformer", property_prefix_,
                                                         boost::bind( &PointCloudBase::getXYZTransformer, this ),
                                                         boost::bind( &PointCloudBase::setXYZTransformer, this, _1 ),
                                                         parent_category_, this );
  setPropertyHelpText( xyz_transformer_property_, "Set the transformer to use to set the position of the points." );
  EditEnumPropertyPtr edit_enum_prop = xyz_transformer_property_.lock();
  edit_enum_prop->setOptionCallback( boost::bind( &PointCloudBase::onTransformerOptions, this, _1, PointCloudTransformer::Support_XYZ ));

  color_transformer_property_ =
    property_manager_->createProperty<EditEnumProperty>( "Color Transformer", property_prefix_,
                                                         boost::bind( &PointCloudBase::getColorTransformer, this ),
                                                         boost::bind( &PointCloudBase::setColorTransformer, this, _1 ),
                                                         parent_category_, this );
  setPropertyHelpText( color_transformer_property_, "Set the transformer to use to set the color of the points." );
  edit_enum_prop = color_transformer_property_.lock();
  edit_enum_prop->setOptionCallback( boost::bind( &PointCloudBase::onTransformerOptions, this, _1, PointCloudTransformer::Support_Color ));

  // Create properties for transformers
  {
    boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
    M_TransformerInfo::iterator it = transformers_.begin();
    M_TransformerInfo::iterator end = transformers_.end();
    for( ; it != end; ++it )
    {
      const std::string& name = it->first;
      TransformerInfo& info = it->second;
      info.transformer->createProperties( property_manager_, parent_category_,
                                          property_prefix_ + "." + name,
                                          PointCloudTransformer::Support_XYZ, info.xyz_props );
      info.transformer->createProperties( property_manager_, parent_category_,
                                          property_prefix_ + "." + name,
                                          PointCloudTransformer::Support_Color, info.color_props );

      if( name != getXYZTransformer() )
      {
        std::for_each( info.xyz_props.begin(), info.xyz_props.end(), hideProperty<PropertyBase> );
      }

      if( name != getColorTransformer() )
      {
        std::for_each( info.color_props.begin(), info.color_props.end(), hideProperty<PropertyBase> );
      }
    }
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
