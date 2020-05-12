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

#include <QColor>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include <ros/time.h>

#include <pluginlib/class_loader.hpp>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/validate_floats.h>

#include <rviz/default_plugin/point_cloud_common.h>

namespace rviz
{
struct IndexAndMessage
{
  IndexAndMessage(int _index, const void* _message) : index(_index), message((uint64_t)_message)
  {
  }

  int index;
  uint64_t message;
};

uint qHash(IndexAndMessage iam)
{
  return ((uint)iam.index) + ((uint)(iam.message >> 32)) + ((uint)(iam.message & 0xffffffff));
}

bool operator==(IndexAndMessage a, IndexAndMessage b)
{
  return a.index == b.index && a.message == b.message;
}

PointCloudSelectionHandler::PointCloudSelectionHandler(float box_size,
                                                       PointCloudCommon::CloudInfo* cloud_info,
                                                       DisplayContext* context)
  : SelectionHandler(context), cloud_info_(cloud_info), box_size_(box_size)
{
}

PointCloudSelectionHandler::~PointCloudSelectionHandler()
{
  // delete all the Property objects on our way out.
  QHash<IndexAndMessage, Property*>::const_iterator iter;
  for (iter = property_hash_.begin(); iter != property_hash_.end(); iter++)
  {
    delete iter.value();
  }
}

void PointCloudSelectionHandler::preRenderPass(uint32_t pass)
{
  SelectionHandler::preRenderPass(pass);

  switch (pass)
  {
  case 0:
    cloud_info_->cloud_->setPickColor(SelectionManager::handleToColor(getHandle()));
    break;
  case 1:
    cloud_info_->cloud_->setColorByIndex(true);
    break;
  default:
    break;
  }
}

void PointCloudSelectionHandler::postRenderPass(uint32_t pass)
{
  SelectionHandler::postRenderPass(pass);

  if (pass == 1)
  {
    cloud_info_->cloud_->setColorByIndex(false);
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

void PointCloudSelectionHandler::createProperties(const Picked& obj, Property* parent_property)
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
      int index = *it;
      const sensor_msgs::PointCloud2ConstPtr& message = cloud_info_->message_;

      IndexAndMessage hash_key(index, message.get());
      if (!property_hash_.contains(hash_key))
      {
        Property* cat =
            new Property(QString("Point %1 [cloud 0x%2]").arg(index).arg((uint64_t)message.get()),
                         QVariant(), "", parent_property);
        property_hash_.insert(hash_key, cat);

        // First add the position.
        VectorProperty* pos_prop =
            new VectorProperty("Position", cloud_info_->transformed_points_[index].position, "", cat);
        pos_prop->setReadOnly(true);

        // Then add all other fields as well.
        for (size_t field = 0; field < message->fields.size(); ++field)
        {
          const sensor_msgs::PointField& f = message->fields[field];
          const std::string& name = f.name;

          if (name == "x" || name == "y" || name == "z" || name == "X" || name == "Y" || name == "Z")
          {
            continue;
          }
          if (name == "rgb" || name == "rgba")
          {
            float float_val =
                valueFromCloud<float>(message, f.offset, f.datatype, message->point_step, index);
            // Convertion hack because rgb are stored int float (datatype=7) and valueFromCloud can't
            // cast float to uint32_t
            uint32_t val = *((uint32_t*)&float_val);
            ColorProperty* prop =
                new ColorProperty(QString("%1: %2").arg(field).arg(QString::fromStdString(name)),
                                  QColor((val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff), "", cat);
            prop->setReadOnly(true);

            FloatProperty* aprop = new FloatProperty(QString("alpha"), ((val >> 24) / 255.0), "", cat);
            aprop->setReadOnly(true);
          }
          else
          {
            float val = valueFromCloud<float>(message, f.offset, f.datatype, message->point_step, index);
            FloatProperty* prop = new FloatProperty(
                QString("%1: %2").arg(field).arg(QString::fromStdString(name)), val, "", cat);
            prop->setReadOnly(true);
          }
        }
      }
    }
  }
}

void PointCloudSelectionHandler::destroyProperties(const Picked& obj, Property* /*parent_property*/)
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
      int index = *it;
      const sensor_msgs::PointCloud2ConstPtr& message = cloud_info_->message_;

      IndexAndMessage hash_key(index, message.get());

      Property* prop = property_hash_.take(hash_key);
      delete prop;
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
    int index = (*it & 0xffffffff) - 1;

    sensor_msgs::PointCloud2ConstPtr message = cloud_info_->message_;

    Ogre::Vector3 pos = cloud_info_->transformed_points_[index].position;
    pos = cloud_info_->scene_node_->convertLocalToWorldPosition(pos);

    float size = box_size_ * 0.5f;

    Ogre::AxisAlignedBox aabb(pos - size, pos + size);

    createBox(std::make_pair(obj.handle, index), aabb, "RVIZ/Cyan");
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

PointCloudCommon::CloudInfo::CloudInfo() : manager_(nullptr), scene_node_(nullptr)
{
}

PointCloudCommon::CloudInfo::~CloudInfo()
{
  clear();
}

void PointCloudCommon::CloudInfo::clear()
{
  if (scene_node_)
  {
    manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

PointCloudCommon::PointCloudCommon(Display* display)
  : auto_size_(false)
  , spinner_(1, &cbqueue_)
  , new_xyz_transformer_(false)
  , new_color_transformer_(false)
  , needs_retransform_(false)
  , transformer_class_loader_(nullptr)
  , display_(display)
{
  selectable_property_ =
      new BoolProperty("Selectable", true,
                       "Whether or not the points in this point cloud are selectable.", display_,
                       SLOT(updateSelectable()), this);

  style_property_ = new EnumProperty("Style", "Flat Squares",
                                     "Rendering mode to use, in order of computational complexity.",
                                     display_, SLOT(updateStyle()), this);
  style_property_->addOption("Points", PointCloud::RM_POINTS);
  style_property_->addOption("Squares", PointCloud::RM_SQUARES);
  style_property_->addOption("Flat Squares", PointCloud::RM_FLAT_SQUARES);
  style_property_->addOption("Spheres", PointCloud::RM_SPHERES);
  style_property_->addOption("Boxes", PointCloud::RM_BOXES);

  point_world_size_property_ = new FloatProperty("Size (m)", 0.01, "Point size in meters.", display_,
                                                 SLOT(updateBillboardSize()), this);
  point_world_size_property_->setMin(0.0001);

  point_pixel_size_property_ = new FloatProperty("Size (Pixels)", 3, "Point size in pixels.", display_,
                                                 SLOT(updateBillboardSize()), this);
  point_pixel_size_property_->setMin(1);

  alpha_property_ = new FloatProperty("Alpha", 1.0,
                                      "Amount of transparency to apply to the points. "
                                      "Note that this is experimental and does not always look correct.",
                                      display_, SLOT(updateAlpha()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  decay_time_property_ = new FloatProperty(
      "Decay Time", 0,
      "Duration, in seconds, to keep the incoming points.  0 means only show the latest points.",
      display_, SLOT(queueRender()));
  decay_time_property_->setMin(0);

  xyz_transformer_property_ =
      new EnumProperty("Position Transformer", "",
                       "Set the transformer to use to set the position of the points.", display_,
                       SLOT(updateXyzTransformer()), this);
  connect(xyz_transformer_property_, SIGNAL(requestOptions(EnumProperty*)), this,
          SLOT(setXyzTransformerOptions(EnumProperty*)));

  color_transformer_property_ =
      new EnumProperty("Color Transformer", "",
                       "Set the transformer to use to set the color of the points.", display_,
                       SLOT(updateColorTransformer()), this);
  connect(color_transformer_property_, SIGNAL(requestOptions(EnumProperty*)), this,
          SLOT(setColorTransformerOptions(EnumProperty*)));
}

void PointCloudCommon::initialize(DisplayContext* context, Ogre::SceneNode* scene_node)
{
  transformer_class_loader_ =
      new pluginlib::ClassLoader<PointCloudTransformer>("rviz", "rviz::PointCloudTransformer");
  loadTransformers();

  context_ = context;
  scene_node_ = scene_node;

  updateStyle();
  updateBillboardSize();
  updateAlpha();
  updateSelectable();

  spinner_.start();
}

PointCloudCommon::~PointCloudCommon()
{
  spinner_.stop();

  if (transformer_class_loader_)
  {
    delete transformer_class_loader_;
  }
}

void PointCloudCommon::loadTransformers()
{
  std::vector<std::string> classes = transformer_class_loader_->getDeclaredClasses();
  std::vector<std::string>::iterator ci;

  for (ci = classes.begin(); ci != classes.end(); ci++)
  {
    const std::string& lookup_name = *ci;
    std::string name = transformer_class_loader_->getName(lookup_name);

    if (transformers_.count(name) > 0)
    {
      ROS_ERROR("Transformer type [%s] is already loaded.", name.c_str());
      continue;
    }

    PointCloudTransformerPtr trans(transformer_class_loader_->createUnmanagedInstance(lookup_name));
    trans->init();
    connect(trans.get(), SIGNAL(needRetransform()), this, SLOT(causeRetransform()));

    TransformerInfo info;
    info.transformer = trans;
    info.readable_name = name;
    info.lookup_name = lookup_name;

    info.transformer->createProperties(display_, PointCloudTransformer::Support_XYZ, info.xyz_props);
    setPropertiesHidden(info.xyz_props, true);

    info.transformer->createProperties(display_, PointCloudTransformer::Support_Color, info.color_props);
    setPropertiesHidden(info.color_props, true);

    transformers_[name] = info;
  }
}

void PointCloudCommon::setAutoSize(bool auto_size)
{
  auto_size_ = auto_size;
  for (unsigned i = 0; i < cloud_infos_.size(); i++)
  {
    cloud_infos_[i]->cloud_->setAutoSize(auto_size);
  }
}


void PointCloudCommon::updateAlpha()
{
  for (unsigned i = 0; i < cloud_infos_.size(); i++)
  {
    bool per_point_alpha = findChannelIndex(cloud_infos_[i]->message_, "rgba") != -1;
    cloud_infos_[i]->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
  }
}

void PointCloudCommon::updateSelectable()
{
  bool selectable = selectable_property_->getBool();

  if (selectable)
  {
    for (unsigned i = 0; i < cloud_infos_.size(); i++)
    {
      cloud_infos_[i]->selection_handler_.reset(
          new PointCloudSelectionHandler(getSelectionBoxSize(), cloud_infos_[i].get(), context_));
      cloud_infos_[i]->cloud_->setPickColor(
          SelectionManager::handleToColor(cloud_infos_[i]->selection_handler_->getHandle()));
    }
  }
  else
  {
    for (unsigned i = 0; i < cloud_infos_.size(); i++)
    {
      cloud_infos_[i]->selection_handler_.reset();
      cloud_infos_[i]->cloud_->setPickColor(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
    }
  }
}

void PointCloudCommon::updateStyle()
{
  PointCloud::RenderMode mode = (PointCloud::RenderMode)style_property_->getOptionInt();
  if (mode == PointCloud::RM_POINTS)
  {
    point_world_size_property_->hide();
    point_pixel_size_property_->show();
  }
  else
  {
    point_world_size_property_->show();
    point_pixel_size_property_->hide();
  }
  for (unsigned int i = 0; i < cloud_infos_.size(); i++)
  {
    cloud_infos_[i]->cloud_->setRenderMode(mode);
  }
  updateBillboardSize();
}

void PointCloudCommon::updateBillboardSize()
{
  PointCloud::RenderMode mode = (PointCloud::RenderMode)style_property_->getOptionInt();
  float size;
  if (mode == PointCloud::RM_POINTS)
  {
    size = point_pixel_size_property_->getFloat();
  }
  else
  {
    size = point_world_size_property_->getFloat();
  }
  for (unsigned i = 0; i < cloud_infos_.size(); i++)
  {
    cloud_infos_[i]->cloud_->setDimensions(size, size, size);
    cloud_infos_[i]->selection_handler_->setBoxSize(getSelectionBoxSize());
  }
  context_->queueRender();
}

void PointCloudCommon::reset()
{
  boost::mutex::scoped_lock lock(new_clouds_mutex_);
  cloud_infos_.clear();
  new_cloud_infos_.clear();
}

void PointCloudCommon::causeRetransform()
{
  needs_retransform_ = true;
}

void PointCloudCommon::update(float /*wall_dt*/, float /*ros_dt*/)
{
  PointCloud::RenderMode mode = (PointCloud::RenderMode)style_property_->getOptionInt();

  float point_decay_time = decay_time_property_->getFloat();
  if (needs_retransform_)
  {
    retransform();
    needs_retransform_ = false;
  }

  // instead of deleting cloud infos, we just clear them
  // and put them into obsolete_cloud_infos, so active selections
  // are preserved

  auto now_sec = ros::Time::now().toSec();

  // if decay time == 0, clear the old cloud when we get a new one
  // otherwise, clear all the outdated ones
  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);
    if (point_decay_time > 0.0 || !new_cloud_infos_.empty())
    {
      while (!cloud_infos_.empty() &&
             now_sec - cloud_infos_.front()->receive_time_.toSec() >= point_decay_time)
      {
        cloud_infos_.front()->clear();
        obsolete_cloud_infos_.push_back(cloud_infos_.front());
        cloud_infos_.pop_front();
        context_->queueRender();
      }
    }
  }

  // garbage-collect old point clouds that don't have an active selection
  L_CloudInfo::iterator it = obsolete_cloud_infos_.begin();
  L_CloudInfo::iterator end = obsolete_cloud_infos_.end();
  for (; it != end; ++it)
  {
    if (!(*it)->selection_handler_.get() || !(*it)->selection_handler_->hasSelections())
    {
      it = obsolete_cloud_infos_.erase(it);
    }
  }

  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);
    if (!new_cloud_infos_.empty())
    {
      float size;
      if (mode == PointCloud::RM_POINTS)
      {
        size = point_pixel_size_property_->getFloat();
      }
      else
      {
        size = point_world_size_property_->getFloat();
      }

      V_CloudInfo::iterator it = new_cloud_infos_.begin();
      V_CloudInfo::iterator end = new_cloud_infos_.end();
      for (; it != end; ++it)
      {
        CloudInfoPtr cloud_info = *it;

        V_CloudInfo::iterator next = it;
        next++;
        // ignore point clouds that are too old, but keep at least one
        if (next != end && now_sec - cloud_info->receive_time_.toSec() >= point_decay_time)
        {
          continue;
        }

        bool per_point_alpha = findChannelIndex(cloud_info->message_, "rgba") != -1;

        cloud_info->cloud_.reset(new PointCloud());
        cloud_info->cloud_->setRenderMode(mode);
        cloud_info->cloud_->addPoints(&(cloud_info->transformed_points_.front()),
                                      cloud_info->transformed_points_.size());
        cloud_info->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
        cloud_info->cloud_->setDimensions(size, size, size);
        cloud_info->cloud_->setAutoSize(auto_size_);

        cloud_info->manager_ = context_->getSceneManager();

        cloud_info->scene_node_ =
            scene_node_->createChildSceneNode(cloud_info->position_, cloud_info->orientation_);

        cloud_info->scene_node_->attachObject(cloud_info->cloud_.get());

        cloud_info->selection_handler_.reset(
            new PointCloudSelectionHandler(getSelectionBoxSize(), cloud_info.get(), context_));

        cloud_infos_.push_back(*it);
      }

      new_cloud_infos_.clear();
    }
  }

  {
    boost::recursive_mutex::scoped_try_lock lock(transformers_mutex_);

    if (lock.owns_lock())
    {
      if (new_xyz_transformer_ || new_color_transformer_)
      {
        M_TransformerInfo::iterator it = transformers_.begin();
        M_TransformerInfo::iterator end = transformers_.end();
        for (; it != end; ++it)
        {
          const std::string& name = it->first;
          TransformerInfo& info = it->second;

          setPropertiesHidden(info.xyz_props, name != xyz_transformer_property_->getStdString());
          setPropertiesHidden(info.color_props, name != color_transformer_property_->getStdString());
        }
      }
    }

    new_xyz_transformer_ = false;
    new_color_transformer_ = false;
  }

  updateStatus();
}

void PointCloudCommon::setPropertiesHidden(const QList<Property*>& props, bool hide)
{
  for (int i = 0; i < props.size(); i++)
  {
    props[i]->setHidden(hide);
  }
}

void PointCloudCommon::updateTransformers(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  std::string xyz_name = xyz_transformer_property_->getStdString();
  std::string color_name = color_transformer_property_->getStdString();

  xyz_transformer_property_->clearOptions();
  color_transformer_property_->clearOptions();

  // Get the channels that we could potentially render
  typedef std::set<std::pair<uint8_t, std::string> > S_string;
  S_string valid_xyz, valid_color;
  bool cur_xyz_valid = false;
  bool cur_color_valid = false;
  bool has_rgb_transformer = false;
  M_TransformerInfo::iterator trans_it = transformers_.begin();
  M_TransformerInfo::iterator trans_end = transformers_.end();
  for (; trans_it != trans_end; ++trans_it)
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
      xyz_transformer_property_->addOptionStd(name);
    }

    if (mask & PointCloudTransformer::Support_Color)
    {
      valid_color.insert(std::make_pair(trans->score(cloud), name));
      if (name == color_name)
      {
        cur_color_valid = true;
      }
      if (name == "RGB8")
      {
        has_rgb_transformer = true;
      }
      color_transformer_property_->addOptionStd(name);
    }
  }

  if (!cur_xyz_valid)
  {
    if (!valid_xyz.empty())
    {
      xyz_transformer_property_->setStringStd(valid_xyz.rbegin()->second);
    }
  }

  if (!cur_color_valid)
  {
    if (!valid_color.empty())
    {
      if (has_rgb_transformer)
      {
        color_transformer_property_->setStringStd("RGB8");
      }
      else
      {
        color_transformer_property_->setStringStd(valid_color.rbegin()->second);
      }
    }
  }
}

void PointCloudCommon::updateStatus()
{
  std::stringstream ss;
  // ss << "Showing [" << total_point_count_ << "] points from [" << clouds_.size() << "] messages";
  display_->setStatusStd(StatusProperty::Ok, "Points", ss.str());
}

void PointCloudCommon::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  CloudInfoPtr info(new CloudInfo);
  info->message_ = cloud;
  info->receive_time_ = ros::Time::now();

  if (transformCloud(info, true))
  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);
    new_cloud_infos_.push_back(info);
    display_->emitTimeSignal(cloud->header.stamp);
  }
}

void PointCloudCommon::updateXyzTransformer()
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (transformers_.count(xyz_transformer_property_->getStdString()) == 0)
  {
    return;
  }
  new_xyz_transformer_ = true;
  causeRetransform();
}

void PointCloudCommon::updateColorTransformer()
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  if (transformers_.count(color_transformer_property_->getStdString()) == 0)
  {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

PointCloudTransformerPtr
PointCloudCommon::getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  M_TransformerInfo::iterator it = transformers_.find(xyz_transformer_property_->getStdString());
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

PointCloudTransformerPtr
PointCloudCommon::getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
  M_TransformerInfo::iterator it = transformers_.find(color_transformer_property_->getStdString());
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


void PointCloudCommon::retransform()
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

  D_CloudInfo::iterator it = cloud_infos_.begin();
  D_CloudInfo::iterator end = cloud_infos_.end();
  for (; it != end; ++it)
  {
    const CloudInfoPtr& cloud_info = *it;
    transformCloud(cloud_info, false);
    cloud_info->cloud_->clear();
    cloud_info->cloud_->addPoints(&cloud_info->transformed_points_.front(),
                                  cloud_info->transformed_points_.size());
  }
}

bool PointCloudCommon::transformCloud(const CloudInfoPtr& cloud_info, bool update_transformers)
{
  if (!cloud_info->scene_node_)
  {
    if (!context_->getFrameManager()->getTransform(cloud_info->message_->header, cloud_info->position_,
                                                   cloud_info->orientation_))
    {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << cloud_info->message_->header.frame_id << "] to frame ["
         << context_->getFrameManager()->getFixedFrame() << "]";
      display_->setStatusStd(StatusProperty::Error, "Message", ss.str());
      return false;
    }
  }

  Ogre::Matrix4 transform;
  transform.makeTransform(cloud_info->position_, Ogre::Vector3(1, 1, 1), cloud_info->orientation_);

  V_PointCloudPoint& cloud_points = cloud_info->transformed_points_;
  cloud_points.clear();

  size_t size = cloud_info->message_->width * cloud_info->message_->height;
  PointCloud::Point default_pt;
  default_pt.color = Ogre::ColourValue(1, 1, 1);
  default_pt.position = Ogre::Vector3::ZERO;
  cloud_points.resize(size, default_pt);

  {
    boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
    if (update_transformers)
    {
      updateTransformers(cloud_info->message_);
    }
    PointCloudTransformerPtr xyz_trans = getXYZTransformer(cloud_info->message_);
    PointCloudTransformerPtr color_trans = getColorTransformer(cloud_info->message_);

    if (!xyz_trans)
    {
      std::stringstream ss;
      ss << "No position transformer available for cloud";
      display_->setStatusStd(StatusProperty::Error, "Message", ss.str());
      return false;
    }

    if (!color_trans)
    {
      std::stringstream ss;
      ss << "No color transformer available for cloud";
      display_->setStatusStd(StatusProperty::Error, "Message", ss.str());
      return false;
    }

    xyz_trans->transform(cloud_info->message_, PointCloudTransformer::Support_XYZ, transform,
                         cloud_points);
    color_trans->transform(cloud_info->message_, PointCloudTransformer::Support_Color, transform,
                           cloud_points);
  }

  for (V_PointCloudPoint::iterator cloud_point = cloud_points.begin(); cloud_point != cloud_points.end();
       ++cloud_point)
  {
    if (!validateFloats(cloud_point->position))
    {
      cloud_point->position.x = 999999.0f;
      cloud_point->position.y = 999999.0f;
      cloud_point->position.z = 999999.0f;
    }
  }

  return true;
}

bool convertPointCloudToPointCloud2(const sensor_msgs::PointCloud& input,
                                    sensor_msgs::PointCloud2& output)
{
  output.header = input.header;
  output.width = input.points.size();
  output.height = 1;
  output.fields.resize(3 + input.channels.size());
  // Convert x/y/z to fields
  output.fields[0].name = "x";
  output.fields[1].name = "y";
  output.fields[2].name = "z";
  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < output.fields.size(); ++d, offset += 4)
  {
    output.fields[d].offset = offset;
    output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }
  output.point_step = offset;
  output.row_step = output.point_step * output.width;
  // Convert the remaining of the channels to fields
  for (size_t d = 0; d < input.channels.size(); ++d)
    output.fields[3 + d].name = input.channels[d].name;
  output.data.resize(input.points.size() * output.point_step);
  output.is_bigendian = false; // @todo ?
  output.is_dense = false;

  // Copy the data points
  for (size_t cp = 0; cp < input.points.size(); ++cp)
  {
    memcpy(&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x,
           sizeof(float));
    memcpy(&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y,
           sizeof(float));
    memcpy(&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z,
           sizeof(float));
    for (size_t d = 0; d < input.channels.size(); ++d)
    {
      if (input.channels[d].values.size() == input.points.size())
      {
        memcpy(&output.data[cp * output.point_step + output.fields[3 + d].offset],
               &input.channels[d].values[cp], sizeof(float));
      }
    }
  }
  return (true);
}

void PointCloudCommon::addMessage(const sensor_msgs::PointCloudConstPtr& cloud)
{
  sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
  convertPointCloudToPointCloud2(*cloud, *out);
  addMessage(out);
}

void PointCloudCommon::addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  processMessage(cloud);
}

void PointCloudCommon::fixedFrameChanged()
{
  reset();
}

void PointCloudCommon::setXyzTransformerOptions(EnumProperty* prop)
{
  fillTransformerOptions(prop, PointCloudTransformer::Support_XYZ);
}

void PointCloudCommon::setColorTransformerOptions(EnumProperty* prop)
{
  fillTransformerOptions(prop, PointCloudTransformer::Support_Color);
}

void PointCloudCommon::fillTransformerOptions(EnumProperty* prop, uint32_t mask)
{
  prop->clearOptions();

  if (cloud_infos_.empty())
  {
    return;
  }

  boost::recursive_mutex::scoped_lock tlock(transformers_mutex_);

  const sensor_msgs::PointCloud2ConstPtr& msg = cloud_infos_.front()->message_;

  M_TransformerInfo::iterator it = transformers_.begin();
  M_TransformerInfo::iterator end = transformers_.end();
  for (; it != end; ++it)
  {
    const PointCloudTransformerPtr& trans = it->second.transformer;
    if ((trans->supports(msg) & mask) == mask)
    {
      prop->addOption(QString::fromStdString(it->first));
    }
  }
}

float PointCloudCommon::getSelectionBoxSize()
{
  if (style_property_->getOptionInt() != PointCloud::RM_POINTS)
  {
    return point_world_size_property_->getFloat();
  }
  else
  {
    return 0.004;
  }
}

} // namespace rviz
