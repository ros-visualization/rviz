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

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreWireBoundingBox.h>

#include <ros/time.h>

#include <tf/transform_listener.h>

#include <pluginlib/class_loader.h>

#include "rviz/default_plugin/point_cloud_transformer.h"
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/display.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/validate_floats.h"

#include "rviz/default_plugin/point_cloud_common.h"

namespace rviz
{

template<typename T>
T getValue(const T& val)
{
  return val;
}

struct IndexAndMessage
{
  IndexAndMessage( int _index, const void* _message )
    : index( _index )
    , message( (uint64_t) _message )
    {}

  int index;
  uint64_t message;
};

uint qHash( IndexAndMessage iam )
{
  return
    ((uint) iam.index) +
    ((uint) (iam.message >> 32)) +
    ((uint) (iam.message & 0xffffffff));
}

bool operator==( IndexAndMessage a, IndexAndMessage b )
{
  return a.index == b.index && a.message == b.message;
}

class PointCloudSelectionHandler: public SelectionHandler
{
public:
  PointCloudSelectionHandler(PointCloudCommon* display);
  virtual ~PointCloudSelectionHandler();

  virtual void createProperties( const Picked& obj, Property* parent_property );
  virtual void destroyProperties( const Picked& obj, Property* parent_property );

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
  void getCloudAndLocalIndexByGlobalIndex(int global_index, PointCloudCommon::CloudInfoPtr& cloud_out, int& index_out);

  PointCloudCommon* display_;
  QHash<IndexAndMessage, Property*> property_hash_;
};

PointCloudSelectionHandler::PointCloudSelectionHandler(PointCloudCommon* display)
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

void PointCloudSelectionHandler::getCloudAndLocalIndexByGlobalIndex(int global_index, PointCloudCommon::CloudInfoPtr& cloud_out, int& index_out)
{
  boost::mutex::scoped_lock lock(display_->clouds_mutex_);

  int count = 0;

  PointCloudCommon::D_CloudInfo::iterator cloud_it = display_->clouds_.begin();
  PointCloudCommon::D_CloudInfo::iterator cloud_end = display_->clouds_.end();
  for (;cloud_it != cloud_end; ++cloud_it)
  {
    const PointCloudCommon::CloudInfoPtr& info = *cloud_it;

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

void PointCloudSelectionHandler::createProperties( const Picked& obj, Property* parent_property )
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
      PointCloudCommon::CloudInfoPtr cloud;

      getCloudAndLocalIndexByGlobalIndex(global_index, cloud, index);

      if (!cloud)
      {
        continue;
      }

      const sensor_msgs::PointCloud2ConstPtr& message = cloud->message_;

      IndexAndMessage hash_key( index, message.get() );
      if( !property_hash_.contains( hash_key ))
      {
        Property* cat = new Property( QString( "Point %1 [cloud 0x%2]" ).arg( index ).arg( (uint64_t) message.get() ),
                                      QVariant(), "", parent_property );
        property_hash_.insert( hash_key, cat );

        // First add the position.
        VectorProperty* pos_prop = new VectorProperty( "Position", cloud->transformed_points_[index].position, "", cat );
        pos_prop->setReadOnly( true );

        // Then add all other fields as well.
        for( size_t field = 0; field < message->fields.size(); ++field )
        {
          const sensor_msgs::PointField& f = message->fields[ field ];
          const std::string& name = f.name;

          if( name == "x" || name == "y" || name == "z" || name == "X" || name == "Y" || name == "Z" )
          {
            continue;
          }

          float val = valueFromCloud<float>( message, f.offset, f.datatype, message->point_step, index );

          FloatProperty* prop = new FloatProperty( QString( "%1: %2" ).arg( field ).arg( QString::fromStdString( name )),
                                                   val, "", cat );
          prop->setReadOnly( true );
        }
      }
    }
  }
}

void PointCloudSelectionHandler::destroyProperties( const Picked& obj, Property* parent_property )
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
      PointCloudCommon::CloudInfoPtr cloud;

      getCloudAndLocalIndexByGlobalIndex(global_index, cloud, index);

      if (!cloud)
      {
        continue;
      }

      const sensor_msgs::PointCloud2ConstPtr& message = cloud->message_;

      IndexAndMessage hash_key( index, message.get() );
      
      Property* prop = property_hash_.take( hash_key );
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
    int global_index = (*it & 0xffffffff) - 1;

    int index = 0;
    PointCloudCommon::CloudInfoPtr cloud;

    getCloudAndLocalIndexByGlobalIndex(global_index, cloud, index);

    if (!cloud)
    {
      continue;
    }

    sensor_msgs::PointCloud2ConstPtr message = cloud->message_;

    Ogre::Vector3 pos = cloud->transform_ * pointFromCloud(message, index);

    float size = display_->getSelectionBoxSize() * 0.5f;

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

PointCloudCommon::CloudInfo::CloudInfo()
: time_(0.0f)
, transform_(Ogre::Matrix4::ZERO)
, num_points_(0)
{}

PointCloudCommon::CloudInfo::~CloudInfo()
{
}

PointCloudCommon::PointCloudCommon( Display* display )
: spinner_(1, &cbqueue_)
, new_cloud_(false)
, new_xyz_transformer_(false)
, new_color_transformer_(false)
, needs_retransform_(false)
, coll_handle_(0)
, total_point_count_(0)
, transformer_class_loader_( new pluginlib::ClassLoader<PointCloudTransformer>( "rviz", "rviz::PointCloudTransformer" ))
, display_( display )
{
  cloud_ = new PointCloud();

  selectable_property_ = new BoolProperty( "Selectable", true,
                                           "Whether or not the points in this point cloud are selectable.",
                                           display_, SLOT( updateSelectable() ), this );

  style_property_ = new EnumProperty( "Style", "Billboards",
                                      "Rendering mode to use, in order of computational complexity.",
                                      display_, SLOT( updateStyle() ), this );
  style_property_->addOption( "Points", PointCloud::RM_POINTS );
  style_property_->addOption( "Billboards", PointCloud::RM_BILLBOARDS );
  style_property_->addOption( "Billboard Spheres", PointCloud::RM_BILLBOARD_SPHERES );
  style_property_->addOption( "Boxes", PointCloud::RM_BOXES );

  billboard_size_property_ = new FloatProperty( "Billboard Size", 0.01,
                                                "Length, in meters, of the side of each billboard (or face if using the Boxes style).",
                                                display_, SLOT( updateBillboardSize() ), this );
  billboard_size_property_->setMin( 0.0001 );

  alpha_property_ = new FloatProperty( "Alpha", 1.0,
                                       "Amount of transparency to apply to the points.  Note that this is experimental and does not always look correct.",
                                       display_, SLOT( updateAlpha() ), this );
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  decay_time_property_ = new FloatProperty( "Decay Time", 0,
                                            "Duration, in seconds, to keep the incoming points.  0 means only show the latest points.",
                                            display_ );
  decay_time_property_->setMin( 0 );

  xyz_transformer_property_ = new EnumProperty( "Position Transformer", "",
                                                "Set the transformer to use to set the position of the points.",
                                                display_, SLOT( updateXyzTransformer() ), this );
  connect( xyz_transformer_property_, SIGNAL( requestOptions( EnumProperty* )),
           this, SLOT( setXyzTransformerOptions( EnumProperty* )));

  color_transformer_property_ = new EnumProperty( "Color Transformer", "",
                                                  "Set the transformer to use to set the color of the points.",
                                                  display_, SLOT( updateColorTransformer() ), this );
  connect( color_transformer_property_, SIGNAL( requestOptions( EnumProperty* )),
           this, SLOT( setColorTransformerOptions( EnumProperty* )));

  loadTransformers();
}

void PointCloudCommon::initialize( DisplayContext* context, Ogre::SceneNode* scene_node )
{
  context_ = context;
  scene_node_ = scene_node;
  scene_node_->attachObject(cloud_);
  coll_handler_ = PointCloudSelectionHandlerPtr(new PointCloudSelectionHandler(this));

  updateStyle();
  updateBillboardSize();
  updateAlpha();
  updateSelectable();

  connect( decay_time_property_, SIGNAL( changed() ), context_, SLOT( queueRender() ));

  spinner_.start();
}

PointCloudCommon::~PointCloudCommon()
{
  spinner_.stop();

  if (coll_handle_)
  {
    SelectionManager* sel_manager = context_->getSelectionManager();
    sel_manager->removeObject(coll_handle_);
  }

  context_->getSceneManager()->destroySceneNode(scene_node_->getName());
  delete cloud_;
  delete transformer_class_loader_;
}

void PointCloudCommon::loadTransformers()
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

    PointCloudTransformerPtr trans( transformer_class_loader_->createUnmanagedInstance( lookup_name ));
    trans->init();
    connect( trans.get(), SIGNAL( needRetransform() ), this, SLOT( causeRetransform() ));

    TransformerInfo info;
    info.transformer = trans;
    info.readable_name = name;
    info.lookup_name = lookup_name;
    transformers_[ name ] = info;

    info.transformer->createProperties( display_, PointCloudTransformer::Support_XYZ, info.xyz_props );
    setPropertiesHidden( info.xyz_props, true );
    info.transformer->createProperties( display_, PointCloudTransformer::Support_Color, info.color_props );
    setPropertiesHidden( info.color_props, true );
  }
}

void PointCloudCommon::updateAlpha()
{
  cloud_->setAlpha( alpha_property_->getFloat() );
}

void PointCloudCommon::updateSelectable()
{
  bool selectable = selectable_property_->getBool();

  SelectionManager* sel_manager = context_->getSelectionManager();

  if( selectable )
  {
    coll_handle_ = sel_manager->createHandle();

    sel_manager->addObject( coll_handle_, coll_handler_ );

    // Break out coll handle into r/g/b/a floats
    float r = ((coll_handle_ >> 16) & 0xff) / 255.0f;
    float g = ((coll_handle_ >> 8) & 0xff) / 255.0f;
    float b = (coll_handle_ & 0xff) / 255.0f;
    Ogre::ColourValue col( r, g, b, 1.0f );
    cloud_->setPickColor( col );
  }
  else
  {
    sel_manager->removeObject( coll_handle_ );
    coll_handle_ = 0;
    cloud_->setPickColor( Ogre::ColourValue( 0.0f, 0.0f, 0.0f, 0.0f ));
  }
}

void PointCloudCommon::updateStyle()
{
  PointCloud::RenderMode mode = (PointCloud::RenderMode) style_property_->getOptionInt();
  if( mode == PointCloud::RM_POINTS )
  {
    billboard_size_property_->hide();
  }
  else
  {
    billboard_size_property_->show();
  }
  cloud_->setRenderMode( mode );
  context_->queueRender();
}

void PointCloudCommon::updateBillboardSize()
{
  float size = billboard_size_property_->getFloat();
  cloud_->setDimensions( size, size, size );
  context_->queueRender();
}

void PointCloudCommon::reset()
{
  clouds_.clear();
  cloud_->clear();
  total_point_count_ = 0;
}

void PointCloudCommon::causeRetransform()
{
  boost::mutex::scoped_lock lock(clouds_mutex_);
  needs_retransform_ = true;
}

void PointCloudCommon::update(float wall_dt, float ros_dt)
{
  float point_decay_time = decay_time_property_->getFloat();
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

    if( point_decay_time > 0.0f )
    {
      bool removed = false;
      uint32_t points_to_pop = 0;
      while( !clouds_.empty() && clouds_.front()->time_ > point_decay_time )
      {
        total_point_count_ -= clouds_.front()->num_points_;
        points_to_pop += clouds_.front()->num_points_;
        clouds_.pop_front();
        removed = true;
      }

      if (removed)
      {
        cloud_->popPoints(points_to_pop);
        context_->queueRender();
      }
    }
  }

  if( new_cloud_ )
  {
    boost::mutex::scoped_lock lock(new_clouds_mutex_);

    if( point_decay_time == 0.0f )
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

          setPropertiesHidden( info.xyz_props, name != xyz_transformer_property_->getStdString() );
          setPropertiesHidden( info.color_props, name != color_transformer_property_->getStdString() );
        }
      }
    }

    new_xyz_transformer_ = false;
    new_color_transformer_ = false;
  }

  updateStatus();
}

void PointCloudCommon::setPropertiesHidden( const QList<Property*>& props, bool hide )
{
  for( int i = 0; i < props.size(); i++ )
  {
    props[ i ]->setHidden( hide );
  }
}

void PointCloudCommon::updateTransformers( const sensor_msgs::PointCloud2ConstPtr& cloud )
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
      xyz_transformer_property_->addOptionStd( name );
    }

    if (mask & PointCloudTransformer::Support_Color)
    {
      valid_color.insert(std::make_pair(trans->score(cloud), name));
      if (name == color_name)
      {
        cur_color_valid = true;
      }
      color_transformer_property_->addOptionStd( name );
    }
  }

  if( !cur_xyz_valid )
  {
    if( !valid_xyz.empty() )
    {
      xyz_transformer_property_->setStringStd( valid_xyz.rbegin()->second );
    }
  }

  if( !cur_color_valid )
  {
    if( !valid_color.empty() )
    {
      color_transformer_property_->setStringStd( valid_color.rbegin()->second );
    }
  }
}

void PointCloudCommon::updateStatus()
{
  std::stringstream ss;
  ss << "Showing [" << total_point_count_ << "] points from [" << clouds_.size() << "] messages";
  display_->setStatusStd(StatusProperty::Ok, "Points", ss.str());
}

void PointCloudCommon::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
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

void PointCloudCommon::updateXyzTransformer()
{
  boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
  if( transformers_.count( xyz_transformer_property_->getStdString() ) == 0 )
  {
    return;
  }
  new_xyz_transformer_ = true;
  causeRetransform();
}

void PointCloudCommon::updateColorTransformer()
{
  boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
  if( transformers_.count( color_transformer_property_->getStdString() ) == 0 )
  {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

PointCloudTransformerPtr PointCloudCommon::getXYZTransformer( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
  boost::recursive_mutex::scoped_lock lock( transformers_mutex_);
  M_TransformerInfo::iterator it = transformers_.find( xyz_transformer_property_->getStdString() );
  if( it != transformers_.end() )
  {
    const PointCloudTransformerPtr& trans = it->second.transformer;
    if( trans->supports( cloud ) & PointCloudTransformer::Support_XYZ )
    {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

PointCloudTransformerPtr PointCloudCommon::getColorTransformer( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
  boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
  M_TransformerInfo::iterator it = transformers_.find( color_transformer_property_->getStdString() );
  if( it != transformers_.end() )
  {
    const PointCloudTransformerPtr& trans = it->second.transformer;
    if( trans->supports( cloud ) & PointCloudTransformer::Support_Color )
    {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

void PointCloudCommon::retransform()
{
  boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

  cloud_->clear();

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
}

bool PointCloudCommon::transformCloud(const CloudInfoPtr& info, V_Point& points, bool update_transformers)
{
  Ogre::Matrix4 transform = info->transform_;

  if (transform == Ogre::Matrix4::ZERO)
  {
    Ogre::Vector3 pos;
    Ogre::Quaternion orient;
    if (!context_->getFrameManager()->getTransform(info->message_->header, pos, orient))
    {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << info->message_->header.frame_id << "] to frame [" << context_->getFrameManager()->getFixedFrame() << "]";
      display_->setStatusStd(StatusProperty::Error, "Message", ss.str());
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
    if( update_transformers )
    {
      updateTransformers( info->message_ );
    }
    PointCloudTransformerPtr xyz_trans = getXYZTransformer(info->message_);
    PointCloudTransformerPtr color_trans = getColorTransformer(info->message_);

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

void PointCloudCommon::addMessage(const sensor_msgs::PointCloudConstPtr& cloud)
{
  sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
  convertPointCloudToPointCloud2(*cloud, *out);
  addMessage(out);
}

void PointCloudCommon::addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if (cloud->width * cloud->height == 0)
  {
    return;
  }

  processMessage(cloud);
}

void PointCloudCommon::fixedFrameChanged()
{
  reset();
}

void PointCloudCommon::setXyzTransformerOptions( EnumProperty* prop )
{
  fillTransformerOptions( prop, PointCloudTransformer::Support_XYZ );
}

void PointCloudCommon::setColorTransformerOptions( EnumProperty* prop )
{
  fillTransformerOptions( prop, PointCloudTransformer::Support_Color );
}

void PointCloudCommon::fillTransformerOptions( EnumProperty* prop, uint32_t mask )
{
  prop->clearOptions();

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
      prop->addOption( QString::fromStdString( it->first ));
    }
  }
}

float PointCloudCommon::getSelectionBoxSize()
{
  if( style_property_->getOptionInt() != PointCloud::RM_POINTS )
  {
    return billboard_size_property_->getFloat();
  }
  else
  {
    return 0.004;
  }
}

} // namespace rviz
