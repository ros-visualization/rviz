/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
#include <QObject>

#include "depth_cloud_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "image_transport/camera_common.h"
#include <image_transport/subscriber_plugin.h>

#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <string>

namespace enc = sensor_msgs::image_encodings;

namespace rviz
{

DepthCloudDisplay::DepthCloudDisplay()
  : rviz::Display()
  , messages_received_(0)
  , depthmap_it_(threaded_nh_)
  , depthmap_sub_()
  , rgb_it_ (threaded_nh_)
  , rgb_sub_()
  , cameraInfo_sub_()
  , queue_size_(5)
{

  // Depth map properties
  QRegExp depth_filter("depth");
  depth_filter.setCaseSensitivity(Qt::CaseInsensitive);

  topic_filter_property_ = new Property("Topic Filter",
                                          true,
                                          "List only topics with names that relate to depth and color images",
                                          this,
                                          SLOT (updateTopicFilter() ));

  depth_topic_property_ = new RosFilteredTopicProperty("Depth Map Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                         "sensor_msgs::Image topic to subscribe to.", depth_filter, this, SLOT( updateTopic() ));

  depth_transport_property_ = new EnumProperty("Depth Map Transport Hint", "raw", "Preferred method of sending images.", this,
                                         SLOT( updateTopic() ));

  connect(depth_transport_property_, SIGNAL( requestOptions( EnumProperty* )), this,
          SLOT( fillTransportOptionList( EnumProperty* )));

  depth_transport_property_->setStdString("raw");

  // color image properties
  QRegExp color_filter("color|rgb|bgr|gray|mono");
  color_filter.setCaseSensitivity(Qt::CaseInsensitive);

  color_topic_property_ = new RosFilteredTopicProperty("Color Image Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                         "sensor_msgs::Image topic to subscribe to.", color_filter, this, SLOT( updateTopic() ));

  color_transport_property_ = new EnumProperty("Color Transport Hint", "raw", "Preferred method of sending images.", this,
                                         SLOT( updateTopic() ));


  connect(color_transport_property_, SIGNAL( requestOptions( EnumProperty* )), this,
          SLOT( fillTransportOptionList( EnumProperty* )));

  color_transport_property_->setStdString("raw");

  // Queue size property
  queue_size_property_ = new IntProperty( "Queue Size", queue_size_,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 1 );

  use_auto_size_property_ = new BoolProperty( "Auto Size", true,
                                           "Automatically scale each point based on its depth value and the camera parameters.",
                                           this, SLOT( updateUseAutoSize() ), this );

  auto_size_factor_property_ = new FloatProperty( "Auto Size Factor", 1,
                                                "Scaling factor to be applied to the auto size.",
                                                this, SLOT( updateAutoSizeFactor() ), this );
  auto_size_factor_property_->setMin( 0.0001 );

  // Instantiate PointCloudCommon class for displaying point clouds
  pointcloud_common_ = new PointCloudCommon(this);

  updateUseAutoSize();

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  threaded_nh_.setCallbackQueue( pointcloud_common_->getCallbackQueue() );

  // Scan for available transport plugins
  scanForTransportSubscriberPlugins();
}

void DepthCloudDisplay::onInitialize()
{
  pointcloud_common_->initialize(context_, scene_node_);
  pointcloud_common_->xyz_transformer_property_->hide();
}

DepthCloudDisplay::~DepthCloudDisplay()
{

  unsubscribe();

  if (pointcloud_common_)
    delete pointcloud_common_;
}

void DepthCloudDisplay::updateQueueSize()
{
  queue_size_ = queue_size_property_->getInt();
}

void DepthCloudDisplay::updateUseAutoSize()
{
  bool use_auto_size = use_auto_size_property_->getBool();
  pointcloud_common_->point_world_size_property_->setReadOnly( use_auto_size );
  pointcloud_common_->setAutoSize( use_auto_size );
  auto_size_factor_property_->setHidden(!use_auto_size);
}

void DepthCloudDisplay::updateAutoSizeFactor()
{
}

void DepthCloudDisplay::updateTopicFilter()
{
  bool enabled = topic_filter_property_->getValue().toBool();
  depth_topic_property_->enableFilter(enabled);
  color_topic_property_->enableFilter(enabled);
}

void DepthCloudDisplay::onEnable()
{
  subscribe();
}

void DepthCloudDisplay::onDisable()
{
  unsubscribe();

  clear();
}

void DepthCloudDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    // reset all message filters
    sync_depth_color_.reset(new synchronizer_depth_color_(sync_policy_depth_color_(queue_size_)));
    depthmap_tf_filter_.reset();
    depthmap_sub_.reset(new image_transport::SubscriberFilter());
    rgb_sub_.reset(new image_transport::SubscriberFilter());
    cameraInfo_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

    std::string depthmap_topic = depth_topic_property_->getTopicStd();
    std::string color_topic = color_topic_property_->getTopicStd();

    std::string depthmap_transport = depth_transport_property_->getStdString();
    std::string color_transport = color_transport_property_->getStdString();

    if (!depthmap_topic.empty() && !depthmap_transport.empty()) {
      // subscribe to depth map topic
      depthmap_sub_->subscribe(depthmap_it_, depthmap_topic, queue_size_,  image_transport::TransportHints(depthmap_transport));

      depthmap_tf_filter_.reset(
          new tf::MessageFilter<sensor_msgs::Image>(*depthmap_sub_, *context_->getTFClient(), fixed_frame_.toStdString(), queue_size_, threaded_nh_));

      // subscribe to CameraInfo  topic
      std::string info_topic = image_transport::getCameraInfoTopic(depthmap_topic);
      cameraInfo_sub_->subscribe(threaded_nh_, info_topic, queue_size_);
      cameraInfo_sub_->registerCallback(boost::bind(&DepthCloudDisplay::caminfoCallback, this, _1));

      if (!color_topic.empty() && !color_transport.empty()) {
        // subscribe to color image topic
        rgb_sub_->subscribe(rgb_it_, color_topic, queue_size_,  image_transport::TransportHints(color_transport));

        // connect message filters to synchronizer
        sync_depth_color_->connectInput(*depthmap_tf_filter_, *rgb_sub_);
        sync_depth_color_->setInterMessageLowerBound(0, ros::Duration(0.5));
        sync_depth_color_->setInterMessageLowerBound(1, ros::Duration(0.5));
        sync_depth_color_->registerCallback(boost::bind(&DepthCloudDisplay::processMessage, this, _1, _2));

        pointcloud_common_->color_transformer_property_->setValue("RGB8");
      } else
      {
        depthmap_tf_filter_->registerCallback(boost::bind(&DepthCloudDisplay::processMessage, this, _1));
      }

    }
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Message", QString("Error subscribing: ") + e.what() );
  }
  catch (image_transport::TransportLoadException& e)
  {
    setStatus( StatusProperty::Error, "Message", QString("Error subscribing: ") + e.what() );
  }
}

void DepthCloudDisplay::caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg )
{
  boost::mutex::scoped_lock lock(camInfo_mutex_);
  camInfo_ = msg;
 }

void DepthCloudDisplay::unsubscribe()
{
  clear();

  try

  {
    // reset all filters
    sync_depth_color_.reset(new synchronizer_depth_color_(sync_policy_depth_color_(queue_size_)));
    depthmap_tf_filter_.reset();
    depthmap_sub_.reset();
    rgb_sub_.reset();
    cameraInfo_sub_.reset();
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Message", QString("Error unsubscribing: ") + e.what() );
  }
}


void DepthCloudDisplay::clear()
{

  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_common_->reset();
}


void DepthCloudDisplay::update(float wall_dt, float ros_dt)
{

  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_common_->update(wall_dt, ros_dt);
}


void DepthCloudDisplay::reset()
{
  clear();
  messages_received_ = 0;
  setStatus( StatusProperty::Ok, "Depth Map", "0 depth maps received" );
  setStatus( StatusProperty::Ok, "Message", "Ok" );
}

void DepthCloudDisplay::processMessage(const sensor_msgs::ImageConstPtr& depth_msg)
{

  processMessage(depth_msg, sensor_msgs::ImageConstPtr());

}

void DepthCloudDisplay::processMessage(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::ImageConstPtr& rgb_msg)
{
   ++messages_received_;
   setStatus( StatusProperty::Ok, "Depth Map", QString::number(messages_received_) + " depth maps received");
   setStatus( StatusProperty::Ok, "Message", "Ok" );


  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(depth_msg->encoding);
  int numChannels = enc::numChannels(depth_msg->encoding);

  sensor_msgs::CameraInfo::ConstPtr camInfo;
  {
    boost::mutex::scoped_lock lock(camInfo_mutex_);
    camInfo = camInfo_;
  }

  if ( use_auto_size_property_->getBool() )
  {
    float f = camInfo->K[0];
    float s = auto_size_factor_property_->getFloat();
    pointcloud_common_->point_world_size_property_->setFloat( s / f );
  }

  // output pointcloud2 message
  sensor_msgs::PointCloud2Ptr cloud_msg ( new sensor_msgs::PointCloud2 );

  if ((bitDepth == 32) && (numChannels == 1))
  {
    // floating point encoded depth map
    convertDepth<float>(depth_msg, rgb_msg, camInfo, cloud_msg);
  }
  else if ((bitDepth == 16) && (numChannels == 1))
  {
    // 32bit integer encoded depth map
    convertDepth<uint16_t>(depth_msg, rgb_msg, camInfo, cloud_msg);
  }
  else
  {
    std::stringstream errorMsg;
    errorMsg << "Input image must be encoded in 32bit floating point format or 16bit integer format (input format is: "
        << depth_msg->encoding << ")";
    setStatusStd( StatusProperty::Error, "Message", errorMsg.str() );
    return;
  }

  // add point cloud message to pointcloud_common to be visualized
  pointcloud_common_->addMessage(cloud_msg);

}

template<typename T>
void DepthCloudDisplay::convertColor(const sensor_msgs::ImageConstPtr& color_msg,
                                     std::vector<uint8_t>& color_data)
  {
    size_t i;
    size_t num_pixel = color_msg->width * color_msg->height;

    // query image properties
    int num_channels = enc::numChannels(color_msg->encoding);

    bool rgb_encoding = false;
    if (color_msg->encoding.find("rgb")!=std::string::npos)
      rgb_encoding = true;

    bool has_alpha = enc::hasAlpha(color_msg->encoding);

    // prepare output vector
    color_data.clear();
    color_data.reserve(num_pixel * 3 * sizeof(uint8_t));

    uint8_t* img_ptr = (uint8_t*)&color_msg->data[sizeof(T) - 1];// pointer to most significant byte

    // color conversion
    switch (num_channels)
    {
      case 1:
        // grayscale image
        for (i = 0; i < num_pixel; ++i)
        {
          uint8_t gray_value = *img_ptr;
          img_ptr += sizeof(T);

          color_data.push_back(gray_value);
          color_data.push_back(gray_value);
          color_data.push_back(gray_value);
        }
        break;
      case 3:
      case 4:
        // rgb/bgr encoding
        for (i = 0; i < num_pixel; ++i)
        {
          uint8_t color1 = *((uint8_t*)img_ptr);
          img_ptr += sizeof(T);
          uint8_t color2 = *((uint8_t*)img_ptr);
          img_ptr += sizeof(T);
          uint8_t color3 = *((uint8_t*)img_ptr);
          img_ptr += sizeof(T);

          if (has_alpha)
            img_ptr += sizeof(T); // skip alpha values

          if (rgb_encoding)
          {
            // rgb encoding
            color_data.push_back(color1);
            color_data.push_back(color2);
            color_data.push_back(color3);
          } else
          {
            // bgr encoding
            color_data.push_back(color3);
            color_data.push_back(color2);
            color_data.push_back(color1);
          }
        }
        break;
      default:
        break;
    }

  }



template<typename T>
void DepthCloudDisplay::convertDepth(const sensor_msgs::ImageConstPtr& depth_msg,
                                     const sensor_msgs::ImageConstPtr& color_msg,
                                     const sensor_msgs::CameraInfo::ConstPtr camInfo_msg,
                                     sensor_msgs::PointCloud2Ptr& cloud_msg)
  {


    int width = depth_msg->width;
    int height = depth_msg->height;

    //////////////////////////////
    // initialize cloud message
    //////////////////////////////

    cloud_msg->header = depth_msg->header;

    if (color_msg)
    {
      cloud_msg->fields.resize(4);
      cloud_msg->fields[0].name = "x";
      cloud_msg->fields[1].name = "y";
      cloud_msg->fields[2].name = "z";
      cloud_msg->fields[3].name = "rgb";
    }
    else
    {
      cloud_msg->fields.resize(3);
      cloud_msg->fields[0].name = "x";
      cloud_msg->fields[1].name = "y";
      cloud_msg->fields[2].name = "z";
    }

    int offset = 0;
    // All offsets are *4, as all field data types are float32
    for (size_t d = 0; d < cloud_msg->fields.size(); ++d, offset += 4)
    {
      cloud_msg->fields[d].offset = offset;
      cloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_msg->fields[d].count = 1;
    }

    cloud_msg->point_step = offset;

    cloud_msg->data.resize(height * width * offset);
    cloud_msg->is_bigendian = false;
    cloud_msg->is_dense = false;

    //////////////////////////////
    // Update camera model
    //////////////////////////////

    if( !camInfo_msg )
    {
      setStatus( StatusProperty::Error, "Message", "Waiting for CameraInfo message.." );
      return;
    }

    // The following computation of center_x,y and fx,fy duplicates
    // code in the image_geometry package, but this avoids dependency
    // on OpenCV, which simplifies releasing rviz.

    // Use correct principal point from calibration
    float center_x = camInfo_msg->P[2] - camInfo_msg->roi.x_offset;
    float center_y = camInfo_msg->P[6] - camInfo_msg->roi.y_offset;

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double scale_x = camInfo_msg->binning_x > 1 ? (1.0 / camInfo_msg->binning_x) : 1.0;
    double scale_y = camInfo_msg->binning_y > 1 ? (1.0 / camInfo_msg->binning_y) : 1.0;

    double fx = camInfo_msg->P[0] * scale_x;
    double fy = camInfo_msg->P[5] * scale_y;

    float constant_x = 1.0f / fx;
    float constant_y = 1.0f / fy;

    //////////////////////////////
    // Color conversion
    //////////////////////////////

    uint8_t* colorImgPtr = 0;
    std::vector<uint8_t> color_data;

    if (color_msg)
    {

      if (depth_msg->header.frame_id != color_msg->header.frame_id)
      {
        std::stringstream errorMsg;
        errorMsg << "Depth image frame id [" << depth_msg->header.frame_id.c_str()
            << "] doesn't match color image frame id [" << color_msg->header.frame_id.c_str() << "]";
        setStatusStd( StatusProperty::Error, "Message", errorMsg.str() );
        return;
      }

      if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height)
      {
        std::stringstream errorMsg;
        errorMsg << "Depth image resolution (" << (int)depth_msg->width << "x" << (int)depth_msg->height << ") "
            "does not match color image resolution (" << (int)color_msg->width << "x" << (int)color_msg->height << ")";
        setStatusStd( StatusProperty::Error, "Message", errorMsg.str() );
        return;
      }

      // convert color coding to 8-bit rgb data
      switch (enc::bitDepth(color_msg->encoding))
      {
        case 8:
          convertColor<uint8_t>(color_msg,color_data);
          break;
        case 16:
          convertColor<uint16_t>(color_msg,color_data);
          break;
        default:
          setStatus( StatusProperty::Error, "Message", "Color image has invalid bit depth." );
          return;
          break;
      }

      colorImgPtr = &color_data[0];
    }

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    float* cloudDataPtr = reinterpret_cast<float*>(&cloud_msg->data[0]);

    size_t pointCount_ = 0;

    Ogre::Vector3 newPoint;
    Ogre::Vector3 transPoint;

    const T* img_ptr = (T*)&depth_msg->data[0];
    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u)
      {
        uint32_t color_rgb;

        if (colorImgPtr)
        {
          uint8_t color_r, color_g, color_b;
          color_r = *colorImgPtr; ++colorImgPtr;
          color_g = *colorImgPtr; ++colorImgPtr;
          color_b = *colorImgPtr; ++colorImgPtr;

          color_rgb = ((uint32_t)color_r << 16 | (uint32_t)color_g << 8 | (uint32_t)color_b);
        }

        float depth = DepthTraits<T>::toMeters( *img_ptr++ );

        // Missing points denoted by NaNs
        if (!DepthTraits<T>::valid(depth))
        {
          continue;
        }

        // Fill in XYZ
        newPoint.x = (u - center_x) * depth * constant_x;
        newPoint.y = (v - center_y) * depth * constant_y;
        newPoint.z = depth;

        if (validateFloats(newPoint))
        {
          *cloudDataPtr = newPoint.x; ++cloudDataPtr;
          *cloudDataPtr = newPoint.y; ++cloudDataPtr;
          *cloudDataPtr = newPoint.z; ++cloudDataPtr;
        }

        ++pointCount_;

        if (colorImgPtr)
        {
          *cloudDataPtr = *reinterpret_cast<float*>(&color_rgb);
          ++cloudDataPtr;
        }
      }
    }

    ////////////////////////////////////////////////
    // finalize pointcloud2 message
    ////////////////////////////////////////////////
    cloud_msg->width = pointCount_;
    cloud_msg->height = 1;
    cloud_msg->data.resize(cloud_msg->height * cloud_msg->width * cloud_msg->point_step);
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
  }

void DepthCloudDisplay::scanForTransportSubscriberPlugins()
{
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader("image_transport",
                                                                       "image_transport::SubscriberPlugin");

  BOOST_FOREACH( const std::string& lookup_name, sub_loader.getDeclaredClasses() )
  {
    // lookup_name is formatted as "pkg/transport_sub", for instance
    // "image_transport/compressed_sub" for the "compressed"
    // transport.  This code removes the "_sub" from the tail and
    // everything up to and including the "/" from the head, leaving
    // "compressed" (for example) in transport_name.
    std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
    transport_name = transport_name.substr(lookup_name.find('/') + 1);

    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try
    {
      boost::shared_ptr<image_transport::SubscriberPlugin> sub = sub_loader.createInstance(lookup_name);
      transport_plugin_types_.insert(transport_name);
    }
    catch (const pluginlib::LibraryLoadException& e)
    {
    }
    catch (const pluginlib::CreateClassException& e)
    {
    }
  }
}

void DepthCloudDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void DepthCloudDisplay::fillTransportOptionList(EnumProperty* property)
{
  property->clearOptions();

  std::vector<std::string> choices;

  choices.push_back("raw");

  // Loop over all current ROS topic names
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    // If the beginning of this topic name is the same as topic_,
    // and the whole string is not the same,
    // and the next character is /
    // and there are no further slashes from there to the end,
    // then consider this a possible transport topic.
    const ros::master::TopicInfo& ti = *it;
    const std::string& topic_name = ti.name;
    const std::string& topic = depth_topic_property_->getStdString();

    if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/'
        && topic_name.find('/', topic.size() + 1) == std::string::npos)
    {
      std::string transport_type = topic_name.substr(topic.size() + 1);

      // If the transport type string found above is in the set of
      // supported transport type plugins, add it to the list.
      if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end())
      {
        choices.push_back(transport_type);
      }
    }
  }

  for (size_t i = 0; i < choices.size(); i++)
  {
    property->addOptionStd(choices[i]);
  }
}

void DepthCloudDisplay::fixedFrameChanged()
{
  Display::reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( rviz::DepthCloudDisplay, rviz::Display)

