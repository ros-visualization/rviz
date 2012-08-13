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

// opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>

#include <sstream>

namespace enc = sensor_msgs::image_encodings;

namespace rviz
{

DepthCloudDisplay::DepthCloudDisplay()
  : rviz::Display()
  , messages_received_(0)
  , depthmap_it_(update_nh_)
  , depthmap_sub_()
  , rgb_it_ (update_nh_)
  , rgb_sub_()
  , cameraInfo_sub_()
  , queue_size_(5)
{

  // Depth map properties
  depth_topic_property_ = new RosTopicProperty("Depth Map Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                         "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateTopic() ));

  depth_transport_property_ = new EnumProperty("Depth Map Transport Hint", "raw", "Preferred method of sending images.", this,
                                         SLOT( updateTopic() ));

  connect(depth_transport_property_, SIGNAL( requestOptions( EnumProperty* )), this,
          SLOT( fillTransportOptionList( EnumProperty* )));

  depth_transport_property_->setStdString("raw");

  // RGB image properties
  rgb_topic_property_ = new RosTopicProperty("RGB Image Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                         "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateTopic() ));

  rgb_transport_property_ = new EnumProperty("RGB Transport Hint", "raw", "Preferred method of sending images.", this,
                                         SLOT( updateTopic() ));


  connect(rgb_transport_property_, SIGNAL( requestOptions( EnumProperty* )), this,
          SLOT( fillTransportOptionList( EnumProperty* )));

  rgb_transport_property_->setStdString("raw");


  // Queue size property
  queue_size_property_ = new IntProperty( "Queue Size", queue_size_,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 1 );


  // Instantiate PointCloudCommon class for displaying point clouds
  pointcloud_common_ = new PointCloudCommon(this);

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue( pointcloud_common_->getCallbackQueue() );

  // Scan for available transport plugins
  scanForTransportSubscriberPlugins();

}

void DepthCloudDisplay::onInitialize()
{
  pointcloud_common_->initialize(context_, scene_node_);
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
    syncDepthRGB_.reset(new SynchronizerDepthRGB(SyncPolicyDepthRGB(queue_size_)));
    depthmap_tf_filter_.reset();
    depthmap_sub_.reset(new image_transport::SubscriberFilter());
    rgb_sub_.reset(new image_transport::SubscriberFilter());
    cameraInfo_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

    std::string depthmap_topic = depth_topic_property_->getTopicStd();
    std::string depthmap_transport = depth_transport_property_->getStdString();
    std::string rgb_topic = rgb_topic_property_->getTopicStd();
    std::string rgb_transport = rgb_transport_property_->getStdString();

    if (!depthmap_topic.empty() && !depthmap_transport.empty()) {
      // subscribe to depth map topic
      depthmap_sub_->subscribe(depthmap_it_, depthmap_topic, queue_size_,  image_transport::TransportHints(depthmap_transport));

      depthmap_tf_filter_.reset(
          new tf::MessageFilter<sensor_msgs::Image>(*depthmap_sub_, *context_->getTFClient(), fixed_frame_.toStdString(), queue_size_, update_nh_));

      // subscribe to CameraInfo  topic
      std::string info_topic = image_transport::getCameraInfoTopic(depthmap_topic);
      cameraInfo_sub_->subscribe(update_nh_, info_topic, queue_size_);
      cameraInfo_sub_->registerCallback(boost::bind(&DepthCloudDisplay::caminfoCallback, this, _1));

      if (!rgb_topic.empty() && !rgb_transport.empty()) {
        // subscribe to rgb image topic
        rgb_sub_->subscribe(rgb_it_, rgb_topic, queue_size_,  image_transport::TransportHints(rgb_transport));

        // connect message filters to synchronizer
        syncDepthRGB_->connectInput(*depthmap_tf_filter_, *rgb_sub_);
        syncDepthRGB_->setInterMessageLowerBound(0, ros::Duration(0.5));
        syncDepthRGB_->setInterMessageLowerBound(1, ros::Duration(0.5));
        syncDepthRGB_->registerCallback(boost::bind(&DepthCloudDisplay::processMessage, this, _1, _2));
      } else
      {
        depthmap_tf_filter_->registerCallback(boost::bind(&DepthCloudDisplay::processMessage, this, _1));
      }

    }
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
  catch (image_transport::TransportLoadException e)
  {
    setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
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

  boost::mutex::scoped_lock lock(mutex_);

  try

  {
    // reset all filters
    syncDepthRGB_.reset(new SynchronizerDepthRGB(SyncPolicyDepthRGB(queue_size_)));
    depthmap_tf_filter_.reset();
    depthmap_sub_.reset();
    rgb_sub_.reset();
    cameraInfo_sub_.reset();
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
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
  setStatus(StatusProperty::Ok, "Depth Map", QString("0 depth maps received"));
}

void DepthCloudDisplay::processMessage(const sensor_msgs::ImageConstPtr& depth_msg)
{

  processMessage(depth_msg, sensor_msgs::ImageConstPtr());

}

void DepthCloudDisplay::processMessage(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::ImageConstPtr& rgb_msg)
{
   ++messages_received_;
   setStatus(StatusProperty::Ok, "Depth Map", QString::number(messages_received_) + " depth maps received");


  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(depth_msg->encoding);
  int numChannels = enc::numChannels(depth_msg->encoding);

  sensor_msgs::CameraInfo::ConstPtr camInfo;
  {
    boost::mutex::scoped_lock lock(camInfo_mutex_);
    camInfo = camInfo_;
  }

  // output pointcloud2 message
  sensor_msgs::PointCloud2Ptr cloud_msg ( new sensor_msgs::PointCloud2 );

  if ((bitDepth == 32) && (numChannels == 1))
  {
    // floating point encoded depth map
    convert<float>(depth_msg, rgb_msg, camInfo, cloud_msg);
  }
  else if ((bitDepth == 16) && (numChannels == 1))
  {
    // 32bit integer encoded depth map
    convert<uint16_t>(depth_msg, rgb_msg, camInfo, cloud_msg);
  }
  else
  {
    std::stringstream errorMsg;
    errorMsg << "Input image must be encoded in 32bit floating point format or 16bit integer format (input format is: "
        << depth_msg->encoding << ")";
    setStatus(StatusProperty::Error, "Message", QString(errorMsg.str().c_str()));
    return;
  }

  // add point cloud message to pointcloud_common to be visualized
  pointcloud_common_->addMessage(cloud_msg);

  setStatus(StatusProperty::Ok, "Message", QString("Ok"));

}

template<typename T>
void DepthCloudDisplay::convert(const sensor_msgs::ImageConstPtr& depth_msg,
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

    image_geometry::PinholeCameraModel cameraModel;

    if (camInfo_msg)
    {
      cameraModel.fromCameraInfo(camInfo_msg);
    }
    else
    {
      setStatus(StatusProperty::Error, "Message", QString("Waiting for CameraInfo message.."));
      return;
    }

    // Use correct principal point from calibration
    float center_x = cameraModel.cx();
    float center_y = cameraModel.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float constant_x = 1.0f / cameraModel.fx();
    float constant_y = 1.0f / cameraModel.fy();

    //////////////////////////////
    // Color conversion
    //////////////////////////////

    unsigned char* colorImgPtr = 0;

    if (color_msg)
    {

      if (depth_msg->header.frame_id != color_msg->header.frame_id)
      {
        std::stringstream errorMsg;
        errorMsg << "Depth image frame id [" << depth_msg->header.frame_id.c_str()
            << "] doesn't match RGB image frame id [" << color_msg->header.frame_id.c_str() << "]";
        setStatus(StatusProperty::Error, "Message", QString(errorMsg.str().c_str()) );
        return;
      }

      if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height)
      {
        std::stringstream errorMsg;
        errorMsg << "Depth resolution (" << (int)depth_msg->width << "x" << (int)depth_msg->height << ") "
            "does not match RGB resolution (" << (int)color_msg->width << "x" << (int)color_msg->height << ")";
        setStatus(StatusProperty::Error, "Message", QString(errorMsg.str().c_str()) );
        return;
      }

      // OpenCV-ros bridge
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(color_msg, "rgba8");

      }
      catch (cv_bridge::Exception& e)
      {
        setStatus(StatusProperty::Error, "Message", QString("OpenCV-ROS bridge: ") + e.what());
        return;
      }
      catch (cv::Exception& e)
      {
        setStatus(StatusProperty::Error, "Message", QString("OpenCV: ") + e.what());
        return;
      }

      colorImgPtr = (unsigned char*)&cv_ptr->image.data[0];
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
          colorImgPtr++; // alpha padding

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

PLUGINLIB_DECLARE_CLASS( rviz, DepthCloud, rviz::DepthCloudDisplay, rviz::Display)

