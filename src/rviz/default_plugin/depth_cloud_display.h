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

#ifndef RVIZ_DEPTH_CLOUD_DISPLAY_H
#define RVIZ_DEPTH_CLOUD_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/message_filter.h>

#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"

#include <rviz/display.h>

#include <rviz/default_plugin/point_cloud_common.h>
#endif

#include <QMap>

using namespace message_filters::sync_policies;

// Forward declarations

namespace image_transport
{
class SubscriberFilter;
}

namespace rviz
{
class EnumProperty;
class FloatProperty;
class BoolProperty;
class IntProperty;

class MultiLayerDepth;

class RosFilteredTopicProperty : public RosTopicProperty
{
  Q_OBJECT
public:
  RosFilteredTopicProperty(const QString& name = QString(),
                           const QString& default_value = QString(),
                           const QString& message_type = QString(),
                           const QString& description = QString(),
                           const QRegExp& filter = QRegExp(),
                           Property* parent = nullptr,
                           const char* changed_slot = nullptr,
                           QObject* receiver = nullptr)
    : RosTopicProperty(name, default_value, message_type, description, parent, changed_slot, receiver)
    , filter_(filter)
    , filter_enabled_(true)
  {
  }

public:
  void enableFilter(bool enabled)
  {
    filter_enabled_ = enabled;
    fillTopicList();
  }

  QRegExp filter() const
  {
    return filter_;
  }

protected Q_SLOTS:
  void fillTopicList() override
  {
    QStringList filtered_strings_;

    // Obtain list of available topics
    RosTopicProperty::fillTopicList();
    // Apply filter
    if (filter_enabled_)
      strings_ = strings_.filter(filter_);
  }

private:
  QRegExp filter_;
  bool filter_enabled_;
};

/**
 * \class DepthCloudDisplay
 *
 */
class DepthCloudDisplay : public rviz::Display
{
  Q_OBJECT
public:
  DepthCloudDisplay();
  ~DepthCloudDisplay() override;

  void onInitialize() override;

  // Overrides from Display
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void setTopic(const QString& topic, const QString& datatype) override;

protected Q_SLOTS:
  void updateQueueSize();
  /** @brief Fill list of available and working transport options */
  void fillTransportOptionList(EnumProperty* property);

  // Property callbacks
  virtual void updateTopic();
  virtual void updateTopicFilter();
  virtual void updateUseAutoSize();
  virtual void updateAutoSizeFactor();
  virtual void updateUseOcclusionCompensation();
  virtual void updateOcclusionTimeOut();

protected:
  void scanForTransportSubscriberPlugins();

  typedef std::vector<rviz::PointCloud::Point> V_Point;

  virtual void processMessage(sensor_msgs::Image::ConstPtr msg);
  virtual void processMessage(sensor_msgs::ImageConstPtr depth_msg, sensor_msgs::ImageConstPtr rgb_msg);
  void caminfoCallback(sensor_msgs::CameraInfo::ConstPtr msg);

  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  void fixedFrameChanged() override;

  void subscribe();
  void unsubscribe();

  void clear();

  // thread-safe status updates
  // add status update to global status list
  void updateStatus(StatusProperty::Level level, const QString& name, const QString& text);

  // use global status list to update rviz plugin status
  void setStatusList();

  uint32_t messages_received_;

  boost::mutex mutex_;

  // ROS image subscription & synchronization
  boost::scoped_ptr<image_transport::ImageTransport> depthmap_it_;
  boost::shared_ptr<image_transport::SubscriberFilter> depthmap_sub_;
  boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::Image> > depthmap_tf_filter_;
  boost::scoped_ptr<image_transport::ImageTransport> rgb_it_;
  boost::shared_ptr<image_transport::SubscriberFilter> rgb_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > cam_info_sub_;
  sensor_msgs::CameraInfo::ConstPtr cam_info_;
  boost::mutex cam_info_mutex_;

  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyDepthColor;
  typedef message_filters::Synchronizer<SyncPolicyDepthColor> SynchronizerDepthColor;

  boost::shared_ptr<SynchronizerDepthColor> sync_depth_color_;

  // RVIZ properties
  Property* topic_filter_property_;
  IntProperty* queue_size_property_;
  BoolProperty* use_auto_size_property_;
  FloatProperty* auto_size_factor_property_;
  RosFilteredTopicProperty* depth_topic_property_;
  EnumProperty* depth_transport_property_;
  RosFilteredTopicProperty* color_topic_property_;
  EnumProperty* color_transport_property_;
  BoolProperty* use_occlusion_compensation_property_;
  FloatProperty* occlusion_shadow_timeout_property_;

  uint32_t queue_size_;

  MultiLayerDepth* ml_depth_data_;

  Ogre::Quaternion current_orientation_;
  Ogre::Vector3 current_position_;
  float angular_thres_;
  float trans_thres_;

  PointCloudCommon* pointcloud_common_;

  std::set<std::string> transport_plugin_types_;
};


} // namespace rviz

#endif // RVIZ_DEPTHMAP_TO_POINTCLOUD_DISPLAY_H
