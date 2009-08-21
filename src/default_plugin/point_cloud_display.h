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

#ifndef RVIZ_POINT_CLOUD_DISPLAY_H
#define RVIZ_POINT_CLOUD_DISPLAY_H

#include "point_cloud_base.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include "ogre_tools/point_cloud.h"

#include "sensor_msgs/PointCloud.h"

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <deque>
#include <queue>
#include <vector>

namespace rviz
{

/**
 * \class PointCloudDisplay
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class PointCloudDisplay : public PointCloudBase
{
public:
  PointCloudDisplay( const std::string& name, VisualizationManager* manager );
  ~PointCloudDisplay();

  // Overrides from Display
  virtual void createProperties();
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();

  /**
   * Set the incoming PointCloud topic
   * @param topic The topic we should listen to
   */
  void setTopic( const std::string& topic );
  const std::string& getTopic() { return topic_; }

protected:
  virtual void onEnable();
  virtual void onDisable();

  /**
   * \brief Subscribes to the topic set by setTopic()
   */
  void subscribe();
  /**
   * \brief Unsubscribes from the current topic
   */
  void unsubscribe();

  /**
   * \brief ROS callback for an incoming point cloud message
   */
  void incomingCloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud);

  std::string topic_;                         ///< The PointCloud topic set by setTopic()

  message_filters::Subscriber<sensor_msgs::PointCloud> sub_;
  tf::MessageFilter<sensor_msgs::PointCloud> tf_filter_;

  ROSTopicStringPropertyWPtr topic_property_;
};

} // namespace rviz

#endif
