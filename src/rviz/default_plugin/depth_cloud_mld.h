/*
 * Copyright (c) 2013, Willow Garage, Inc.
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
 *
 *  Created on: Jan 22, 2013
 *      Author: jkammerl
 */

#ifndef RVIZ_MULTI_LAYER_DEPTH_H_
#define RVIZ_MULTI_LAYER_DEPTH_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <exception>

namespace rviz
{

class MultiLayerDepthException: public std::exception
{
public:
	MultiLayerDepthException(const std::string& error_msg) :
		std::exception(), error_msg_(error_msg) {
	}
	virtual ~MultiLayerDepthException() throw () {}

	virtual const char * what() const throw () {
		return error_msg_.c_str();
	}

protected:
	std::string error_msg_;
};

class MultiLayerDepth
{
public:
  MultiLayerDepth() :
    voxel_time_out_(1.0),
    color_filter_(0.5),
    voxel_resolution_(0.01)
  {};
  virtual ~MultiLayerDepth() {
	  reset();
  }


  void addDepthColorCameraInfo(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::ImageConstPtr& color_msg,
                               sensor_msgs::CameraInfoConstPtr& camera_info_msg);


  sensor_msgs::PointCloud2Ptr generatePointCloudFromDepth ();

  void reset()
  {
    memset(&multilayer_depth_cache_[0], 0, sizeof(float)*multilayer_depth_cache_.size());
    memset(&point_shadow_cache_[0], 0, sizeof(uint8_t)*point_shadow_cache_.size());
    memset(&multilayer_depth_timeout_[0], 0, sizeof(double)*multilayer_depth_timeout_.size());
  }

  void setVoxelResolution(float resolution)
  {
    voxel_resolution_ = resolution;
  }

  void setVoxelTimeOut(double time_out)
  {
    voxel_time_out_ = time_out;
  }

  void setColorTransitionFilter(float filter_val)
  {
    color_filter_ = filter_val;
  }

  void enableOcclusionCompensation(bool occlusion_compensation)
  {
    occlusion_compensation_ = occlusion_compensation;
    reset();
  }

protected:
  // Convert input color image to 8-bit rgb encoding
  template<typename T>
  void convertColor(const sensor_msgs::ImageConstPtr& color_msg,
                    std::vector<uint32_t>& rgba_color_raw);

  template<typename T>
    sensor_msgs::PointCloud2Ptr generatePointCloudML(const sensor_msgs::ImageConstPtr& depth_msg,
                                                     std::vector<uint32_t>& rgba_color_raw);

  template<typename T>
    sensor_msgs::PointCloud2Ptr generatePointCloudSL(const sensor_msgs::ImageConstPtr& depth_msg,
                                                     std::vector<uint32_t>& rgba_color_raw);

  sensor_msgs::PointCloud2Ptr initPointCloud();
  void finalizingPointCloud(sensor_msgs::PointCloud2Ptr& point_cloud, std::size_t size);

  void initializeConversion();

  boost::mutex input_update_mutex_;
  sensor_msgs::ImageConstPtr depth_image_;
  sensor_msgs::ImageConstPtr color_image_;
  sensor_msgs::CameraInfoConstPtr camera_info_;

  std::vector<float> projectionMapX_;
  std::vector<float> projectionMapY_;

  std::vector< float > multilayer_depth_cache_;
  std::vector< double > multilayer_depth_timeout_;
  std::vector< uint8_t > point_shadow_cache_;

  double voxel_time_out_;

  float color_filter_;

  float voxel_resolution_;

  bool occlusion_compensation_;

};

} /* namespace pointcloud_aggregator */
#endif /* MULTI_LAYER_DEPTH_H_ */
