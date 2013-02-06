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
    voxel_resolution_(0.001),
    pixel_counter_(0),
    global_time_stamp_(0.0f)
  {};
  virtual ~MultiLayerDepth() {
	  reset();
  }


  void addDepthColorCameraInfo(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::ImageConstPtr& color_msg,
                               sensor_msgs::CameraInfoConstPtr& camera_info_msg);


  sensor_msgs::PointCloud2Ptr generatePointCloudFromMLDepth ();
  sensor_msgs::PointCloud2Ptr generatePointCloudFromDepth ();

  void reset()
  {

    std::vector<std::vector< DepthPixel* > >::iterator it;
    std::vector<std::vector< DepthPixel* > >::const_iterator it_end = multilayer_depth_.end();

    for (it=multilayer_depth_.begin(); it!=it_end; ++it)
    {

      const std::vector<DepthPixel*>& voxel_list = *it;

      std::vector<DepthPixel*>::const_iterator it_layer;
      std::vector<DepthPixel*>::const_iterator it_layer_end = voxel_list.end();

      for (it_layer = voxel_list.begin(); it_layer != it_layer_end; ++it_layer)
      {
        delete (*it_layer);
      }

      it->clear();
      it->reserve(16);
    }

    pixel_counter_ = 0;

    memset(&multilayer_surface_cache_[0], 0, sizeof(DepthPixel*)*multilayer_surface_cache_.size());
    memset(&multilayer_depth_cache_[0], 0, sizeof(float)*multilayer_depth_cache_.size());
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

protected:
  struct DepthPixel
  {
    double time_out_;

    float point_[3];
    uint32_t color_rgb_;
    uint32_t color_rgb_dark_;

  };

  // Convert input color image to 8-bit rgb encoding
  template<typename T>
  void convertColor(const sensor_msgs::ImageConstPtr& color_msg,
                    std::vector<uint8_t>& color_data);

  template<typename T>
    void updateMLDImage(const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::ImageConstPtr& color_msg,
                        sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
                        sensor_msgs::PointCloud2Ptr output_cloud);


  template<typename T>
    sensor_msgs::PointCloud2Ptr generatePointCloud(const sensor_msgs::ImageConstPtr depth_msg,
                                                   const sensor_msgs::ImageConstPtr color_msg,
                                                   sensor_msgs::CameraInfo::ConstPtr camera_info_msg);

  sensor_msgs::PointCloud2Ptr initNewPointCloud();


  DepthPixel* processRay(std::size_t idx, float depth)
  {
    std::vector<DepthPixel*>& voxel_list = multilayer_depth_[idx];

    std::vector<DepthPixel*>::iterator ray_read = voxel_list.begin();
    std::vector<DepthPixel*>::iterator ray_write = voxel_list.begin();
    const std::vector<DepthPixel*>::const_iterator ray_end = voxel_list.end();

    std::size_t size = voxel_list.size();

    // clear out ray
    DepthPixel* ret = 0;
    bool voxel_found = false;

    while (ray_read != ray_end)
    {
      DepthPixel* depth_pixel = *ray_read;

      if ( (depth_pixel->time_out_ < global_time_stamp_ ) ||
           (depth_pixel->point_[2] < depth - voxel_resolution_) )
      {
        delete (depth_pixel);
        --size;
        ++ray_read;

        --pixel_counter_;
      }
      else
      {
        if (!voxel_found && (fabs(depth_pixel->point_[2] - depth) <= voxel_resolution_))
        {
          voxel_found = true;
          ret = depth_pixel;
        }

        (*ray_write) = (*ray_read);

        ++ray_read;
        ++ray_write;
      }
    }
    voxel_list.resize(size);

    return ret;
  }


  void setSize(std::size_t size)
  {
    if (size!=multilayer_depth_.size())
    {
      reset();
      multilayer_depth_.resize(size, std::vector< DepthPixel* >() );
      multilayer_surface_cache_.resize(size, 0);
      multilayer_depth_cache_.resize(size, 0.0f);
      reset();

      std::cout<<"RESET!"<<std::endl;
    }
  }

  boost::mutex input_update_mutex_;
  sensor_msgs::ImageConstPtr depth_image_;
  sensor_msgs::ImageConstPtr color_image_;
  sensor_msgs::CameraInfoConstPtr camera_info_;

  std::vector<std::vector< DepthPixel* > > multilayer_depth_;

  std::vector< DepthPixel*  > multilayer_surface_cache_;
  std::vector< float  > multilayer_depth_cache_;

  double voxel_time_out_;

  float color_filter_;

  float voxel_resolution_;

  std::size_t pixel_counter_;

  double global_time_stamp_;

};

} /* namespace pointcloud_aggregator */
#endif /* MULTI_LAYER_DEPTH_H_ */
