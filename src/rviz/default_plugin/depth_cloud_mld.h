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
    voxel_time_out_(50),
    color_filter_(0.5),
    voxel_resolution_(0.2),
    pixel_counter_(0)
  {};
  virtual ~MultiLayerDepth() {
	  reset();
  }


  void addDepthColorCameraInfo(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::ImageConstPtr& color_msg,
                               sensor_msgs::CameraInfoConstPtr& camera_info_msg);


  sensor_msgs::PointCloud2Ptr generatePointCloud () const;

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
      it->reserve(8);
    }

    pixel_counter_ = 0;

    memset(&multilayer_surface_cache_[0], 0, sizeof(DepthPixel*)*multilayer_surface_cache_.size());
    memset(&multilayer_depth_cache_[0], 0, sizeof(float)*multilayer_depth_cache_.size());
  }

  void setVoxelResolution(float resolution)
  {
    voxel_resolution_ = resolution/1000.0f;
  }

  void setVoxelTimeOut(unsigned int time_out)
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
    unsigned int time_out_;

    float color_r_;
    float color_g_;
    float color_b_;

    float x_;
    float y_;
    float z_;
  };

  // Convert input color image to 8-bit rgb encoding
  template<typename T>
  void convertColor(const sensor_msgs::ImageConstPtr& color_msg,
                    std::vector<uint8_t>& color_data);

  template<typename T>
    void processInputImageData(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg,
                               sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

  DepthPixel* processRay(std::size_t idx, float depth)
  {
    std::vector<DepthPixel*>& voxel_list = multilayer_depth_[idx];

    std::size_t size = voxel_list.size();

    std::vector<DepthPixel*>::const_iterator it_read = voxel_list.begin();
    std::vector<DepthPixel*>::iterator it_write = voxel_list.begin();
    std::vector<DepthPixel*>::const_iterator it_end = voxel_list.end();

    // clear out ray
    DepthPixel* ret = 0;

    bool voxel_found = false;

    while (it_read != it_end)
    {
      DepthPixel* depth_pixel = *it_read;

      if ((  depth_pixel->z_ < depth - voxel_resolution_) ||
          (!depth_pixel->time_out_) )
      {
        delete (depth_pixel);
        --size;
        ++it_read;

        --pixel_counter_;

      }
      else
      {
        if (!voxel_found && (fabs(depth_pixel->z_ - depth) <= voxel_resolution_))
        {
          voxel_found = true;
          ret = depth_pixel;
        }

        (*it_write) = (*it_read);

        ++it_read;
        ++it_write;
      }
    }
    voxel_list.resize(size);

    return ret;
  }



  void setSize(std::size_t size)
  {
    if (size!=multilayer_depth_.size())
    {
      multilayer_depth_.resize(size, std::vector< DepthPixel* >() );
      multilayer_surface_cache_.resize(size, 0);
      multilayer_depth_cache_.resize(size, 0.0f);
      reset();
    }
  }


  std::vector<std::vector< DepthPixel* > > multilayer_depth_;

  std::vector< DepthPixel*  > multilayer_surface_cache_;
  std::vector< float  > multilayer_depth_cache_;

  unsigned int voxel_time_out_;

  float color_filter_;

  float voxel_resolution_;

  std::size_t pixel_counter_;

};

} /* namespace pointcloud_aggregator */
#endif /* MULTI_LAYER_DEPTH_H_ */
