/*
 * Copyright (c) 20013, Willow Garage, Inc.
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
 *
 *  Created on: Jan 22, 2013
 *      Author: jkammerl
 */

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <string.h>
#include <sstream>

#include "depth_cloud_mld.h"

namespace enc = sensor_msgs::image_encodings;

#define POINT_STEP (sizeof(float)*4)

namespace rviz
{

// Encapsulate differences between processing float and uint16_t depths
template<typename T>
  struct DepthTraits
  {
  };

template<>
  struct DepthTraits<uint16_t>
  {
    static inline bool valid(float depth)
    {
      return depth != 0.0;
    }
    static inline float toMeters(uint16_t depth)
    {
      return depth * 0.001f;
    } // originally mm
  };

template<>
  struct DepthTraits<float>
  {
    static inline bool valid(float depth)
    {
      return std::isfinite(depth);
    }
    static inline float toMeters(float depth)
    {
      return depth;
    }
  };


struct RGBA
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha;
};


void MultiLayerDepth::initializeConversion(const sensor_msgs::ImageConstPtr& depth_msg,
                                           sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{

  if (!depth_msg || !camera_info_msg)
  {
    std::string error_msg ("Waiting for CameraInfo message..");
    throw( MultiLayerDepthException (error_msg));
  }

  // do some sanity checks
  int binning_x = camera_info_msg->binning_x > 1 ? camera_info_msg->binning_x : 1;
  int binning_y = camera_info_msg->binning_y > 1 ? camera_info_msg->binning_y : 1;

  int roi_width = camera_info_msg->roi.width > 0 ? camera_info_msg->roi.width : camera_info_msg->width;
  int roi_height = camera_info_msg->roi.height > 0 ? camera_info_msg->roi.height : camera_info_msg->height;

  int expected_width = roi_width / binning_x;
  int expected_height = roi_height / binning_y;

  if ( expected_width != depth_msg->width ||
       expected_height != depth_msg->height )
  {
    std::ostringstream s;
    s << "Depth image size and camera info don't match: ";
    s << depth_msg->width << " x " << depth_msg->height;
    s << " vs " << expected_width << " x " << expected_height;
    s << "(binning: " << binning_x << " x " << binning_y;
    s << ", ROI size: " << roi_width << " x " << roi_height << ")";
    throw( MultiLayerDepthException (s.str()));
  }

  int width = depth_msg->width;
  int height = depth_msg->height;

  std::size_t size = width * height;

  if (size != shadow_depth_.size())
  {
    // Allocate memory for shadow processing
    shadow_depth_.resize(size, 0.0f);
    shadow_timestamp_.resize(size, 0.0);
    shadow_buffer_.resize(size * POINT_STEP, 0);

    // Procompute 3D projection matrix
    //
    // The following computation of center_x,y and fx,fy duplicates
    // code in the image_geometry package, but this avoids dependency
    // on OpenCV, which simplifies releasing rviz.

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double scale_x = camera_info_msg->binning_x > 1 ? (1.0 / camera_info_msg->binning_x) : 1.0;
    double scale_y = camera_info_msg->binning_y > 1 ? (1.0 / camera_info_msg->binning_y) : 1.0;

    // Use correct principal point from calibration
    float center_x = (camera_info_msg->P[2] - camera_info_msg->roi.x_offset)*scale_x;
    float center_y = (camera_info_msg->P[6] - camera_info_msg->roi.y_offset)*scale_y;

    double fx = camera_info_msg->P[0] * scale_x;
    double fy = camera_info_msg->P[5] * scale_y;

    float constant_x = 1.0f / fx;
    float constant_y = 1.0f / fy;

    projection_map_x_.resize(width);
    projection_map_y_.resize(height);
    std::vector<float>::iterator projX = projection_map_x_.begin();
    std::vector<float>::iterator projY = projection_map_y_.begin();

    // precompute 3D projection matrix
    for (int v = 0; v < height; ++v, ++projY)
      *projY = (v - center_y) * constant_y;

    for (int u = 0; u < width; ++u, ++projX)
      *projX = (u - center_x) * constant_x;

    // reset shadow vectors
    reset();
  }
}


template<typename T>
  sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudSL(const sensor_msgs::ImageConstPtr& depth_msg,
                                                                    std::vector<uint32_t>& rgba_color_raw)
  {

    int width = depth_msg->width;
    int height = depth_msg->height;

    sensor_msgs::PointCloud2Ptr cloud_msg = initPointCloud();
    cloud_msg->data.resize(height * width * cloud_msg->point_step);

    uint32_t* color_img_ptr = 0;

    if (rgba_color_raw.size())
      color_img_ptr = &rgba_color_raw[0];

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    float* cloud_data_ptr = reinterpret_cast<float*>(&cloud_msg->data[0]);
    const std::size_t point_step = cloud_msg->point_step;

    std::size_t point_count = 0;
    std::size_t point_idx = 0;

    double time_now = ros::Time::now().toSec();
    double time_expire = time_now+shadow_time_out_;

    const T* depth_img_ptr = (T*)&depth_msg->data[0];

    std::vector<float>::iterator proj_x;
    std::vector<float>::const_iterator proj_x_end = projection_map_x_.end();

    std::vector<float>::iterator proj_y ;
    std::vector<float>::const_iterator proj_y_end = projection_map_y_.end();

    // iterate over projection matrix
    for (proj_y = projection_map_y_.begin(); proj_y !=proj_y_end; ++proj_y)
    {
      for (proj_x = projection_map_x_.begin(); proj_x !=proj_x_end; ++proj_x,
                                                                    ++point_idx,
                                                                    ++depth_img_ptr)
      {

        T depth_raw = *depth_img_ptr;
        if (DepthTraits<T>::valid(depth_raw))
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          // define point color
          uint32_t color;
          if (color_img_ptr)
          {
            color = *color_img_ptr;
          }
          else
          {
            color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
          }

          // fill in X,Y,Z and color
          *cloud_data_ptr = (*proj_x) * depth;  ++cloud_data_ptr;
          *cloud_data_ptr = (*proj_y) * depth;  ++cloud_data_ptr;
          *cloud_data_ptr = depth; ++cloud_data_ptr;
          *cloud_data_ptr = *reinterpret_cast<float*>(&color); ++cloud_data_ptr;

          ++point_count;
        }

        // increase color iterator pointer
        if (color_img_ptr)
          ++color_img_ptr;
      }
    }

    finalizingPointCloud(cloud_msg, point_count);

    return cloud_msg;
  }


template<typename T>
  sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudML(const sensor_msgs::ImageConstPtr& depth_msg,
                                                                    std::vector<uint32_t>& rgba_color_raw)
  {
    int width = depth_msg->width;
    int height = depth_msg->height;

    sensor_msgs::PointCloud2Ptr cloud_msg = initPointCloud();
    cloud_msg->data.resize(height * width * cloud_msg->point_step * 2);

    uint32_t* color_img_ptr = 0;

    if (rgba_color_raw.size())
      color_img_ptr = &rgba_color_raw[0];

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    float* cloud_data_ptr = reinterpret_cast<float*>(&cloud_msg->data[0]);
    uint8_t* cloud_shadow_buffer_ptr = &shadow_buffer_[0];

    const std::size_t point_step = cloud_msg->point_step;

    std::size_t point_count = 0;
    std::size_t point_idx = 0;

    double time_now = ros::Time::now().toSec();
    double time_expire = time_now-shadow_time_out_;

    const T* depth_img_ptr = (T*)&depth_msg->data[0];

    std::vector<float>::iterator proj_x;
    std::vector<float>::const_iterator proj_x_end = projection_map_x_.end();

    std::vector<float>::iterator proj_y ;
    std::vector<float>::const_iterator proj_y_end = projection_map_y_.end();

    // iterate over projection matrix
    for (proj_y = projection_map_y_.begin(); proj_y !=proj_y_end; ++proj_y)
    {
      for (proj_x = projection_map_x_.begin(); proj_x !=proj_x_end; ++proj_x,
                                                                    ++point_idx,
                                                                    ++depth_img_ptr,
                                                                    cloud_shadow_buffer_ptr += point_step)
      {

        // lookup shadow depth
        float shadow_depth = shadow_depth_[point_idx];

        // check for time-outs
        if ( (shadow_depth!=0.0f) && (shadow_timestamp_[point_idx]<time_expire) )
        {
          // clear shadow pixel
          shadow_depth = shadow_depth_[point_idx] = 0.0f;
        }

        T depth_raw = *depth_img_ptr;
        if (DepthTraits<T>::valid(depth_raw))
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          // pointer to current point data
          float* cloud_data_pixel_ptr = cloud_data_ptr;

          // define point color
          uint32_t color;
          if (color_img_ptr)
          {
            color = *color_img_ptr;
          }
          else
          {
            color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
          }

          // fill in X,Y,Z and color
          *cloud_data_ptr = (*proj_x) * depth;  ++cloud_data_ptr;
          *cloud_data_ptr = (*proj_y) * depth;  ++cloud_data_ptr;
          *cloud_data_ptr = depth; ++cloud_data_ptr;
          *cloud_data_ptr = *reinterpret_cast<float*>(&color); ++cloud_data_ptr;

          ++point_count;

          // if shadow point exists -> display it
          if (depth < shadow_depth - shadow_distance_)
          {
            // copy point data from shadow buffer to point cloud
            memcpy(cloud_data_ptr, cloud_shadow_buffer_ptr, point_step);
            cloud_data_ptr += 4;
            ++point_count;
          }
          else
          {
            // save a copy of current point to shadow buffer
            memcpy(cloud_shadow_buffer_ptr, cloud_data_pixel_ptr, point_step);

            // reduce color intensity in shadow buffer
            RGBA* color = reinterpret_cast<RGBA*>(cloud_shadow_buffer_ptr + sizeof(float) * 3);
            color->red /= 2;
            color->green /= 2;
            color->blue /= 2;

            // update shadow depth & time out
            shadow_depth_[point_idx] = depth;
            shadow_timestamp_[point_idx] = time_now;
          }

        }
        else
        {
          // current depth pixel is invalid -> check shadow buffer
          if (shadow_depth != 0)
          {
            // copy shadow point to point cloud
            memcpy(cloud_data_ptr, cloud_shadow_buffer_ptr, point_step);
            cloud_data_ptr += 4;
            ++point_count;
          }
        }

        // increase color iterator pointer
        if (color_img_ptr)
          ++color_img_ptr;

      }
    }

    finalizingPointCloud(cloud_msg, point_count);

    return cloud_msg;
  }


template<typename T>
void MultiLayerDepth::convertColor(const sensor_msgs::ImageConstPtr& color_msg,
                                   std::vector<uint32_t>& rgba_color_raw)
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
    rgba_color_raw.clear();
    rgba_color_raw.reserve(num_pixel);

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

          rgba_color_raw.push_back((uint32_t)gray_value << 16 | (uint32_t)gray_value << 8 | (uint32_t)gray_value);
        }
        break;
      case 3:
      case 4:
        // rgb/bgr encoding
        for (i = 0; i < num_pixel; ++i)
        {
          uint8_t color1 = *((uint8_t*)img_ptr); img_ptr += sizeof(T);
          uint8_t color2 = *((uint8_t*)img_ptr); img_ptr += sizeof(T);
          uint8_t color3 = *((uint8_t*)img_ptr); img_ptr += sizeof(T);

          if (has_alpha)
            img_ptr += sizeof(T); // skip alpha values

          if (rgb_encoding)
          {
            // rgb encoding
            rgba_color_raw.push_back((uint32_t)color1 << 16 | (uint32_t)color2 << 8 | (uint32_t)color3 << 0);
          } else
          {
            // bgr encoding
            rgba_color_raw.push_back((uint32_t)color3 << 16 | (uint32_t)color2 << 8 | (uint32_t)color1 << 0);
          }
        }
        break;
      default:
        break;
    }

  }

sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudFromDepth(sensor_msgs::ImageConstPtr depth_msg,
                                                                         sensor_msgs::ImageConstPtr color_msg,
                                                                         sensor_msgs::CameraInfoConstPtr camera_info_msg)
{

  // Add data to multi depth image
  sensor_msgs::PointCloud2Ptr point_cloud_out;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(depth_msg->encoding);
  int numChannels = enc::numChannels(depth_msg->encoding);

  if (!camera_info_msg)
  {
    throw MultiLayerDepthException("Camera info missing!");
  }

  // precompute projection matrix and initialize shadow buffer
  initializeConversion(depth_msg, camera_info_msg);

  std::vector<uint32_t> rgba_color_raw_;

  if (color_msg)
  {
    if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height)
    {
      std::stringstream error_msg;
      error_msg << "Depth image resolution (" << (int)depth_msg->width << "x" << (int)depth_msg->height << ") "
          "does not match color image resolution (" << (int)color_msg->width << "x" << (int)color_msg->height << ")";
      throw( MultiLayerDepthException ( error_msg.str() ) );
    }

    // convert color coding to 8-bit rgb data
    switch (enc::bitDepth(color_msg->encoding))
    {
      case 8:
        convertColor<uint8_t>(color_msg, rgba_color_raw_);
        break;
      case 16:
        convertColor<uint16_t>(color_msg, rgba_color_raw_);
        break;
      default:
        std::string error_msg ("Color image has invalid bit depth");
        throw( MultiLayerDepthException (error_msg));
        break;
    }
  }

  if (!occlusion_compensation_)
  {
    // generate single layer depth cloud

    if ((bitDepth == 32) && (numChannels == 1))
    {
      // floating point encoded depth map
      point_cloud_out = generatePointCloudSL<float>(depth_msg, rgba_color_raw_);
    }
    else if ((bitDepth == 16) && (numChannels == 1))
    {
      // 32bit integer encoded depth map
      point_cloud_out = generatePointCloudSL<uint16_t>(depth_msg, rgba_color_raw_);
    }
  }
  else
  {
    // generate two layered depth cloud (depth+shadow)

    if ((bitDepth == 32) && (numChannels == 1))
    {
      // floating point encoded depth map
      point_cloud_out = generatePointCloudML<float>(depth_msg, rgba_color_raw_);
    }
    else if ((bitDepth == 16) && (numChannels == 1))
    {
      // 32bit integer encoded depth map
      point_cloud_out = generatePointCloudML<uint16_t>(depth_msg, rgba_color_raw_);
    }
  }

  if ( !point_cloud_out )
  {
    std::string error_msg ("Depth image has invalid format (only 16 bit and float are supported)!");
    throw( MultiLayerDepthException (error_msg));
  }

  return point_cloud_out;
}

sensor_msgs::PointCloud2Ptr MultiLayerDepth::initPointCloud()
{
  sensor_msgs::PointCloud2Ptr point_cloud_out = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

  point_cloud_out->fields.resize(4);
  std::size_t point_offset = 0;

  point_cloud_out->fields[0].name = "x";
  point_cloud_out->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[0].count = 1;
  point_cloud_out->fields[0].offset = point_offset;
  point_offset += sizeof(float);

  point_cloud_out->fields[1].name = "y";
  point_cloud_out->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[1].count = 1;
  point_cloud_out->fields[1].offset = point_offset;
  point_offset += sizeof(float);

  point_cloud_out->fields[2].name = "z";
  point_cloud_out->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[2].count = 1;
  point_cloud_out->fields[2].offset = point_offset;
  point_offset += sizeof(float);

  point_cloud_out->fields[3].name = "rgb";
  point_cloud_out->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[3].count = 1;
  point_cloud_out->fields[3].offset = point_offset;
  point_offset += sizeof(float);

  point_cloud_out->point_step = point_offset;

  point_cloud_out->is_bigendian = false;
  point_cloud_out->is_dense = false;

  return point_cloud_out;
}

void MultiLayerDepth::finalizingPointCloud(sensor_msgs::PointCloud2Ptr& point_cloud, std::size_t size)
{
  point_cloud->width = size;
  point_cloud->height = 1;
  point_cloud->data.resize(point_cloud->height * point_cloud->width * point_cloud->point_step);
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
}

}
