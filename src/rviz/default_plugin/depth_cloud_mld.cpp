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


template<typename T>
void MultiLayerDepth::convertColor(const sensor_msgs::ImageConstPtr& color_msg,
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
  void MultiLayerDepth::processInputImageData(const sensor_msgs::ImageConstPtr& depth_msg,
                                              const sensor_msgs::ImageConstPtr& color_msg,
                                              sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
  {
    std::size_t width = depth_msg->width;
    std::size_t height = depth_msg->height;

    setSize(width*height);

    //////////////////////////////
    // Update camera model
    //////////////////////////////

    image_geometry::PinholeCameraModel cameraModel;

    cameraModel.fromCameraInfo(camera_info_msg);

    // Use correct principal point from calibration
    float center_x = cameraModel.cx();
    float center_y = cameraModel.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float constant_x = 1.0f / cameraModel.fx();
    float constant_y = 1.0f / cameraModel.fy();

    //////////////////////////////
    // Color conversion
    //////////////////////////////

    uint8_t* colorImgPtr = 0;
    std::vector<uint8_t> color_data;

    if (color_msg)
    {

      if (depth_msg->header.frame_id != color_msg->header.frame_id)
      {
        std::string error_msg ("Depth image frame id [" +
                               depth_msg->header.frame_id +
                               "] doesn't match RGB image frame id [" +
                               color_msg->header.frame_id +
                               "]");
        throw( MultiLayerDepthException (error_msg));
      }

      if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height)
      {
       std::stringstream error_msg;
       error_msg << "Depth resolution ("
                 << (int) depth_msg->width << "x" << (int) depth_msg->height
                 << ") does not match RGB resolution ("
                 << (int) color_msg->width << "x" << (int) color_msg->height
                 << ")";
        throw( MultiLayerDepthException (error_msg.str()));
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
          std::string error_msg ("Color image has invalid bit depth.");
          throw( MultiLayerDepthException (error_msg));
          break;
      }
      colorImgPtr = &color_data[0];
    }

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    const float* cloudDataPtr = reinterpret_cast<const float*>(&depth_msg->data[0]);

    const T* img_ptr = (T*)&depth_msg->data[0];

    size_t idx = 0;

    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u, ++idx)
      {

        float color_r, color_g, color_b;

        if (colorImgPtr)
        {
          color_r = *colorImgPtr; ++colorImgPtr;
          color_g = *colorImgPtr; ++colorImgPtr;
          color_b = *colorImgPtr; ++colorImgPtr;
        }

        T depth_raw = *img_ptr++;

        // Missing points denoted by NaNs
        if (DepthTraits<T>::valid(depth_raw) )
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          DepthPixel* depth_pixel = 0;

          if (fabs(multilayer_depth_cache_[idx]-depth)<voxel_resolution_)
          {
            depth_pixel = multilayer_surface_cache_[idx];
          } else
          {
            depth_pixel = processRay(idx, depth);
            multilayer_depth_cache_[idx] = depth;
          }

          if (!depth_pixel)
          {
            depth_pixel = new DepthPixel();

            std::vector<DepthPixel*>& voxel_list = multilayer_depth_[idx];
            voxel_list.push_back(depth_pixel);

            pixel_counter_++;
          }

          multilayer_surface_cache_[idx] = depth_pixel;

          depth_pixel->x_ = (u - center_x) * depth * constant_x;
          depth_pixel->y_ = (v - center_y) * depth * constant_y;
          depth_pixel->z_ = depth;

          depth_pixel->time_out_ = voxel_time_out_;

          if (colorImgPtr)
          {
            depth_pixel->color_r_ = color_r;
            depth_pixel->color_g_ = color_g;
            depth_pixel->color_b_ = color_b;
          } else
          {
            depth_pixel->color_r_ = depth_pixel->color_g_ = depth_pixel->color_b_ = 255.0f;
          }

        }
      }
    }

  }

void  MultiLayerDepth::addDepthColorCameraInfo(const sensor_msgs::ImageConstPtr& depth_msg,
                                               const sensor_msgs::ImageConstPtr& color_msg,
                                               sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(depth_msg->encoding);
  int numChannels = enc::numChannels(depth_msg->encoding);

  if (camera_info_msg)
  {
    if ((bitDepth == 32) && (numChannels == 1))
    {
      // floating point encoded depth map
      processInputImageData<float>(depth_msg, color_msg, camera_info_msg);
    }
    else if ((bitDepth == 16) && (numChannels == 1))
    {
      // 32bit integer encoded depth map
      processInputImageData<uint16_t>(depth_msg, color_msg, camera_info_msg);
    }
    else
    {
      std::string error_msg ("Depth input image must be encoded in 32bit floating point format or 16bit integer format (input format is: " +
                             depth_msg->encoding +
                             ")");
      throw( MultiLayerDepthException (error_msg));
    }
  }

}

sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloud() const
{

  sensor_msgs::PointCloud2Ptr point_cloud_out = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

  point_cloud_out->fields.resize(4);
  std::size_t point_offset = 0;

  point_cloud_out->fields[0].name = "x";
  point_cloud_out->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[0].count = 1;
  point_offset += sizeof(float);

  point_cloud_out->fields[1].name = "y";
  point_cloud_out->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[1].count = 1;
  point_offset += sizeof(float);

  point_cloud_out->fields[2].name = "z";
  point_cloud_out->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[2].count = 1;
  point_offset += sizeof(float);

  point_cloud_out->fields[3].name = "rgb";
  point_cloud_out->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  point_cloud_out->fields[3].count = 1;
  point_offset += sizeof(float);

  point_cloud_out->point_step = point_offset;

  point_cloud_out->data.resize(pixel_counter_ * point_offset);
  point_cloud_out->is_bigendian = false;
  point_cloud_out->is_dense = false;

  point_cloud_out->width = pixel_counter_;
  point_cloud_out->height = 1;
  point_cloud_out->row_step = point_cloud_out->point_step * point_cloud_out->width;

  std::vector<std::vector<DepthPixel*> >::const_iterator it;
  std::vector<std::vector<DepthPixel*> >::const_iterator it_end = multilayer_depth_.end();

  float* point_cloud_data_ptr = reinterpret_cast<float*>(&point_cloud_out->data[0]);

  for (it = multilayer_depth_.begin(); it != it_end; ++it)
  {
    const std::vector<DepthPixel*>& voxel_list = *it;

    std::vector<DepthPixel*>::const_iterator it_layer;
    std::vector<DepthPixel*>::const_iterator it_layer_end = voxel_list.end();

    static uint32_t shadow_color = ((uint32_t)100 << 16 | (uint32_t)100 << 8 | (uint32_t)100);

    for (it_layer = voxel_list.begin(); it_layer != it_layer_end; ++it_layer)
    {
      DepthPixel* depth_pixel = *it_layer;

      depth_pixel->time_out_ = std::min(depth_pixel->time_out_, voxel_time_out_);

      if (depth_pixel->time_out_)
      {
        *point_cloud_data_ptr = depth_pixel->x_; ++point_cloud_data_ptr;
        *point_cloud_data_ptr = depth_pixel->y_; ++point_cloud_data_ptr;
        *point_cloud_data_ptr = depth_pixel->z_; ++point_cloud_data_ptr;

        uint32_t color_r = depth_pixel->color_r_ * color_filter_;
        uint32_t color_g = depth_pixel->color_g_ * color_filter_;
        uint32_t color_b = depth_pixel->color_b_ * color_filter_;

        uint32_t color_rgb = (color_r << 16 | color_g << 8 | color_b);
        *point_cloud_data_ptr = *reinterpret_cast<float*>(&color_rgb);
        ++point_cloud_data_ptr;

        --depth_pixel->time_out_;
      }
    }

    return point_cloud_out;
  }
}



}
