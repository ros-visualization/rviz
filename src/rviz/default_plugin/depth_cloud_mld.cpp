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


struct RGBA
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha;
};


void MultiLayerDepth::initializeConversion()
{

  if (!depth_image_ || !camera_info_)
  {
    std::string error_msg ("Waiting for CameraInfo message..");
    throw( MultiLayerDepthException (error_msg));
  }

  int width = depth_image_->width;
  int height = depth_image_->height;

  std::size_t size = width * height;

  if (size != multilayer_depth_cache_.size())
  {
    multilayer_depth_cache_.resize(size, 0.0f);
    multilayer_depth_timeout_.resize(size, 0.0);

    point_shadow_cache_.resize(size * (4 * sizeof(float)), 0);

    projectionMapX_.resize(width);
    projectionMapY_.resize(height);

    //////////////////////////////
    // Update camera model
    //////////////////////////////

    // The following computation of center_x,y and fx,fy duplicates
    // code in the image_geometry package, but this avoids dependency
    // on OpenCV, which simplifies releasing rviz.

    // Use correct principal point from calibration
    float center_x = camera_info_->P[2] - camera_info_->roi.x_offset;
    float center_y = camera_info_->P[6] - camera_info_->roi.y_offset;

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double scale_x = camera_info_->binning_x > 1 ? (1.0 / camera_info_->binning_x) : 1.0;
    double scale_y = camera_info_->binning_y > 1 ? (1.0 / camera_info_->binning_y) : 1.0;

    double fx = camera_info_->P[0] * scale_x;
    double fy = camera_info_->P[5] * scale_y;

    float constant_x = 1.0f / fx;
    float constant_y = 1.0f / fy;

    std::vector<float>::iterator projX = projectionMapX_.begin();
    std::vector<float>::iterator projY = projectionMapY_.begin();

    for (int v = 0; v < height; ++v, ++projY)
      *projY = (v - center_y) * constant_y;

    for (int u = 0; u < width; ++u, ++projX)
      *projX = (u - center_x) * constant_x;

    reset();
  }
}


template<typename T>
  sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudSL(const sensor_msgs::ImageConstPtr& depth_msg,
                                                                    std::vector<uint32_t>& rgba_color_raw)
  {

    int width = depth_image_->width;
    int height = depth_image_->height;

    sensor_msgs::PointCloud2Ptr cloud_msg = initPointCloud();
    cloud_msg->data.resize(height * width * cloud_msg->point_step);

    uint32_t* colorImgPtr = 0;

    if (rgba_color_raw.size())
      colorImgPtr = &rgba_color_raw[0];

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    float* cloudDataPtr = reinterpret_cast<float*>(&cloud_msg->data[0]);
    const std::size_t point_step = cloud_msg->point_step;

    std::size_t pointCount = 0;
    std::size_t idx = 0;

    double time_now = ros::Time::now().toSec();
    double time_expires = time_now+voxel_time_out_;

    const T* img_ptr = (T*)&depth_msg->data[0];

    std::vector<float>::iterator projX;
    std::vector<float>::const_iterator projX_end = projectionMapX_.end();
    std::vector<float>::iterator projY ;
    std::vector<float>::const_iterator projY_end = projectionMapY_.end();

    for (projY = projectionMapY_.begin(); projY !=projY_end; ++projY)
    {
      for (projX = projectionMapX_.begin(); projX !=projX_end; ++projX, ++idx)
      {

        T depth_raw = *img_ptr;
        if (DepthTraits<T>::valid(depth_raw))
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          uint32_t color;
          if (colorImgPtr)
          {
            color = *colorImgPtr;
          }
          else
          {
            color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
          }

          *cloudDataPtr = (*projX) * depth;  ++cloudDataPtr;
          *cloudDataPtr = (*projY) * depth;  ++cloudDataPtr;
          *cloudDataPtr = depth; ++cloudDataPtr;
          *cloudDataPtr = *reinterpret_cast<float*>(&color); ++cloudDataPtr;

          ++pointCount;
        }

        if (colorImgPtr)
          ++colorImgPtr;

        ++img_ptr;
      }
    }

    finalizingPointCloud(cloud_msg, pointCount);

    return cloud_msg;

  }


template<typename T>
  sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudML(const sensor_msgs::ImageConstPtr& depth_msg,
                                                                    std::vector<uint32_t>& rgba_color_raw)
  {
    int width = depth_image_->width;
    int height = depth_image_->height;

    sensor_msgs::PointCloud2Ptr cloud_msg = initPointCloud();
    cloud_msg->data.resize(height * width * cloud_msg->point_step * 2);

    uint32_t* colorImgPtr = 0;

    if (rgba_color_raw.size())
      colorImgPtr = &rgba_color_raw[0];

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    float* cloudDataPtr = reinterpret_cast<float*>(&cloud_msg->data[0]);
    uint8_t* cloudShadowCachePtr = reinterpret_cast<uint8_t*>(&point_shadow_cache_[0]);

    const std::size_t point_step = cloud_msg->point_step;

    std::size_t pointCount = 0;
    std::size_t idx = 0;

    double time_now = ros::Time::now().toSec();
    double time_expires = time_now+voxel_time_out_;

    const T* img_ptr = (T*)&depth_msg->data[0];

    std::vector<float>::iterator projX;
    std::vector<float>::const_iterator projX_end = projectionMapX_.end();
    std::vector<float>::iterator projY ;
    std::vector<float>::const_iterator projY_end = projectionMapY_.end();

    for (projY = projectionMapY_.begin(); projY !=projY_end; ++projY)
    {
      for (projX = projectionMapX_.begin(); projX !=projX_end; ++projX, ++idx)
      {

        float deptch_cache = multilayer_depth_cache_[idx];

        if ( (deptch_cache!=0.0f) && (multilayer_depth_timeout_[idx]<time_now) )
        {
          multilayer_depth_cache_[idx] = 0.0f;
          deptch_cache = 0.0f;
        }

        T depth_raw = *img_ptr;
        if (DepthTraits<T>::valid(depth_raw))
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          float* cloudDataPixelPtr = cloudDataPtr;

          uint32_t color;
          if (colorImgPtr)
          {
            color = *colorImgPtr;
          }
          else
          {
            color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
          }

          *cloudDataPtr = (*projX) * depth;  ++cloudDataPtr;
          *cloudDataPtr = (*projY) * depth;  ++cloudDataPtr;
          *cloudDataPtr = depth; ++cloudDataPtr;
          *cloudDataPtr = *reinterpret_cast<float*>(&color); ++cloudDataPtr;

          ++pointCount;

          if (depth < deptch_cache - voxel_resolution_)
          {
            memcpy(cloudDataPtr, cloudShadowCachePtr, point_step);
            cloudDataPtr += 4;
            ++pointCount;
          }
          else
          {
            memcpy(cloudShadowCachePtr, cloudDataPixelPtr, point_step);

            RGBA* color = reinterpret_cast<RGBA*>(cloudShadowCachePtr + sizeof(float) * 3);
            color->red /= 2;
            color->green /= 2;
            color->blue /= 2;

            multilayer_depth_cache_[idx] = depth;
            multilayer_depth_timeout_[idx] = time_expires;
          }

        }
        else
        {
          if (deptch_cache != 0)
          {
            memcpy(cloudDataPtr, cloudShadowCachePtr, point_step);
            cloudDataPtr += 4;
            ++pointCount;
          }
        }

        cloudShadowCachePtr += point_step;

        if (colorImgPtr)
          ++colorImgPtr;

        ++img_ptr;

      }
    }

    finalizingPointCloud(cloud_msg, pointCount);

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

    // prepare output vectors
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
            rgba_color_raw.push_back((uint32_t)color1 << 16 | (uint32_t)color2 << 8 | (uint32_t)color3 << 0);
          } else
          {
            rgba_color_raw.push_back((uint32_t)color3 << 16 | (uint32_t)color2 << 8 | (uint32_t)color1 << 0);
          }
        }
        break;
      default:
        break;
    }

  }

sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudFromDepth()
{

  // Add data to multi depth image
  sensor_msgs::PointCloud2Ptr point_cloud_out;

  {
    boost::mutex::scoped_lock lock(input_update_mutex_);

    // Bit depth of image encoding
    int bitDepth = enc::bitDepth(depth_image_->encoding);
    int numChannels = enc::numChannels(depth_image_->encoding);

    if (camera_info_)
    {

      initializeConversion();

      std::vector<uint32_t> rgba_color_raw_;

      if (color_image_)
      {

        // convert color coding to 8-bit rgb data
        switch (enc::bitDepth(color_image_->encoding))
        {
          case 8:
            convertColor<uint8_t>(color_image_, rgba_color_raw_);
            break;
          case 16:
            convertColor<uint16_t>(color_image_, rgba_color_raw_);
            break;
          default:
            std::string error_msg ("Color image has invalid bit depth");
            throw( MultiLayerDepthException (error_msg));
            break;
        }
      }

      if (!occlusion_compensation_)
      {
        if ((bitDepth == 32) && (numChannels == 1))
        {
          // floating point encoded depth map
          point_cloud_out = generatePointCloudSL<float>(depth_image_, rgba_color_raw_);
        }
        else if ((bitDepth == 16) && (numChannels == 1))
        {
          // 32bit integer encoded depth map
          point_cloud_out = generatePointCloudSL<uint16_t>(depth_image_, rgba_color_raw_);
        }
      } else
      {
        if ((bitDepth == 32) && (numChannels == 1))
        {
          // floating point encoded depth map
          point_cloud_out = generatePointCloudML<float>(depth_image_, rgba_color_raw_);
        }
        else if ((bitDepth == 16) && (numChannels == 1))
        {
          // 32bit integer encoded depth map
          point_cloud_out = generatePointCloudML<uint16_t>(depth_image_, rgba_color_raw_);
        }
      }
    }
  }

  return point_cloud_out;
}

void  MultiLayerDepth::addDepthColorCameraInfo(const sensor_msgs::ImageConstPtr& depth_msg,
                                               const sensor_msgs::ImageConstPtr& color_msg,
                                               sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  boost::mutex::scoped_lock lock(input_update_mutex_);

  depth_image_ = depth_msg;
  color_image_ = color_msg;
  camera_info_ = camera_info_msg;
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
