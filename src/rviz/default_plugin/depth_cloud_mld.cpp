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
  void MultiLayerDepth::updateMLDImage(const sensor_msgs::ImageConstPtr& depth_msg,
                                       const sensor_msgs::ImageConstPtr& color_msg,
                                       sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
                                       sensor_msgs::PointCloud2Ptr output_cloud)
  {

    global_time_stamp_ = ros::Time::now().toSec();

    std::size_t width = depth_msg->width;
    std::size_t height = depth_msg->height;


    output_cloud->data.resize((pixel_counter_+width*height) * output_cloud->point_step);
    float* point_cloud_data_ptr = reinterpret_cast<float*>(&output_cloud->data[0]);

    setSize(width*height);


    //////////////////////////////
    // Color conversion
    //////////////////////////////

    uint8_t* colorImgPtr = 0;
    std::vector<uint8_t> color_data;

    if (color_msg)
    {

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

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    std::size_t point_counter = 0;

    const T* img_ptr = (T*)&depth_msg->data[0];

    size_t idx = 0;

    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u, ++idx)
      {
        std::vector<DepthPixel*>& voxel_list = multilayer_depth_[idx];

        uint32_t color_r, color_g, color_b;

        if (colorImgPtr)
        {
          color_r = *colorImgPtr; ++colorImgPtr;
          color_g = *colorImgPtr; ++colorImgPtr;
          color_b = *colorImgPtr; ++colorImgPtr;
        }

        T depth_raw = *img_ptr;
        ++img_ptr;

        // Missing points denoted by NaNs
        if (DepthTraits<T>::valid(depth_raw) )
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          std::vector<DepthPixel*>::iterator ray_read = voxel_list.begin();
          std::vector<DepthPixel*>::iterator ray_write = voxel_list.begin();
          const std::vector<DepthPixel*>::const_iterator ray_end = voxel_list.end();

          std::size_t size = voxel_list.size();
          bool voxel_found= false;

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

                memcpy(point_cloud_data_ptr, depth_pixel->point_, sizeof(depth_pixel->point_));
                point_cloud_data_ptr += 3;
                *point_cloud_data_ptr = *reinterpret_cast<float*>(&depth_pixel->color_rgb_);
                ++point_cloud_data_ptr;
                ++point_counter;
                depth_pixel->time_out_ = global_time_stamp_+voxel_time_out_;
              } else
              {
                memcpy(point_cloud_data_ptr, depth_pixel->point_, sizeof(depth_pixel->point_));
                point_cloud_data_ptr += 3;

                *point_cloud_data_ptr = *reinterpret_cast<float*>(&depth_pixel->color_rgb_dark_);
                ++point_cloud_data_ptr;
                ++point_counter;
              }

              (*ray_write) = (*ray_read);

              ++ray_read;
              ++ray_write;
            }
          }
          voxel_list.resize(size);

          if (!voxel_found)
          {
            DepthPixel* depth_pixel = new DepthPixel();

            voxel_list.push_back(depth_pixel);

            depth_pixel->point_[0] = (u - center_x) * depth * constant_x;
            depth_pixel->point_[1] = (v - center_y) * depth * constant_y;
            depth_pixel->point_[2] = depth;

            pixel_counter_++;

            depth_pixel->time_out_ = global_time_stamp_+voxel_time_out_;

            if (colorImgPtr)
            {
              depth_pixel->color_rgb_dark_ = ((color_r/2) << 16 | (color_g/2) << 8 | (color_b/2));
              depth_pixel->color_rgb_ = (color_r << 16 | color_g << 8 | color_b);
            } else
            {
              depth_pixel->color_rgb_dark_ = ((256/2) << 16 | (256/2) << 8 | (256/2));
              depth_pixel->color_rgb_ = 0xFFFFFFFF;
            }

            memcpy(point_cloud_data_ptr, depth_pixel->point_, sizeof(depth_pixel->point_));
            point_cloud_data_ptr += 3;
            *point_cloud_data_ptr = *reinterpret_cast<float*>(&depth_pixel->color_rgb_);
            ++point_cloud_data_ptr;

            ++point_counter;
          }

        } else
        {
          std::vector<DepthPixel*>::iterator ray_read = voxel_list.begin();
          std::vector<DepthPixel*>::iterator ray_write = voxel_list.begin();
          const std::vector<DepthPixel*>::const_iterator ray_end = voxel_list.end();

          std::size_t size = voxel_list.size();

          while (ray_read != ray_end)
          {
            DepthPixel* depth_pixel = *ray_read;

            if ( depth_pixel->time_out_ < global_time_stamp_  )
            {
              delete (depth_pixel);
              --size;
              ++ray_read;

              --pixel_counter_;
            }
            else
            {
              memcpy(point_cloud_data_ptr, depth_pixel->point_, sizeof(depth_pixel->point_));
              point_cloud_data_ptr += 3;

              *point_cloud_data_ptr = *reinterpret_cast<float*>(&depth_pixel->color_rgb_dark_);
              ++point_cloud_data_ptr;

              (*ray_write) = (*ray_read);

              ++ray_read;
              ++ray_write;

              ++point_counter;
            }
          }
          voxel_list.resize(size);
        }
      }
    }


    output_cloud->data.resize(point_counter * output_cloud->point_step);

    output_cloud->width = point_counter;
    output_cloud->height = 1;
    output_cloud->row_step = output_cloud->point_step * output_cloud->width;

  }
/*
 *
template<typename T>
  void MultiLayerDepth::updateMLDImage(const sensor_msgs::ImageConstPtr& depth_msg,
                                       const sensor_msgs::ImageConstPtr& color_msg,
                                       sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
                                       sensor_msgs::PointCloud2Ptr output_cloud)
  {

    global_time_stamp_ = ros::Time::now().toSec();

    std::size_t width = depth_msg->width;
    std::size_t height = depth_msg->height;

    setSize(width*height);


    //////////////////////////////
    // Color conversion
    //////////////////////////////

    uint8_t* colorImgPtr = 0;
    std::vector<uint8_t> color_data;

    if (color_msg)
    {

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

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    const T* img_ptr = (T*)&depth_msg->data[0];

    size_t idx = 0;

    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u, ++idx)
      {


        uint32_t color_r, color_g, color_b;

        if (colorImgPtr)
        {
          color_r = *colorImgPtr; ++colorImgPtr;
          color_g = *colorImgPtr; ++colorImgPtr;
          color_b = *colorImgPtr; ++colorImgPtr;
        }

        T depth_raw = *img_ptr;
        ++img_ptr;

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

            depth_pixel->point_[0] = (u - center_x) * depth * constant_x;
            depth_pixel->point_[1] = (v - center_y) * depth * constant_y;
            depth_pixel->point_[2] = depth;

            pixel_counter_++;
          }

          multilayer_surface_cache_[idx] = depth_pixel;

          depth_pixel->time_out_ = global_time_stamp_+voxel_time_out_;

          if (colorImgPtr)
          {
            depth_pixel->color_[0] = color_r;
            depth_pixel->color_[1] = color_g;
            depth_pixel->color_[2] = color_b;

            depth_pixel->color_rgb_ = (color_r << 16 | color_g << 8 | color_b);
          } else
          {
            depth_pixel->color_[0] = 255.0f;
            depth_pixel->color_[1] = 255.0f;
            depth_pixel->color_[2] = 255.0f;

            depth_pixel->color_rgb_ = 0xFFFFFFFF;
          }



        }
      }
    }

  }
 *
 */
sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloudFromMLDepth()
{

  // Add data to multi depth image
   sensor_msgs::PointCloud2Ptr point_cloud_out = initNewPointCloud();
  {
    boost::mutex::scoped_lock lock(input_update_mutex_);

    // Bit depth of image encoding
    int bitDepth = enc::bitDepth(depth_image_->encoding);
    int numChannels = enc::numChannels(depth_image_->encoding);



    if (camera_info_)
    {
      if ((bitDepth == 32) && (numChannels == 1))
      {
        // floating point encoded depth map
        updateMLDImage<float>(depth_image_, color_image_, camera_info_, point_cloud_out);
      }
      else if ((bitDepth == 16) && (numChannels == 1))
      {
        // 32bit integer encoded depth map
        updateMLDImage<uint16_t>(depth_image_, color_image_, camera_info_, point_cloud_out);
      }
      else
      {
        std::string error_msg(
            "Depth input image must be encoded in 32bit floating point format or 16bit integer format (input format is: "
                + depth_image_->encoding + ")");
        throw(MultiLayerDepthException(error_msg));
      }
    }
  }

// Generate output point cloud

  std::size_t point_counter = 0;
/*


  point_cloud_out->data.resize(pixel_counter_ * point_cloud_out->point_step);
  float* point_cloud_data_ptr = reinterpret_cast<float*>(&point_cloud_out->data[0]);


  std::vector<std::vector<DepthPixel*> >::iterator it;
  std::vector<std::vector<DepthPixel*> >::const_iterator it_end = multilayer_depth_.end();

  for (it = multilayer_depth_.begin(); it != it_end; ++it)
  {
    std::vector<DepthPixel*>& voxel_list = *it;

    std::vector<DepthPixel*>::iterator ray_read = voxel_list.begin();
    const std::vector<DepthPixel*>::const_iterator ray_end = voxel_list.end();

    while (ray_read != ray_end)
    {
      DepthPixel* depth_pixel = *ray_read;

      memcpy(point_cloud_data_ptr, depth_pixel->point_, sizeof(depth_pixel->point_));
      point_cloud_data_ptr += 3;

      uint32_t color_rgb;

      if (depth_pixel->time_out_ == global_time_stamp_ + voxel_time_out_)
      {
        color_rgb = depth_pixel->color_rgb_;
      }
      else
      {
        uint32_t color_r = (uint32_t)(depth_pixel->color_[0] * color_filter_) & 0xFF;
        uint32_t color_g = (uint32_t)(depth_pixel->color_[1] * color_filter_) & 0xFF;
        uint32_t color_b = (uint32_t)(depth_pixel->color_[2] * color_filter_) & 0xFF;
        color_rgb = (color_r << 16 | color_g << 8 | color_b);
      }

      *point_cloud_data_ptr = *reinterpret_cast<float*>(&color_rgb);
      ++point_cloud_data_ptr;

      ++ray_read;
      ++point_counter;
    }
  }


  point_cloud_out->data.resize(point_counter * point_cloud_out->point_step);

  point_cloud_out->width = point_counter;
  point_cloud_out->height = 1;
  point_cloud_out->row_step = point_cloud_out->point_step * point_cloud_out->width;
*/
  std::cout<<point_counter<<std::endl;

  return point_cloud_out;
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
      if ((bitDepth == 32) && (numChannels == 1))
      {
        // floating point encoded depth map
        point_cloud_out = generatePointCloud<float>(depth_image_, color_image_, camera_info_);
      }
      else if ((bitDepth == 16) && (numChannels == 1))
      {
        // 32bit integer encoded depth map
        point_cloud_out = generatePointCloud<uint16_t>(depth_image_, color_image_, camera_info_);
      }
      else
      {
        std::string error_msg(
            "Depth input image must be encoded in 32bit floating point format or 16bit integer format (input format is: "
                + depth_image_->encoding + ")");
        throw(MultiLayerDepthException(error_msg));
      }
    }
  }

  return point_cloud_out;
}

template<typename T>
  sensor_msgs::PointCloud2Ptr MultiLayerDepth::generatePointCloud(const sensor_msgs::ImageConstPtr depth_msg,
                                                                  const sensor_msgs::ImageConstPtr color_msg,
                                                                  sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
  {

    int width = depth_image_->width;
    int height = depth_image_->height;

    //////////////////////////////
    // initialize cloud message
    //////////////////////////////

    sensor_msgs::PointCloud2Ptr cloud_msg = initNewPointCloud();

    cloud_msg->data.resize(height * width * cloud_msg->point_step);

    //////////////////////////////
    // Update camera model
    //////////////////////////////

    if (!camera_info_)
    {
      std::string error_msg ("Waiting for CameraInfo message..");
      throw( MultiLayerDepthException (error_msg));
    }

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

    //////////////////////////////
    // Color conversion
    //////////////////////////////

    uint8_t* colorImgPtr = 0;
    std::vector<uint8_t> color_data;

    if (color_image_)
    {

      // convert color coding to 8-bit rgb data
      switch (enc::bitDepth(color_image_->encoding))
      {
        case 8:
          convertColor<uint8_t>(color_image_, color_data);
          break;
        case 16:
          convertColor<uint16_t>(color_image_, color_data);
          break;
        default:
          std::string error_msg ("Color image has invalid bit depth");
          throw( MultiLayerDepthException (error_msg));
          break;
      }

      colorImgPtr = &color_data[0];
    }

    ////////////////////////////////////////////////
    // depth map to point cloud conversion
    ////////////////////////////////////////////////

    float* cloudDataPtr = reinterpret_cast<float*>(&cloud_msg->data[0]);

    std::size_t pointCount_ = 0;

    const T* img_ptr = (T*)&depth_image_->data[0];
    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u)
      {
        uint8_t color_r, color_g, color_b;

        if (colorImgPtr)
        {
          color_r = *colorImgPtr;
          ++colorImgPtr;
          color_g = *colorImgPtr;
          ++colorImgPtr;
          color_b = *colorImgPtr;
          ++colorImgPtr;
        }

        T depth_raw = *img_ptr;
        img_ptr++;

        // Missing points denoted by NaNs
        if (DepthTraits<T>::valid(depth_raw))
        {
          float depth = DepthTraits<T>::toMeters(depth_raw);

          // Fill in XYZ
          *cloudDataPtr = (u - center_x) * depth * constant_x;
          ++cloudDataPtr;
          *cloudDataPtr = (v - center_y) * depth * constant_y;
          ++cloudDataPtr;
          *cloudDataPtr = depth;
          ++cloudDataPtr;

          ++pointCount_;

          if (colorImgPtr)
          {
            uint32_t color_rgb = ((uint32_t)color_r << 16 | (uint32_t)color_g << 8 | (uint32_t)color_b);
            *cloudDataPtr = *reinterpret_cast<float*>(&color_rgb);
            ++cloudDataPtr;
          }
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

    return cloud_msg;

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

sensor_msgs::PointCloud2Ptr MultiLayerDepth::initNewPointCloud()
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

}
