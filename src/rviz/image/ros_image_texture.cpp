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

#include <map>
#include <sstream>
#include <algorithm>
#include <iostream>

#include <boost/algorithm/string/erase.hpp>
#include <boost/foreach.hpp>

#include <OgreTextureManager.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rviz/image/ros_image_texture.h"

namespace rviz
{

ROSImageTexture::ROSImageTexture()
: new_image_(false)
, width_(0)
, height_(0)
, median_frames_(5)
, target_frame_("")
{
  empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  tf_sub_.reset(new tf2_ros::TransformListener(tf_buffer_));

  static uint32_t count = 0;
  std::stringstream ss;
  ss << "ROSImageTexture" << count++;
  texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, empty_image_, Ogre::TEX_TYPE_2D, 0);

  setNormalizeFloatImage(true);
}

ROSImageTexture::~ROSImageTexture()
{
  current_image_.reset();
}

void ROSImageTexture::clear()
{
  boost::mutex::scoped_lock lock(mutex_);

  texture_->unload();
  texture_->loadImage(empty_image_);

  new_image_ = false;
  current_image_.reset();
}

const sensor_msgs::Image::ConstPtr& ROSImageTexture::getImage()
{
  boost::mutex::scoped_lock lock(mutex_);

  return current_image_;
}

void ROSImageTexture::setMedianFrames( unsigned median_frames )
{
  median_frames_ = median_frames;
}

void ROSImageTexture::setRotateImageFrame(std::string target_frame)
{
  target_frame_ = target_frame;
}

double ROSImageTexture::updateMedian( std::deque<double>& buffer, double value )
{
  //update buffer
  while(buffer.size() > median_frames_-1)
  {
    buffer.pop_back();
  }
  buffer.push_front(value);
  // get median
  std::deque<double> buffer2 = buffer;
  std::nth_element( buffer2.begin(), buffer2.begin()+buffer2.size()/2, buffer2.end() );
  return *( buffer2.begin()+buffer2.size()/2 );
}

void ROSImageTexture::setNormalizeFloatImage( bool normalize, double min, double max )
{
  normalize_ = normalize;
  min_ = min;
  max_ = max;
}


template<typename T>
void ROSImageTexture::normalize( T* image_data, size_t image_data_size, std::vector<uint8_t> &buffer  )
{
  // Prepare output buffer
  buffer.resize(image_data_size, 0);

  T minValue;
  T maxValue;

  if ( normalize_ )
  {
    T* input_ptr = image_data;
    // Find min. and max. pixel value
    minValue = std::numeric_limits<T>::max();
    maxValue = std::numeric_limits<T>::min();
    for( unsigned i = 0; i < image_data_size; ++i )
    {
      minValue = std::min( minValue, *input_ptr );
      maxValue = std::max( maxValue, *input_ptr );
      input_ptr++;
    }

    if ( median_frames_ > 1 )
    {
      minValue = updateMedian( min_buffer_, minValue );
      maxValue = updateMedian( max_buffer_, maxValue );
    }
  }
  else
  {
    // set fixed min/max
    minValue = min_;
    maxValue = max_;
  }

  // Rescale floating point image and convert it to 8-bit
  double range = maxValue - minValue;
  if( range > 0.0 )
  {
    T* input_ptr = image_data;

    // Pointer to output buffer
    uint8_t* output_ptr = &buffer[0];

    // Rescale and quantize
    for( size_t i = 0; i < image_data_size; ++i, ++output_ptr, ++input_ptr )
    {
      double val = (double(*input_ptr - minValue) / range);
      if ( val < 0 ) val = 0;
      if ( val > 1 ) val = 1;
      *output_ptr = val * 255u;
    }
  }
}

sensor_msgs::Image::Ptr ROSImageTexture::rotateImage(sensor_msgs::Image::ConstPtr msg)
{
  // Convert the image into something opencv can handle.
  cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

  // Compute the output image size.
  int max_dim = in_image.cols > in_image.rows ? in_image.cols : in_image.rows;
  int min_dim = in_image.cols < in_image.rows ? in_image.cols : in_image.rows;
  int noblack_dim = min_dim / sqrt(2);
  int diag_dim = sqrt(in_image.cols*in_image.cols + in_image.rows*in_image.rows);
  int candidates[] = {noblack_dim, min_dim, max_dim, diag_dim, diag_dim}; // diag_dim repeated to simplify limit case.
  int output_image_size = 2;
  int step = output_image_size;
  int out_size = candidates[step] + (candidates[step + 1] - candidates[step]) * (output_image_size - step);

  geometry_msgs::Vector3Stamped target_vector;
  target_vector.header.stamp = msg->header.stamp;
  target_vector.header.frame_id = target_frame_;
  target_vector.vector.z = 1;
  if (!target_frame_.empty())
  {
    // Transform the target vector into the image frame.
    try
    {
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp);
      tf2::doTransform(target_vector, target_vector, transform);
    }
    catch (tf2::LookupException& e)
    {
      ROS_ERROR_THROTTLE(30, "Error rotating image: %s", e.what());
    }
  }

  // Transform the source vector into the image frame.
  geometry_msgs::Vector3Stamped source_vector;
  source_vector.header.stamp = msg->header.stamp;
  source_vector.header.frame_id = msg->header.frame_id;
  source_vector.vector.y = -1;
  if (!msg->header.frame_id.empty())
  {
    try
    {
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        msg->header.frame_id, msg->header.frame_id, msg->header.stamp);
      tf2::doTransform(source_vector, source_vector, transform);
    }
    catch (tf2::LookupException& e)
    {
      ROS_ERROR_THROTTLE(30, "Error rotating image: %s", e.what());
    }
  }

  // Calculate the angle of the rotation.
  double angle = 0;
  if ((target_vector.vector.x != 0 || target_vector.vector.y != 0) &&
      (source_vector.vector.x != 0 || source_vector.vector.y != 0))
  {
    angle = atan2(target_vector.vector.y, target_vector.vector.x);
    angle -= atan2(source_vector.vector.y, source_vector.vector.x);
  }

  // Compute the rotation matrix.
  cv::Mat rot_matrix = cv::getRotationMatrix2D(cv::Point2f(in_image.cols / 2.0, in_image.rows / 2.0), 180 * angle / M_PI, 1);
  cv::Mat translation = rot_matrix.col(2);
  rot_matrix.at<double>(0, 2) += (out_size - in_image.cols) / 2.0;
  rot_matrix.at<double>(1, 2) += (out_size - in_image.rows) / 2.0;

  // Do the rotation
  cv::Mat out_image;
  cv::warpAffine(in_image, out_image, rot_matrix, cv::Size(out_size, out_size));

  sensor_msgs::Image::Ptr dst_msg = cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
  dst_msg->header.frame_id = target_frame_;
  return dst_msg;
}

bool ROSImageTexture::update()
{
  sensor_msgs::Image::ConstPtr raw_image;
  bool new_image = false;
  {
    boost::mutex::scoped_lock lock(mutex_);

    raw_image = current_image_;
    new_image = new_image_;
  }

  if (!raw_image || !new_image)
  {
    return false;
  }

  new_image_ = false;

  if (raw_image->data.empty())
  {
    return false;
  }

  sensor_msgs::Image::Ptr image = rotateImage(raw_image);

  Ogre::PixelFormat format = Ogre::PF_R8G8B8;
  Ogre::Image ogre_image;
  std::vector<uint8_t> buffer;

  uint8_t* imageDataPtr = (uint8_t*)&image->data[0];
  size_t imageDataSize = image->data.size();

  if (image->encoding == sensor_msgs::image_encodings::RGB8)
  {
    format = Ogre::PF_BYTE_RGB;
  }
  else if (image->encoding == sensor_msgs::image_encodings::RGBA8)
  {
    format = Ogre::PF_BYTE_RGBA;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
           image->encoding == sensor_msgs::image_encodings::BGRA8)
  {
    format = Ogre::PF_BYTE_BGRA;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
           image->encoding == sensor_msgs::image_encodings::BGR8)
  {
    format = Ogre::PF_BYTE_BGR;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
           image->encoding == sensor_msgs::image_encodings::MONO8)
  {
    format = Ogre::PF_BYTE_L;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
           image->encoding == sensor_msgs::image_encodings::MONO16)
  {
    imageDataSize /= sizeof(uint16_t);
    normalize<uint16_t>( (uint16_t*)&image->data[0], imageDataSize, buffer );
    format = Ogre::PF_BYTE_L;
    imageDataPtr = &buffer[0];
  }
  else if (image->encoding.find("bayer") == 0)
  {
    format = Ogre::PF_BYTE_L;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    imageDataSize /= sizeof(float);
    normalize<float>( (float*)&image->data[0], imageDataSize, buffer );
    format = Ogre::PF_BYTE_L;
    imageDataPtr = &buffer[0];
  }
  else
  {
    throw UnsupportedImageEncoding(image->encoding);
  }

  width_ = image->width;
  height_ = image->height;

  // TODO: Support different steps/strides

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(imageDataPtr, imageDataSize));

  try
  {
    ogre_image.loadRawData(pixel_stream, width_, height_, 1, format, 1, 0);
  }
  catch (Ogre::Exception& e)
  {
    // TODO: signal error better
    ROS_ERROR("Error loading image: %s", e.what());
    return false;
  }

  texture_->unload();
  texture_->loadImage(ogre_image);

  return true;
}

void ROSImageTexture::addMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  current_image_ = msg;
  new_image_ = true;
}

} // end of namespace rviz
