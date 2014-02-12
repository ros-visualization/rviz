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

#ifndef RVIZ_ROS_IMAGE_TEXTURE_H
#define RVIZ_ROS_IMAGE_TEXTURE_H

#include <sensor_msgs/Image.h>

#include <OgreTexture.h>
#include <OgreImage.h>
#include <OgreSharedPtr.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <stdexcept>

namespace rviz
{

class UnsupportedImageEncoding : public std::runtime_error
{
public:
  UnsupportedImageEncoding(const std::string& encoding)
  : std::runtime_error("Unsupported image encoding [" + encoding + "]")
  {}
};

class ROSImageTexture
{
public:
  ROSImageTexture();
  ~ROSImageTexture();

  void addMessage(const sensor_msgs::Image::ConstPtr& image);
  bool update();
  void clear();

  const Ogre::TexturePtr& getTexture() { return texture_; }
  const sensor_msgs::Image::ConstPtr& getImage();

  uint32_t getWidth() { return width_; }
  uint32_t getHeight() { return height_; }

  // automatic range normalization
  void setNormalizeFloatImage( bool normalize, double min=0.0, double max=1.0 );
  void setMedianFrames( unsigned median_frames );

private:

  double updateMedian( std::deque<double>& buffer, double new_value );

  template<typename T>
  void normalize( T* image_data, size_t image_data_size, std::vector<uint8_t> &buffer  );

  sensor_msgs::Image::ConstPtr current_image_;
  boost::mutex mutex_;
  bool new_image_;

  Ogre::TexturePtr texture_;
  Ogre::Image empty_image_;

  uint32_t width_;
  uint32_t height_;

  // fields for float image running median computation
  bool normalize_;
  double min_;
  double max_;
  unsigned median_frames_;
  std::deque<double> min_buffer_;
  std::deque<double> max_buffer_;
};

}

#endif
