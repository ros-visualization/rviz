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

#include <OGRE/OgreTextureManager.h>

#include <sensor_msgs/image_encodings.h>

#include "rviz/image/ros_image_texture.h"

namespace rviz
{

ROSImageTexture::ROSImageTexture()
: new_image_(false)
, width_(0)
, height_(0)
, median_frames_(5)
{
  empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

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

float ROSImageTexture::updateMedian( std::deque<float>& buffer, float value )
{
  //update buffer
  while(buffer.size() > median_frames_-1)
  {
    buffer.pop_back();
  }
  buffer.push_front(value);
  // get median
  std::deque<float> buffer2 = buffer;
  std::nth_element( buffer2.begin(), buffer2.begin()+buffer2.size()/2, buffer2.end() );
  return *( buffer2.begin()+buffer2.size()/2 );
}

void ROSImageTexture::setNormalizeFloatImage( bool normalize, double min, double max )
{
  normalize_ = normalize;
  min_ = min;
  max_ = max;
}

bool ROSImageTexture::update()
{
  sensor_msgs::Image::ConstPtr image;
  bool new_image = false;
  {
    boost::mutex::scoped_lock lock(mutex_);

    image = current_image_;
    new_image = new_image_;
  }

  if (!image || !new_image)
  {
    return false;
  }

  new_image_ = false;

  if (image->data.empty())
  {
    return false;
  }

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
    format = Ogre::PF_SHORT_L;

    /// REVISED CONVERSION TO 8-BIT INTEGER IMAGE
    /*
    size_t i;

    // Ogre encoding
    format = Ogre::PF_BYTE_L;

    // Prepare output buffer
    imageDataSize /= sizeof(uint16_t);
    buffer.resize(imageDataSize);
    imageDataPtr = &buffer[0];

    // Pointer to input image
    uint8_t* input_ptr = (uint8_t*)&image->data[1]; // pointer to high byte of first 16-bit word
    // Pointer to output buffer
    uint8_t* output_ptr = &buffer[0];

    // Downsample to 8-bit - just copy the high bytes of 16-bit words
    for (i = 0; i < imageDataSize; ++i, ++output_ptr, input_ptr+=sizeof(uint16_t))
    {
      *output_ptr = *input_ptr;
    }
    */
  }
  else if (image->encoding.find("bayer") == 0)
  {
    format = Ogre::PF_BYTE_L;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    size_t i;
    float* input_ptr;

    // Ogre encoding
    format = Ogre::PF_BYTE_L;

    // Prepare output buffer
    imageDataSize /= sizeof(float);
    buffer.resize(imageDataSize);
    imageDataPtr = &buffer[0];

    // Pointer to input floating point image
    input_ptr = (float*)&image->data[0];;

    float minValue;
    float maxValue;

    if ( normalize_ )
    {
      // Find min. and max. pixel value
      minValue = std::numeric_limits<float>::max();
      maxValue = std::numeric_limits<float>::min();
      for( i = 0; i < imageDataSize; ++i )
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
    float dynamic_range = maxValue - minValue;
    if( dynamic_range > 0.0f )
    {
      // Pointer to input floating point image
      input_ptr = (float*) &image->data[0];

      // Pointer to output buffer
      uint8_t* output_ptr = &buffer[0];

      // Rescale and quantize
      for( i = 0; i < imageDataSize; ++i, ++output_ptr, ++input_ptr )
      {
        float val = ((*input_ptr - minValue) / dynamic_range);
        if ( val < 0 ) val = 0;
        if ( val > 1 ) val = 1;
        *output_ptr = val * 255u;
      }

    } else
    {
      // clear output buffer
      memset(imageDataPtr, imageDataSize, sizeof(uint8_t));
    }
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
