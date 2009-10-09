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

#include "ros_image_texture.h"

#include <tf/tf.h>

#include <OGRE/OgreTextureManager.h>

namespace rviz
{

ROSImageTexture::ROSImageTexture(const ros::NodeHandle& nh)
: nh_(nh)
, new_image_(false)
, width_(0)
, height_(0)
, tf_client_(0)
, image_count_(0)
{
  empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  static uint32_t count = 0;
  std::stringstream ss;
  ss << "ROSImageTexture" << count++;
  texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, empty_image_);
}

ROSImageTexture::~ROSImageTexture()
{
}

void ROSImageTexture::clear()
{
  boost::mutex::scoped_lock lock(mutex_);

  texture_->unload();
  texture_->loadImage(empty_image_);

  new_image_ = false;
  current_image_.reset();

  if (tf_filter_)
  {
    tf_filter_->clear();
  }

  image_count_ = 0;
}

void ROSImageTexture::setFrame(const std::string& frame, tf::TransformListener* tf_client)
{
  tf_client_ = tf_client;
  frame_ = frame;
  setTopic(topic_);
}

void ROSImageTexture::setTopic(const std::string& topic)
{
  topic_ = topic;
  tf_filter_.reset();
  sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>());

  if (!topic.empty())
  {
    sub_->subscribe(nh_, topic, 1);

    if (frame_.empty())
    {
      sub_->registerCallback(boost::bind(&ROSImageTexture::callback, this, _1));
    }
    else
    {
      ROS_ASSERT(tf_client_);
      tf_filter_.reset(new tf::MessageFilter<sensor_msgs::Image>(*sub_, (tf::Transformer&)*tf_client_, frame_, 2, nh_));
      tf_filter_->registerCallback(boost::bind(&ROSImageTexture::callback, this, _1));
    }
  }
}

const sensor_msgs::Image::ConstPtr& ROSImageTexture::getImage()
{
  boost::mutex::scoped_lock lock(mutex_);

  return current_image_;
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

  Ogre::PixelFormat format = Ogre::PF_R8G8B8;

  if (image->encoding == sensor_msgs::image_encodings::RGB8)
  {
    format = Ogre::PF_R8G8B8;
  }
  else if (image->encoding == sensor_msgs::image_encodings::RGBA8)
  {
    format = Ogre::PF_R8G8B8A8;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
           //image->encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
           image->encoding == sensor_msgs::image_encodings::BGRA8)
  {
    format = Ogre::PF_B8G8R8A8;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
           //image->encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
           image->encoding == sensor_msgs::image_encodings::BGR8)
  {
    format = Ogre::PF_B8G8R8;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
           //image->encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
           image->encoding == sensor_msgs::image_encodings::MONO8)
  {
    format = Ogre::PF_L8;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
           //image->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
           image->encoding == sensor_msgs::image_encodings::MONO16)
  {
    format = Ogre::PF_L16;
  }
  else
  {
    throw UnsupportedImageEncoding(image->encoding);
  }

  width_ = image->width;
  height_ = image->height;

  uint32_t size = image->height * image->step;
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream((void*)(&image->data[0]), size));
  texture_->unload();
  texture_->loadRawData(pixel_stream, width_, height_, format);

  return true;
}

void ROSImageTexture::callback(const sensor_msgs::Image::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  current_image_ = msg;
  new_image_ = true;

  ++image_count_;
}

}
