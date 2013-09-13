/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <boost/algorithm/string/erase.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>

#include <image_transport/subscriber_plugin.h>

#include "rviz/validate_floats.h"

#include "rviz/image/image_display_base.h"

namespace rviz
{

ImageDisplayBase::ImageDisplayBase() :
    Display()
    , it_(update_nh_)
    , sub_()
    , tf_filter_()
    , messages_received_(0)
{
  topic_property_ = new RosTopicProperty("Image Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                         "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateTopic() ));

  transport_property_ = new EnumProperty("Transport Hint", "raw", "Preferred method of sending images.", this,
                                         SLOT( updateTopic() ));

  connect(transport_property_, SIGNAL( requestOptions( EnumProperty* )), this,
          SLOT( fillTransportOptionList( EnumProperty* )));

  queue_size_property_ = new IntProperty( "Queue Size", 2,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 1 );

  transport_property_->setStdString("raw");

  scanForTransportSubscriberPlugins();

}

ImageDisplayBase::~ImageDisplayBase()
{
  unsubscribe();
}

void ImageDisplayBase::incomingMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  if (!msg || context_->getFrameManager()->getPause() )
  {
    return;
  }

  ++messages_received_;
  setStatus(StatusProperty::Ok, "Image", QString::number(messages_received_) + " images received");

  emitTimeSignal( msg->header.stamp );

  processMessage(msg);
}


void ImageDisplayBase::reset()
{
  Display::reset();
  if (tf_filter_)
    tf_filter_->clear();
  messages_received_ = 0;
}

void ImageDisplayBase::updateQueueSize()
{
  uint32_t size = queue_size_property_->getInt();
  if (tf_filter_)
    tf_filter_->setQueueSize(size);
}

void ImageDisplayBase::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  try
  {

    tf_filter_.reset();

    sub_.reset(new image_transport::SubscriberFilter());

    if (!topic_property_->getTopicStd().empty() && !transport_property_->getStdString().empty() )
    {
      sub_->subscribe(it_, topic_property_->getTopicStd(), (uint32_t)queue_size_property_->getInt(),
                      image_transport::TransportHints(transport_property_->getStdString()));

      if (targetFrame_.empty())
      {
        sub_->registerCallback(boost::bind(&ImageDisplayBase::incomingMessage, this, _1));
      }
      else
      {
        tf_filter_.reset( new tf::MessageFilter<sensor_msgs::Image>(*sub_, (tf::Transformer&)*(context_->getTFClient()), targetFrame_, (uint32_t)queue_size_property_->getInt(), update_nh_));
        tf_filter_->registerCallback(boost::bind(&ImageDisplayBase::incomingMessage, this, _1));
      }
    }
    setStatus(StatusProperty::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
  catch (image_transport::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }

  messages_received_ = 0;
  setStatus(StatusProperty::Warn, "Image", "No Image received");
}

void ImageDisplayBase::unsubscribe()
{
  tf_filter_.reset();
  sub_.reset(new image_transport::SubscriberFilter());
}

void ImageDisplayBase::fixedFrameChanged()
{
  if (tf_filter_)
  {
    tf_filter_->setTargetFrame(fixed_frame_.toStdString());
    reset();
  }
}

void ImageDisplayBase::scanForTransportSubscriberPlugins()
{
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader("image_transport",
                                                                       "image_transport::SubscriberPlugin");

  BOOST_FOREACH( const std::string& lookup_name, sub_loader.getDeclaredClasses() )
  {
    // lookup_name is formatted as "pkg/transport_sub", for instance
    // "image_transport/compressed_sub" for the "compressed"
    // transport.  This code removes the "_sub" from the tail and
    // everything up to and including the "/" from the head, leaving
    // "compressed" (for example) in transport_name.
    std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
    transport_name = transport_name.substr(lookup_name.find('/') + 1);

    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try
    {
      boost::shared_ptr<image_transport::SubscriberPlugin> sub = sub_loader.createInstance(lookup_name);
      transport_plugin_types_.insert(transport_name);
    }
    catch (const pluginlib::LibraryLoadException& e)
    {
    }
    catch (const pluginlib::CreateClassException& e)
    {
    }
  }
}

void ImageDisplayBase::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void ImageDisplayBase::fillTransportOptionList(EnumProperty* property)
{
  property->clearOptions();

  std::vector<std::string> choices;

  choices.push_back("raw");

  // Loop over all current ROS topic names
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    // If the beginning of this topic name is the same as topic_,
    // and the whole string is not the same,
    // and the next character is /
    // and there are no further slashes from there to the end,
    // then consider this a possible transport topic.
    const ros::master::TopicInfo& ti = *it;
    const std::string& topic_name = ti.name;
    const std::string& topic = topic_property_->getStdString();

    if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/'
        && topic_name.find('/', topic.size() + 1) == std::string::npos)
    {
      std::string transport_type = topic_name.substr(topic.size() + 1);

      // If the transport type string found above is in the set of
      // supported transport type plugins, add it to the list.
      if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end())
      {
        choices.push_back(transport_type);
      }
    }
  }

  for (size_t i = 0; i < choices.size(); i++)
  {
    property->addOptionStd(choices[i]);
  }
}

} // end namespace rviz
