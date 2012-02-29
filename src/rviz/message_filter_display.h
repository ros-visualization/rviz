/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef MESSAGE_FILTER_DISPLAY_H
#define MESSAGE_FILTER_DISPLAY_H

#include <sstream>
#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include "rviz/display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"

namespace rviz
{

/** @brief Templated superclass of displays which use a
 *         tf::MessageFilter for their incoming messages.
 *
 * MessageFilterDisplay is templated on the message class.  The intent
 * is that Display subclasses which need a tf::MessageFilter for their
 * incoming data will subclass from this type, templated on the
 * message class they need. */
template<class MessageClass>
class MessageFilterDisplay: public Display
{
public:
  typedef boost::shared_ptr<MessageClass const> MessageConstPtr;

  MessageFilterDisplay( int initial_queue_size, bool use_threaded_nh = false )
    : tf_filter_( NULL )
    , use_threaded_nh_( use_threaded_nh )
    , initial_queue_size_( initial_queue_size )
    {
    }

  virtual ~MessageFilterDisplay()
    {
      delete tf_filter_;
    }

  void onInitialize()
    {
      tf_filter_ = new tf::MessageFilter<MessageClass>( *vis_manager_->getTFClient(), "", initial_queue_size_, getNH() );
      tf_filter_->connectInput( sub_ );
      tf_filter_->registerCallback( &MessageFilterDisplay::processMessage, this );
    }

  /**
   * Set the incoming message topic
   * @param topic The topic we should listen to
   */
  void setTopic( const std::string& topic )
    {
      unsubscribe();
      topic_ = topic;
      reset();
      subscribe();

      propertyChanged( topic_property_ );

      causeRender();
    }

  const std::string& getTopic()
    {
      return topic_;
    }

  /** Set the incoming message queue size. */
  void setQueueSize( int size )
    {
      if( size != (int) tf_filter_->getQueueSize() )
      {
        tf_filter_->setQueueSize( (uint32_t) size );
        propertyChanged( queue_size_property_ );
      }
    }

  int getQueueSize()
    {
      return (int) tf_filter_->getQueueSize();
    }

  void fixedFrameChanged()
    {
      tf_filter_->setTargetFrame( fixed_frame_ );
    }

protected:
  virtual void onEnable()
    {
      subscribe();
    }
  virtual void onDisable()
    {
      unsubscribe();
      tf_filter_->clear();
    }

  /**
   * \brief Subscribes to the topic set by setTopic()
   */
  virtual void subscribe()
    {
      if ( !isEnabled() )
      {
        return;
      }

      try
      {
        sub_.subscribe( getNH(), topic_, initial_queue_size_ );
        setStatus( status_levels::Ok, "Topic", "OK" );
      }
      catch (ros::Exception& e)
      {
        setStatus( status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what() );
      }
    }

  /**
   * \brief Unsubscribes from the current topic
   */
  virtual void unsubscribe()
    {
      sub_.unsubscribe();
    }

  virtual void createProperties()
    {
      topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_,
                                                                                   boost::bind( &MessageFilterDisplay::getTopic, this ),
                                                                                   boost::bind( &MessageFilterDisplay::setTopic, this, _1 ),
                                                                                   parent_category_, this );
      std::stringstream help;
      help << ros::message_traits::datatype<MessageClass>() << " topic to subscribe to.";
      setPropertyHelpText( topic_property_, help.str().c_str() );
      ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
      topic_prop->setMessageType(ros::message_traits::datatype<MessageClass>());

      queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                             boost::bind( &MessageFilterDisplay::getQueueSize, this ),
                                                                             boost::bind( &MessageFilterDisplay::setQueueSize, this, _1 ),
                                                                             parent_category_, this );
      std::stringstream help2;
      help2 << "Advanced: set the size of the incoming "
            << ros::message_traits::datatype<MessageClass>()
            << " message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your "
            << ros::message_traits::datatype<MessageClass>()
            << " data, but it can greatly increase memory usage if the messages are big.";
      setPropertyHelpText( queue_size_property_, help2.str().c_str() );
    }

  /**
   * @brief Implement this to handle an incoming message which has
   *        been vetted by the tf::MessageFilter.
   */
  virtual void processMessage( const MessageConstPtr& message ) = 0;

  /**
   * @brief return threaded_nh_ or update_nh_, according to the value of use_threaded_nh_.
   */
  ros::NodeHandle& getNH()
    {
      if( use_threaded_nh_ )
      {
        return threaded_nh_;
      }
      else
      {
        return update_nh_;
      }
    }

  std::string topic_;  ///< The message topic set by setTopic()

  message_filters::Subscriber<MessageClass> sub_;
  tf::MessageFilter<MessageClass>* tf_filter_;

  ROSTopicStringPropertyWPtr topic_property_;
  IntPropertyWPtr queue_size_property_;

  bool use_threaded_nh_;
  int initial_queue_size_;
};

} // end namespace rviz

#endif // MESSAGE_FILTER_DISPLAY_H
