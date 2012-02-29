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
#include "marker_array_display.h"

#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"

namespace rviz
{

MarkerArrayDisplay::MarkerArrayDisplay()
  : MarkerDisplay()
  , topic_("visualization_marker_array")
{
}

MarkerArrayDisplay::~MarkerArrayDisplay()
{
}

void MarkerArrayDisplay::setTopic(const std::string& topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();
  propertyChanged(topic_property_);
}

void MarkerArrayDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Marker Array Topic", property_prefix_,
                                                                               boost::bind( &MarkerArrayDisplay::getTopic, this ),
                                                                               boost::bind( &MarkerArrayDisplay::setTopic, this, _1 ),
                                                                               parent_category_, this );
  setPropertyHelpText( topic_property_,
                       "visualization_msgs::MarkerArray topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<visualization_msgs::MarkerArray>());

  queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                         boost::bind( &MarkerArrayDisplay::getQueueSize, this ),
                                                                         boost::bind( &MarkerArrayDisplay::setQueueSize, this, _1 ),
                                                                         parent_category_, this );
  setPropertyHelpText( queue_size_property_, "Advanced: set the size of the incoming Marker message queue.  This should generally be at least a few times larger than the number of Markers in each MarkerArray." );

  namespaces_category_ = property_manager_->createCategory("Namespaces", property_prefix_, parent_category_, this);
}

/**
 * \brief Subscribes to the marker array topic
 */
void MarkerArrayDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if (!topic_.empty())
  {
    array_sub_.shutdown();

    try
    {
      array_sub_ = update_nh_.subscribe(topic_, 1000, &MarkerArrayDisplay::handleMarkerArray, this);
      setStatus(status_levels::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
    }
  }
}

void MarkerArrayDisplay::unsubscribe()
{
  array_sub_.shutdown();
}

// I seem to need this wrapper function to make the compiler like my
// function pointer in the .subscribe() call above.
void MarkerArrayDisplay::handleMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& array)
{
  incomingMarkerArray( array );
}

} // end namespace rviz
