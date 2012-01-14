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

#include "interactive_marker_client.h"
#include <ros/console.h>

namespace rviz
{

InteractiveMarkerClient::PublisherContext::PublisherContext()
{
  update_seen = false;
  init_seen = false;
  last_update_seq_num = 0;
  last_init_seq_num = 0;
  update_time_ok = true;
  initialized = false;
  last_update_time = ros::Time::now();
}

// Update the value of initialized based on previous sequence
// numbers and the number from this new update.
bool InteractiveMarkerClient::PublisherContext::checkInitWith(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& update)
{
  if( (init_seen &&
       update->type == visualization_msgs::InteractiveMarkerUpdate::UPDATE &&
       last_init_seq_num + 1 >= update->seq_num)
      ||
      (init_seen &&
       update->type == visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE &&
       last_init_seq_num >= update->seq_num) )
  {
    initialized = true;
  }
  return initialized;
}

// Update the value of initialized based on previous sequence
// numbers and the number from this init message.
bool InteractiveMarkerClient::PublisherContext::checkInitWith(const visualization_msgs::InteractiveMarkerInit::ConstPtr& init)
{
  M_InteractiveMarkerUpdate::const_iterator q_it_same_seq = update_queue.find( init->seq_num );
  M_InteractiveMarkerUpdate::const_iterator q_it_seq_plus_one = update_queue.find( init->seq_num + 1 );

  if( (update_seen &&
       init->seq_num + 1 >= last_update_seq_num + 1) ||
      (q_it_seq_plus_one != update_queue.end() && q_it_seq_plus_one->second->type == visualization_msgs::InteractiveMarkerUpdate::UPDATE) ||
      (q_it_same_seq != update_queue.end() && q_it_same_seq->second->type == visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE) )
  {
    initialized = true;
  }
  return initialized;
}

void InteractiveMarkerClient::PublisherContext::enqueueUpdate(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& update)
{
  if( update->type == visualization_msgs::InteractiveMarkerUpdate::UPDATE )
  {
    update_queue[ update->seq_num ] = update;
  }
  else // else this must be a KEEP_ALIVE type.
  {
    // Keep-alive messages get queued only if there is not an update
    // queued at that sequence number already.
    if( update_queue.find( update->seq_num ) == update_queue.end() )
    {
      update_queue[ update->seq_num ] = update;
    }
  }
  update_seen = true;
  last_update_seq_num = update->seq_num;
}


InteractiveMarkerClient::InteractiveMarkerClient( InteractiveMarkerReceiver* receiver )
  : cleared_( true )
  , subscribed_to_init_( false )
{
  receiver_ = receiver;
}

// ROS callback notifying us of an init message (full update with all data)
void InteractiveMarkerClient::processMarkerInit(const visualization_msgs::InteractiveMarkerInit::ConstPtr& marker_init)
{
  ROS_DEBUG("InteractiveMarkerClient: %s INIT %lu",
            marker_init->server_id.c_str(),
            marker_init->seq_num);

  // get caller ID of the sending entity
  if ( marker_init->server_id.empty() )
  {
    receiver_->setStatusError( "Topic", "server_id is empty!");
  }

  M_PublisherContext::iterator context_it = publisher_contexts_.find(marker_init->server_id);

  // if this is the first message from that server, create new context
  if ( context_it == publisher_contexts_.end() )
  {
    PublisherContextPtr pc(new PublisherContext());
    context_it = publisher_contexts_.insert( std::make_pair(marker_init->server_id,pc) ).first;
  }

  PublisherContextPtr context = context_it->second;

  // If this context is already initialized, we can safely ignore this
  // init message, because the updates will give us any changes
  // present here.
  if( context->initialized )
  {
    return;
  }

  if( context->checkInitWith( marker_init ))
  {
    receiver_->processMarkerChanges( &marker_init->markers );
    cleared_ = false;

    context->last_init_seq_num = marker_init->seq_num;
    context->init_seen = true;
    context->last_update_time = ros::Time::now();

    // This update completed initialization for this context.
    receiver_->setStatusOk( context_it->first, "Initialization complete.");

    // The next update message we want is the one just after this
    // init message.  Now that this context is initialized, init
    // messages will be ignored and updates will get checked against
    // last_update_seq_num.
    context->last_update_seq_num = context->last_init_seq_num;

    // See if we are completely done initializing and can unsubscribe from init.
    maybeUnsubscribeFromInit();
    
    playbackUpdateQueue( context );
  }
  else if( context->update_queue.empty() )
  {
    // If we've already seen an init for this publisher but we are not
    // initialized, we must erase all markers and start again to avoid
    // processing two init messages without clearMarkers() in between.
    // 
    // TODO: If we had the ability to clear the markers from a single
    // server at a time, we could just clear the one server's worth
    // here and not have to re-init all the servers.
    if( context->init_seen )
    {
      reinit();
    }

    receiver_->processMarkerChanges( &marker_init->markers );
    cleared_ = false;

    context->last_init_seq_num = marker_init->seq_num;
    context->init_seen = true;
    context->last_update_time = ros::Time::now();
  }
  // else we must have some queued updates but they don't match up with this init, so do nothing.
}

// play back the relevant updates from the queue, if any, and clear the queue.
void InteractiveMarkerClient::playbackUpdateQueue( PublisherContextPtr& context )
{
  uint64_t next_seq_needed = context->last_init_seq_num + 1;

  M_InteractiveMarkerUpdate::iterator update_it = context->update_queue.begin();
  for( ; update_it != context->update_queue.end(); update_it++ )
  {
    visualization_msgs::InteractiveMarkerUpdate::ConstPtr update = update_it->second;

    if( update->seq_num == next_seq_needed )
    {
      applyUpdate( update, context );
      next_seq_needed++;
    }
    else if( update->seq_num < next_seq_needed )
    {
      ROS_DEBUG("Ignoring unneeded queued update number %lu, looking for %lu.", update->seq_num, next_seq_needed);
    }
    else
    {
      ROS_ERROR("Found queued update number %lu, missed %lu.", update->seq_num, next_seq_needed);
    }
  }
  context->update_queue.clear();
}

void InteractiveMarkerClient::processMarkerUpdate(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& marker_update)
{
//  ROS_DEBUG("InteractiveMarkerClient: %s %s %lu",
//            marker_update->server_id.c_str(),
//            (marker_update->type == visualization_msgs::InteractiveMarkerUpdate::UPDATE ? "UPDATE" : "KEEP_ALIVE"),
//            marker_update->seq_num);

  // get caller ID of the sending entity
  if ( marker_update->server_id.empty() )
  {
    receiver_->setStatusError( "Topic", "server_id is empty!");
  }

  M_PublisherContext::iterator context_it = publisher_contexts_.find(marker_update->server_id);

  // If we haven't seen this publisher before, we need to reset the
  // display and listen to the init topic, plus of course add this
  // publisher to our list.
  if ( context_it == publisher_contexts_.end() )
  {
    PublisherContextPtr pc(new PublisherContext());
    // Remember this update message, because we may be able to use it.
    pc->enqueueUpdate(marker_update);

    context_it = publisher_contexts_.insert( std::make_pair(marker_update->server_id,pc) ).first;

    reinit();

    // We have enqueued the update message, and we can't do anything
    // else until we get an Init message from this publisher, so return.
    return;
  }

  PublisherContextPtr context = context_it->second;

  if ( !context->initialized )
  {
    if( context->checkInitWith( marker_update ))
    {
      // This update completed initialization for this context.
      receiver_->setStatusOk( context_it->first, "Initialization complete.");

      // The next update message we want is the one just after the
      // init message.  Now that this context is initialized, init
      // messages will be ignored and updates will get checked against
      // last_update_seq_num.
      context->last_update_seq_num = context->last_init_seq_num;

      // See if we are completely done initializing and can unsubscribe from init.
      maybeUnsubscribeFromInit();
    }
    else
    {
      // We have received an update before the actual init message, so
      // just queue it up and return.
      receiver_->setStatusWarn( marker_update->server_id, "Received update or keep-alive without previous INIT message. It might be lost.");
      context->enqueueUpdate(marker_update);
      return;
    }
  }

  // From here down, we know this context is initialized and we are
  // just doing a normal update or keepalive.
  applyUpdate( marker_update, context );
}

void InteractiveMarkerClient::clear()
{
  publisher_contexts_.clear();
  reinit();
}

void InteractiveMarkerClient::unsubscribedFromInit()
{
  subscribed_to_init_ = false;
}

void InteractiveMarkerClient::reinit()
{
  if( !cleared_ )
  {
    receiver_->clearMarkers();
    cleared_ = true;
  }
  if( !subscribed_to_init_ )
  {
    subscribed_to_init_ = receiver_->subscribeToInit();
  }

  // Set all contexts to uninitialized since we are throwing away all the marker data.
  M_PublisherContext::iterator context_it;
  for( context_it = publisher_contexts_.begin(); context_it != publisher_contexts_.end(); context_it++ )
  {
    context_it->second->initialized = false;
  }
}

// If all publisher contexts are initialized, unsubscribe from the "init" topic.
void InteractiveMarkerClient::maybeUnsubscribeFromInit() {
  M_PublisherContext::iterator context_it;
  for( context_it = publisher_contexts_.begin(); context_it != publisher_contexts_.end(); context_it++ )
  {
    if( !context_it->second->initialized )
    {
      return;
    }
  }
  receiver_->unsubscribeFromInit();
  subscribed_to_init_ = false;
}

void InteractiveMarkerClient::applyUpdate( const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& marker_update,
                                           PublisherContextPtr& context )
{
  uint64_t expected_seq_num = 0;

  switch ( marker_update->type )
  {
    case visualization_msgs::InteractiveMarkerUpdate::UPDATE:
      expected_seq_num = context->last_update_seq_num + 1;
      break;

    case visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE:
      expected_seq_num = context->last_update_seq_num;
      break;
  }

  if ( marker_update->seq_num != expected_seq_num )
  {
    if( marker_update->seq_num < expected_seq_num )
    {
      ROS_INFO("Received sequence number %lu, less than expected sequence number %lu. Ignoring.",
               marker_update->seq_num, expected_seq_num);
      return;
    }

    // we've lost some updates
    std::ostringstream s;
    s << "Detected lost update or server restart. Resetting. Reason: Received wrong sequence number (expected: " <<
        expected_seq_num << ", received: " << marker_update->seq_num << ")";
    receiver_->setStatusError( marker_update->server_id, s.str());
    reinit();
    return;
  }

  context->last_update_seq_num = marker_update->seq_num;
  context->update_seen = true;
  context->last_update_time = ros::Time::now();

  if( marker_update->type == visualization_msgs::InteractiveMarkerUpdate::UPDATE )
  {
    receiver_->processMarkerChanges( &marker_update->markers, &marker_update->poses, &marker_update->erases );
    cleared_ = false;
  }
}

bool InteractiveMarkerClient::isPublisherListEmpty()
{
  return publisher_contexts_.empty();
}

void InteractiveMarkerClient::flagLateConnections()
{
  M_PublisherContext::iterator it;
  for ( it = publisher_contexts_.begin(); it != publisher_contexts_.end(); it++ )
  {
    PublisherContextPtr& context = it->second;
    double time_since_last_update = (ros::Time::now() - context->last_update_time).toSec();
    if ( time_since_last_update > 1.0 )
    {
      std::stringstream s;
      s << "No update received for " << (int)time_since_last_update << " seconds. Connection might be lost.";
      receiver_->setStatusWarn( it->first, s.str() );
      context->update_time_ok = false;
    }
    if ( !context->update_time_ok && time_since_last_update <= 1.0 )
    {
      receiver_->setStatusOk( it->first, "OK" );
    }
  }
}

} // namespace rviz
