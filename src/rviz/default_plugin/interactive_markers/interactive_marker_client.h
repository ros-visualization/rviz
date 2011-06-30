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
#ifndef INTERACTIVE_MARKER_CLIENT_H
#define INTERACTIVE_MARKER_CLIENT_H

#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

namespace rviz
{

/**
 * Interface for objects which need to receive interactive markers
 * from the server.  InteractiveMarkerClient needs one of these to
 * send its processed data into.
 */
class InteractiveMarkerReceiver
{
public:
  /**
   * Implement this to add or update all markers in "markers", update
   * the poses of all markers mentioned in "poses", and to erase all
   * markers named in "erases".
   */
  virtual void processMarkerChanges( const std::vector<visualization_msgs::InteractiveMarker>* markers = NULL,
                                     const std::vector<visualization_msgs::InteractiveMarkerPose>* poses = NULL,
                                     const std::vector<std::string>* erases = NULL ) = 0;
  /**
   * Implement this to clear all markers.
   */
  virtual void clearMarkers() = 0;

  /**
   * Implement this to subscribe to just the init messages.
   * @return true on success, false on failure.
   */
  virtual bool subscribeToInit() = 0;

  /**
   * Implement this to unsubscribe from the init messages.  Should
   * never fail.
   */
  virtual void unsubscribeFromInit() = 0;

  /**
   * Implement this to receive named status messages indicating an
   * "OK" state.
   */
  virtual void setStatusOk(const std::string& name, const std::string& text) = 0;

  /**
   * Implement this to receive named status messages indicating a
   * "warning" state.
   */
  virtual void setStatusWarn(const std::string& name, const std::string& text) = 0;

  /**
   * Implement this to receive named status messages indicating an
   * "error" state.
   */
  virtual void setStatusError(const std::string& name, const std::string& text) = 0;
};

/**
 * Client for interactive markers sent from one or more interactive
 * marker servers.  Implements the logic of init and update
 * sequencing.  To actually display or use the received markers,
 * implement a subclass of InteractiveMarkerReceiver and pass that to
 * the constructor.
 *
 * TODO: think about integrating topic subscription into this class,
 * it might simplify some of the between-class logic. (hersh)
 */
class InteractiveMarkerClient
{
public:
  /**
   * Constructor.
   * @param receiver pointer to the consumer of interactive marker data.  Must not be NULL.
   */
  InteractiveMarkerClient( InteractiveMarkerReceiver* receiver );

  /**
   * ROS callback notifying us of a new marker.
   * Send this to NodeHandle::subscribe() for the update topic.
   */
  void processMarkerUpdate(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& marker_update);

  /**
   * ROS callback notifying us of an init message (full update with
   * all data).  Send this to NodeHandle::subscribe() for the init topic.
   */
  void processMarkerInit(const visualization_msgs::InteractiveMarkerInit::ConstPtr& marker_init);

  /**
   * Reinitialize.  Call this when topics have been unsubscribed and
   * are now being subscribed again.  Calls
   * InteractiveMarkerReceiver::subscribeToInit().
   */
  void reinit();

  /**
   * If the user of the InteractiveMarkerClient unsubscribes from the
   * init topic on its own, the client needs to be told.  Call this
   * function to do so.
   */
  void unsubscribedFromInit();

  /**
   * Return true if we haven't seen any publishers since the last reinit().
   */
  bool isPublisherListEmpty();

  /**
   * Clear the list of publisher contexts and call reinit().
   */
  void clear();

  /**
   * Compare the current time with the last_update_time of each
   * publisher context and call setStatusWarn() for ones which are
   * late.
   */
  void flagLateConnections();

private:

  InteractiveMarkerReceiver* receiver_;

  typedef std::map<uint64_t, visualization_msgs::InteractiveMarkerUpdate::ConstPtr> M_InteractiveMarkerUpdate;

  struct PublisherContext {
    bool update_seen;
    bool init_seen;
    uint64_t last_update_seq_num;
    uint64_t last_init_seq_num;
    ros::Time last_update_time;
    bool update_time_ok;
    bool initialized;

    // We queue up UPDATE messages which arrive before we get an init
    // message, in case the update messages are ahead of the init
    // messages.  No KEEP_ALIVE messages are stored here.
    M_InteractiveMarkerUpdate update_queue;

    PublisherContext();
    void enqueueUpdate(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& update);

    // Update the value of initialized based on previous sequence
    // numbers and the number from this new update.
    bool checkInitWith(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& update);

    // Update the value of initialized based on previous sequence
    // numbers and the number from this init message.
    bool checkInitWith(const visualization_msgs::InteractiveMarkerInit::ConstPtr& init);
  };
  typedef boost::shared_ptr<PublisherContext> PublisherContextPtr;

  typedef std::map<std::string, PublisherContextPtr> M_PublisherContext;
  M_PublisherContext publisher_contexts_;

  // If the markers have been cleared since the last call to
  // InteractiveMarkerReceiver::processMarkerChanges().
  bool cleared_;

  // If we have called InteractiveMarkerReceiver::subscribeToInit()
  // since we've called
  // InteractiveMarkerReceiver::unsubscribeFromInit().
  bool subscribed_to_init_;

  // play back the relevant updates from the queue, if any, and clear the queue.
  void playbackUpdateQueue( PublisherContextPtr& context );

  // Apply a single update message from a given publisher context.
  void applyUpdate( const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& marker_update,
                    PublisherContextPtr& context );

  // Check if we are ready to unsubscribe from the init topic, and
  // unsubscribe if so.
  void maybeUnsubscribeFromInit();
};

} // namespace rviz

#endif // INTERACTIVE_MARKER_CLIENT_H
