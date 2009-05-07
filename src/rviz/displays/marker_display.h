/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_MARKER_DISPLAY_H
#define RVIZ_MARKER_DISPLAY_H

#include "display.h"
#include "selection/forwards.h"

#include <map>

#include <visualization_msgs/Marker.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace ros
{
class Node;
}

namespace ogre_tools
{
class Object;
}

namespace mechanism
{
class Robot;
}

namespace planning_models
{
class KinematicModel;
}

namespace tf
{
template<class Message> class MessageNotifier;
}

namespace rviz
{

class MarkerSelectionHandler;
typedef boost::shared_ptr<MarkerSelectionHandler> MarkerSelectionHandlerPtr;

typedef std::pair<std::string, int32_t> MarkerID;

/**
 * \class MarkerDisplay
 * \brief Displays "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Markers come in as visualization_msgs::Marker messages.  See the Marker message for more information.
 */
class MarkerDisplay : public Display
{
public:
  MarkerDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~MarkerDisplay();

  virtual void update(float wall_dt, float ros_dt);

  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void reset();

  static const char* getTypeStatic() { return "Markers"; }
  virtual const char* getType() const { return getTypeStatic(); }
  static const char* getDescription();

protected:
  virtual void onEnable();
  virtual void onDisable();

  /**
   * \brief Subscribes to the "visualization_marker" topic
   */
  void subscribe();
  /**
   * \brief Unsubscribes from the "visualization_marker" topic
   */
  void unsubscribe();

  /**
   * \brief Removes all the markers
   */
  void clearMarkers();

  typedef boost::shared_ptr<visualization_msgs::Marker> MarkerPtr;

  /**
   * \brief Processes a marker message
   * @param message The message to process
   */
  void processMessage( const MarkerPtr& message );
  /**
   * \brief Processes an "Add" marker message
   * @param message The message to process
   */
  void processAdd( const MarkerPtr& message );
  /**
   * \brief Processes a "Delete" marker message
   * @param message The message to process
   */
  void processDelete( const MarkerPtr& message );
  /**
   * \brief Set any necessary values (ie. position, orientation, scale, color, etc.) on a marker's object
   */
  void setValues( const MarkerPtr& message, ogre_tools::Object* object );

  struct MarkerInfo;
  void destroyMarker(MarkerInfo& marker);
  MarkerInfo* getMarker(MarkerID id);

  /**
   * \brief ROS callback notifying us of a new marker
   */
  void incomingMarker(const MarkerPtr& marker);

  struct MarkerInfo
  {
    MarkerInfo( ogre_tools::Object* object, const MarkerPtr& message )
    : object_(object)
    , coll_(0)
    , message_(message)
    , time_elapsed_(0.0f)
    {}

    ogre_tools::Object* object_;
    CollObjectHandle coll_;
    boost::shared_ptr<visualization_msgs::Marker> message_;

    float time_elapsed_;
  };

  typedef std::map<MarkerID, MarkerInfo> M_IDToMarker;
  M_IDToMarker markers_;                                ///< Map of marker id to the marker info structure

  typedef std::vector<MarkerPtr> V_MarkerMessage;
  V_MarkerMessage message_queue_;                       ///< Marker message queue.  Messages are added to this as they are received, and then processed
                                                        ///< in our update() function
  boost::mutex queue_mutex_;

  Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to

  mechanism::Robot* descr_;
  planning_models::KinematicModel* kinematic_model_;

  tf::MessageNotifier<visualization_msgs::Marker>* notifier_;

  friend class MarkerSelectionHandler;
};

} // namespace rviz

#endif /* RVIZ_MARKER_DISPLAY_H */
