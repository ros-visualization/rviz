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


#ifndef RVIZ_ROBOT_BASE2D_POSE_DISPLAY_H_
#define RVIZ_ROBOT_BASE2D_POSE_DISPLAY_H_

#include "display.h"
#include "helpers/color.h"
#include "properties/forwards.h"

#include <deprecated_msgs/RobotBase2DOdom.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

namespace ogre_tools
{
class Arrow;
}

namespace Ogre
{
class SceneNode;
}

namespace tf
{
template<class Message> class MessageNotifier;
}

namespace rviz
{

/**
 * \class RobotBase2DPoseDisplay
 * \brief Accumulates and displays the pose from a deprecated_msgs::RobotBase2DOdom message
 */
class RobotBase2DPoseDisplay : public Display
{
public:
  RobotBase2DPoseDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~RobotBase2DPoseDisplay();

  void setTopic( const std::string& topic );
  const std::string& getTopic() { return topic_; }

  void setColor( const Color& color );
  const Color& getColor() { return color_; }

  void setPositionTolerance( float tol );
  float getPositionTolerance() { return position_tolerance_; }

  void setAngleTolerance( float tol );
  float getAngleTolerance() { return angle_tolerance_; }

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  static const char* getTypeStatic() { return "RobotBase2DPose"; }
  virtual const char* getType() const { return getTypeStatic(); }
  static const char* getDescription();

protected:
  void subscribe();
  void unsubscribe();
  void clear();

  typedef boost::shared_ptr<deprecated_msgs::RobotBase2DOdom> MessagePtr;

  void incomingMessage( const MessagePtr& message );
  void processMessage( const MessagePtr& message );
  void transformArrow( const MessagePtr& message, ogre_tools::Arrow* arrow );

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  Color color_;

  typedef std::vector<ogre_tools::Arrow*> V_Arrow;
  V_Arrow arrows_;

  Ogre::SceneNode* scene_node_;

  typedef std::vector<MessagePtr> V_RobotBase2DOdom;
  V_RobotBase2DOdom message_queue_;
  boost::mutex queue_mutex_;
  MessagePtr last_used_message_;


  float position_tolerance_;
  float angle_tolerance_;

  ColorPropertyWPtr color_property_;
  ROSTopicStringPropertyWPtr topic_property_;
  FloatPropertyWPtr position_tolerance_property_;
  FloatPropertyWPtr angle_tolerance_property_;

  tf::MessageNotifier<deprecated_msgs::RobotBase2DOdom>* notifier_;
};

} // namespace rviz

#endif /* RVIZ_ROBOT_BASE2D_POSE_DISPLAY_H_ */
