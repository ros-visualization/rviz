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
 *
 * $Id$
 *
 */

#ifndef RVIZ_COLLISION_MAP_DISPLAY_H_
#define RVIZ_COLLISION_MAP_DISPLAY_H_

#include "display.h"
#include "helpers/color.h"
#include "properties/forwards.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>

#include <robot_msgs/OrientedBoundingBox.h>
#include <robot_msgs/CollisionMap.h>

namespace ogre_tools
{
class PointCloud;
}

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace tf
{
template<class Message> class MessageNotifier;
}

namespace rviz
{

namespace collision_render_ops
{
enum CollisionRenderOp
{
  CBoxes, CPoints, CCount,
};
}
typedef collision_render_ops::CollisionRenderOp CollisionRenderOp;

/**
 * \class CollisionMapDisplay
 * \brief Displays a collision_map::CollisionMap message
 */
class CollisionMapDisplay : public Display
{
public:
  CollisionMapDisplay(const std::string& name, VisualizationManager* manager);

  virtual ~CollisionMapDisplay();

  void setTopic(const std::string& topic);
  const std::string& getTopic()
  {
    return (topic_);
  }

  void setColor(const Color& color);
  const Color& getColor()
  {
    return (color_);
  }

  void setOverrideColor(bool override);
  bool getOverrideColor()
  {
    return (override_color_);
  }

  void setRenderOperation(int op);
  int getRenderOperation()
  {
    return (render_operation_);
  }

  void setPointSize(float size);
  float getPointSize()
  {
    return (point_size_);
  }

  void setZPosition(float z);
  float getZPosition()
  {
    return (z_position_);
  }

  void setAlpha(float alpha);
  float getAlpha()
  {
    return (alpha_);
  }

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  static const char *getTypeStatic()
  {
    return "Collision Map";
  }
  virtual const char *getType() const
  {
    return getTypeStatic();
  }
  static const char *getDescription();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  typedef boost::shared_ptr<robot_msgs::CollisionMap> CollisionMapPtr;
  void incomingMessage(const CollisionMapPtr& message);
  void processMessage();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  Color color_;
  int render_operation_;
  bool override_color_;
  float point_size_;
  float z_position_;
  float alpha_;

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  ogre_tools::PointCloud* cloud_;

  boost::mutex message_mutex_;
  CollisionMapPtr new_message_;
  CollisionMapPtr current_message_;
  tf::MessageNotifier<robot_msgs::CollisionMap>* notifier_;

  ColorPropertyWPtr color_property_;
  ROSTopicStringPropertyWPtr topic_property_;
  BoolPropertyWPtr override_color_property_;
  EnumPropertyWPtr render_operation_property_;
  FloatPropertyWPtr point_size_property_;
  FloatPropertyWPtr z_position_property_;
  FloatPropertyWPtr alpha_property_;
};

} // namespace rviz

#endif /* RVIZ_COLLISION_MAP_DISPLAY_H_ */
