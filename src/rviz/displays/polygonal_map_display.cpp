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

#include "polygonal_map_display.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include "ogre_tools/arrow.h"

#include <ros/node.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/point_cloud.h>

namespace rviz
{

PolygonalMapDisplay::PolygonalMapDisplay(const std::string & name,
    VisualizationManager * manager)
: Display(name, manager)
, color_(0.1f, 1.0f, 0.0f)
, render_operation_(polygon_render_ops::PLines)
, override_color_(false)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Polygonal Map" << count++;
  manual_object_ = scene_manager_->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  cloud_ = new ogre_tools::PointCloud (scene_manager_, scene_node_);
  billboard_line_ = new ogre_tools::BillboardLine (scene_manager_, scene_node_);

  setAlpha (1.0f);
  setPointSize (0.02f);
  setLineWidth(0.02f);
  setZPosition (0.0f);

  notifier_ = new tf::MessageNotifier<robot_msgs::PolygonalMap> (tf_, ros_node_,
                                                                 boost::bind(&PolygonalMapDisplay::incomingMessage, this, _1),
                                                                 "", "", 1);
}

PolygonalMapDisplay::~PolygonalMapDisplay()
{
  unsubscribe();
  clear();

  delete notifier_;

  scene_manager_->destroyManualObject(manual_object_);

  delete cloud_;
  delete billboard_line_;
}

void PolygonalMapDisplay::clear()
{
  manual_object_->clear ();
  cloud_->clear ();
  billboard_line_->clear ();
}

void PolygonalMapDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PolygonalMapDisplay::setColor(const Color & color)
{
  color_ = color;

  propertyChanged(color_property_);

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = PolygonalMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void PolygonalMapDisplay::setOverrideColor(bool override)
{
  override_color_ = override;

  propertyChanged(override_color_property_);

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = PolygonalMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void PolygonalMapDisplay::setRenderOperation(int op)
{
  render_operation_ = op;

  propertyChanged(render_operation_property_);

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = PolygonalMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void PolygonalMapDisplay::setLineWidth (float width)
{
  line_width_ = width;

  propertyChanged(line_width_property_);

  message_mutex_.lock ();
  new_message_ = current_message_;
  processMessage ();
  new_message_ = PolygonalMapPtr ();
  message_mutex_.unlock ();
  causeRender ();
}

void PolygonalMapDisplay::setPointSize (float size)
{
  point_size_ = size;

  propertyChanged(point_size_property_);

  cloud_->setBillboardDimensions (size, size);
  causeRender ();
}

void PolygonalMapDisplay::setZPosition(float z)
{
  z_position_ = z;

  propertyChanged(z_position_property_);

  scene_node_->setPosition(0.0f, z, 0.0f);
  causeRender();
}

void PolygonalMapDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;
  cloud_->setAlpha (alpha);

  propertyChanged(alpha_property_);

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = PolygonalMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void PolygonalMapDisplay::subscribe()
{
  if (!isEnabled())
    return;

  notifier_->setTopic(topic_);
}

void PolygonalMapDisplay::unsubscribe()
{
  notifier_->setTopic("");
}

void PolygonalMapDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void PolygonalMapDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible(false);
}

void PolygonalMapDisplay::fixedFrameChanged()
{
  notifier_->setTargetFrame( fixed_frame_ );
  clear();
}

void PolygonalMapDisplay::update(float wall_dt, float ros_dt)
{
  message_mutex_.lock();
  if (new_message_)
  {
    processMessage();
    current_message_ = new_message_;
    new_message_ = PolygonalMapPtr();
    causeRender();
  }
  message_mutex_.unlock();
}

void PolygonalMapDisplay::processMessage ()
{
  if (!new_message_)
  {
    return;
  }

  clear ();

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f),
      btVector3(0.0f, 0.0f, z_position_)), new_message_->header.stamp,
      new_message_->header.frame_id);

  try
  {
    tf_->transformPose(fixed_frame_, pose, pose);
  }
  catch (tf::TransformException & e)
  {
    ROS_ERROR("Error transforming from frame 'map' to frame '%s'",
        fixed_frame_.c_str());
  }

  Ogre::Vector3 position = Ogre::Vector3(pose.getOrigin().x(),
      pose.getOrigin().y(), pose.getOrigin().z());
  robotToOgre(position);

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX(yaw, pitch, roll);

  Ogre::Matrix3 orientation(ogreMatrixFromRobotEulers(yaw, pitch, roll));

  Ogre::ColourValue color;

  uint32_t num_polygons = new_message_->get_polygons_size ();
  uint32_t num_total_points = 0;
  for (uint32_t i = 0; i < num_polygons; i++)
    num_total_points += new_message_->polygons[i].points.size ();

  // If we render points, we don't care about the order
  if (render_operation_ == polygon_render_ops::PPoints)
  {
    typedef std::vector<ogre_tools::PointCloud::Point> V_Point;
    V_Point points;
    points.resize (num_total_points);
    uint32_t cnt_total_points = 0;
    for (uint32_t i = 0; i < num_polygons; i++)
    {
      for (uint32_t j = 0; j < new_message_->polygons[i].points.size(); j++)
      {
        ogre_tools::PointCloud::Point &current_point = points[cnt_total_points];

        current_point.x_ = new_message_->polygons[i].points[j].x;
        current_point.y_ = new_message_->polygons[i].points[j].y;
        current_point.z_ = new_message_->polygons[i].points[j].z;
        if (override_color_)
          color = Ogre::ColourValue (color_.r_, color_.g_, color_.b_, alpha_);
        else
          color = Ogre::ColourValue (new_message_->polygons[i].color.r,
                                     new_message_->polygons[i].color.g,
                                     new_message_->polygons[i].color.b, alpha_);
        current_point.r_ = color.r;
        current_point.g_ = color.g;
        current_point.b_ = color.b;
        cnt_total_points++;
      }
    }

    cloud_->clear();

    if (!points.empty())
      cloud_->addPoints(&points.front(), points.size());
  }
  // Lines ?
  else if (render_operation_ == polygon_render_ops::PLines)
  {
    for (uint32_t i = 0; i < num_polygons; i++)
    {
      manual_object_->estimateVertexCount (new_message_->polygons[i].points.size ());
      manual_object_->begin ("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
      for (uint32_t j = 0; j < new_message_->polygons[i].points.size (); j++)
      {
        manual_object_->position (new_message_->polygons[i].points[j].x,
                                  new_message_->polygons[i].points[j].y,
                                  new_message_->polygons[i].points[j].z);
        if (override_color_)
          color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
        else
          color = Ogre::ColourValue (new_message_->polygons[i].color.r,
                                     new_message_->polygons[i].color.g,
                                     new_message_->polygons[i].color.b, alpha_);
        manual_object_->colour (color);
      }
      manual_object_->end ();
    }
  }
  // Line Billboards ?
  else if (render_operation_ == polygon_render_ops::PBillboards)
  {
    billboard_line_->setMaxPointsPerLine (2);
    billboard_line_->setLineWidth (line_width_);
    if (override_color_)
      billboard_line_->setColor (color_.r_, color_.g_, color_.b_, alpha_);

    billboard_line_->setNumLines (num_total_points);

    // Go over all polygons
    for (uint32_t i = 0; i < num_polygons; i++)
    {
      // Create lines connecting the points from the current polygon
      if (new_message_->polygons[i].points.size () == 0)
        continue;
      uint32_t j = 0;
      for (j = 0; j < new_message_->polygons[i].points.size () - 1; j++)
      {
        Ogre::Vector3 p1 (new_message_->polygons[i].points[j].x,
                          new_message_->polygons[i].points[j].y,
                          new_message_->polygons[i].points[j].z);
        Ogre::Vector3 p2 (new_message_->polygons[i].points[j+1].x,
                          new_message_->polygons[i].points[j+1].y,
                          new_message_->polygons[i].points[j+1].z);

        billboard_line_->newLine ();
        billboard_line_->addPoint (p1);
        billboard_line_->addPoint (p2);
      }
      if (!override_color_)
        billboard_line_->setColor (new_message_->polygons[i].color.r, new_message_->polygons[i].color.g, new_message_->polygons[i].color.b, alpha_);
    }
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void PolygonalMapDisplay::incomingMessage(const PolygonalMapPtr& message)
{
  message_mutex_.lock();
  new_message_ = message;
  message_mutex_.unlock();
}

void PolygonalMapDisplay::reset()
{
  clear();
}

void PolygonalMapDisplay::targetFrameChanged()
{
}

void PolygonalMapDisplay::createProperties()
{
  override_color_property_ = property_manager_->createProperty<BoolProperty> ("Override Color", property_prefix_, boost::bind(&PolygonalMapDisplay::getOverrideColor, this),
                                                                              boost::bind(&PolygonalMapDisplay::setOverrideColor, this, _1), category_, this);
  color_property_ = property_manager_->createProperty<ColorProperty> ("Color", property_prefix_, boost::bind(&PolygonalMapDisplay::getColor, this),
                                                                      boost::bind(&PolygonalMapDisplay::setColor, this, _1), category_, this);
  render_operation_property_ = property_manager_->createProperty<EnumProperty> ("Render Operation", property_prefix_, boost::bind(&PolygonalMapDisplay::getRenderOperation, this),
                                                                                boost::bind(&PolygonalMapDisplay::setRenderOperation, this, _1), category_, this);
  EnumPropertyPtr enum_prop = render_operation_property_.lock();
  enum_prop->addOption("Lines", polygon_render_ops::PLines);
  enum_prop->addOption("Points", polygon_render_ops::PPoints);
  enum_prop->addOption("Billboard Lines", polygon_render_ops::PBillboards);

  line_width_property_ = property_manager_->createProperty<FloatProperty> ("Line Width", property_prefix_, boost::bind (&PolygonalMapDisplay::getLineWidth, this),
                                                                           boost::bind (&PolygonalMapDisplay::setLineWidth, this, _1 ), category_, this );
  FloatPropertyPtr float_prop = line_width_property_.lock();
  float_prop->setMin (0.001);

  point_size_property_ = property_manager_->createProperty<FloatProperty>("Point Size", property_prefix_, boost::bind(&PolygonalMapDisplay::getPointSize, this),
                                                                          boost::bind( &PolygonalMapDisplay::setPointSize, this, _1 ), category_, this);

  z_position_property_ = property_manager_->createProperty<FloatProperty> ("Z Position", property_prefix_, boost::bind(&PolygonalMapDisplay::getZPosition, this),
                                                                           boost::bind(&PolygonalMapDisplay::setZPosition, this, _1), category_, this);
  alpha_property_ = property_manager_->createProperty<FloatProperty> ("Alpha", property_prefix_, boost::bind(&PolygonalMapDisplay::getAlpha, this),
                                                                      boost::bind(&PolygonalMapDisplay::setAlpha, this, _1), category_,this);
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&PolygonalMapDisplay::getTopic, this),
                                                                               boost::bind(&PolygonalMapDisplay::setTopic, this, _1), category_, this);
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(robot_msgs::PolygonalMap::__s_getDataType());


}

const char*
PolygonalMapDisplay::getDescription()
{
  return ("Displays data from a robot_msgs::PolygonalMap message as either points, billboards, or lines.");
}

} // namespace rviz
