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
 *
 */

#include "polygonal_map_display.h"
#include "visualization_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include "ogre_tools/arrow.h"

#include <tf/transform_listener.h>

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
, tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Polygonal Map" << count++;
  manual_object_ = scene_manager_->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  cloud_ = new ogre_tools::PointCloud ();
  scene_node_->attachObject(cloud_);
  billboard_line_ = new ogre_tools::BillboardLine (scene_manager_, scene_node_);

  setAlpha (1.0f);
  setPointSize (0.02f);
  setLineWidth(0.02f);

  tf_filter_.connectTo(sub_);
  tf_filter_.connect(boost::bind(&PolygonalMapDisplay::incomingMessage, this, _1));
}

PolygonalMapDisplay::~PolygonalMapDisplay()
{
  unsubscribe();
  clear();

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

  processMessage(current_message_);
  causeRender();
}

void PolygonalMapDisplay::setOverrideColor(bool override)
{
  override_color_ = override;

  propertyChanged(override_color_property_);

  processMessage(current_message_);
  causeRender();
}

void PolygonalMapDisplay::setRenderOperation(int op)
{
  render_operation_ = op;

  propertyChanged(render_operation_property_);

  processMessage(current_message_);
  causeRender();
}

void PolygonalMapDisplay::setLineWidth (float width)
{
  line_width_ = width;

  propertyChanged(line_width_property_);

  processMessage(current_message_);
  causeRender ();
}

void PolygonalMapDisplay::setPointSize (float size)
{
  point_size_ = size;

  propertyChanged(point_size_property_);

  cloud_->setDimensions (size, size, size);
  causeRender ();
}


void PolygonalMapDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;
  cloud_->setAlpha (alpha);

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void PolygonalMapDisplay::subscribe()
{
  if (!isEnabled())
    return;

  sub_.subscribe(update_nh_, topic_, 1);
}

void PolygonalMapDisplay::unsubscribe()
{
  sub_.unsubscribe();
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
  tf_filter_.setTargetFrame( fixed_frame_ );
  clear();
}

void PolygonalMapDisplay::update(float wall_dt, float ros_dt)
{
}

void PolygonalMapDisplay::processMessage (const mapping_msgs::PolygonalMap::ConstPtr& msg)
{
  clear();

  if (!msg)
  {
    return;
  }

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f),
      btVector3(0.0f, 0.0f, 0.0f)), msg->header.stamp,
      msg->header.frame_id);

  try
  {
    vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
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

  uint32_t num_polygons = msg->get_polygons_size ();
  uint32_t num_total_points = 0;
  for (uint32_t i = 0; i < num_polygons; i++)
    num_total_points += msg->polygons[i].points.size ();

  // If we render points, we don't care about the order
  if (render_operation_ == polygon_render_ops::PPoints)
  {
    typedef std::vector<ogre_tools::PointCloud::Point> V_Point;
    V_Point points;
    points.resize (num_total_points);
    uint32_t cnt_total_points = 0;
    for (uint32_t i = 0; i < num_polygons; i++)
    {
      for (uint32_t j = 0; j < msg->polygons[i].points.size(); j++)
      {
        ogre_tools::PointCloud::Point &current_point = points[cnt_total_points];

        current_point.x = msg->polygons[i].points[j].x;
        current_point.y = msg->polygons[i].points[j].y;
        current_point.z = msg->polygons[i].points[j].z;
        if (override_color_)
          color = Ogre::ColourValue (color_.r_, color_.g_, color_.b_, alpha_);
        else
          color = Ogre::ColourValue (msg->polygons[i].color.r,
                                     msg->polygons[i].color.g,
                                     msg->polygons[i].color.b, alpha_);
        current_point.setColor(color.r, color.g, color.b);
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
      manual_object_->estimateVertexCount (msg->polygons[i].points.size ());
      manual_object_->begin ("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
      for (uint32_t j = 0; j < msg->polygons[i].points.size (); j++)
      {
        manual_object_->position (msg->polygons[i].points[j].x,
                                  msg->polygons[i].points[j].y,
                                  msg->polygons[i].points[j].z);
        if (override_color_)
          color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
        else
          color = Ogre::ColourValue (msg->polygons[i].color.r,
                                     msg->polygons[i].color.g,
                                     msg->polygons[i].color.b, alpha_);
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
      if (msg->polygons[i].points.size () == 0)
        continue;
      uint32_t j = 0;
      for (j = 0; j < msg->polygons[i].points.size () - 1; j++)
      {
        Ogre::Vector3 p1 (msg->polygons[i].points[j].x,
                          msg->polygons[i].points[j].y,
                          msg->polygons[i].points[j].z);
        Ogre::Vector3 p2 (msg->polygons[i].points[j+1].x,
                          msg->polygons[i].points[j+1].y,
                          msg->polygons[i].points[j+1].z);

        billboard_line_->newLine ();
        billboard_line_->addPoint (p1);
        billboard_line_->addPoint (p2);
      }
      if (!override_color_)
        billboard_line_->setColor (msg->polygons[i].color.r, msg->polygons[i].color.g, msg->polygons[i].color.b, alpha_);
    }
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void PolygonalMapDisplay::incomingMessage(const mapping_msgs::PolygonalMap::ConstPtr& msg)
{
  processMessage(msg);
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

  alpha_property_ = property_manager_->createProperty<FloatProperty> ("Alpha", property_prefix_, boost::bind(&PolygonalMapDisplay::getAlpha, this),
                                                                      boost::bind(&PolygonalMapDisplay::setAlpha, this, _1), category_,this);
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&PolygonalMapDisplay::getTopic, this),
                                                                               boost::bind(&PolygonalMapDisplay::setTopic, this, _1), category_, this);
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(mapping_msgs::PolygonalMap::__s_getDataType());


}

const char*
PolygonalMapDisplay::getDescription()
{
  return ("Displays data from a mapping_msgs::PolygonalMap message as either points, billboards, or lines.");
}

} // namespace rviz
