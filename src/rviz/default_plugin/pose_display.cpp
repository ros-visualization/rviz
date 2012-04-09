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

#include "pose_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/axes.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

class PoseDisplaySelectionHandler : public SelectionHandler
{
public:
  PoseDisplaySelectionHandler(const std::string& name)
  : name_(name)
  {}

  void createProperties(const Picked& obj, PropertyManager* property_manager)
  {
    std::stringstream prefix;
    prefix << "Pose " << name_;
    CategoryPropertyWPtr cat = property_manager->createCategory(prefix.str(), prefix.str());
    properties_.push_back(property_manager->createProperty<StringProperty>("Frame", prefix.str(), boost::bind(&PoseDisplaySelectionHandler::getFrame, this), StringProperty::Setter(), cat));
    properties_.push_back(property_manager->createProperty<Vector3Property>("Position", prefix.str(), boost::bind(&PoseDisplaySelectionHandler::getPosition, this), Vector3Property::Setter(), cat));
    properties_.push_back(property_manager->createProperty<QuaternionProperty>("Orientation", prefix.str(), boost::bind(&PoseDisplaySelectionHandler::getOrientation, this), QuaternionProperty::Setter(), cat));
  }

  void setMessage(const geometry_msgs::PoseStampedConstPtr& message)
  {
    message_ = message;
  }

  std::string getFrame()
  {
    return message_->header.frame_id;
  }

  Ogre::Vector3 getPosition()
  {
    return Ogre::Vector3(message_->pose.position.x, message_->pose.position.y, message_->pose.position.z);
  }

  Ogre::Quaternion getOrientation()
  {
    return Ogre::Quaternion(message_->pose.orientation.x, message_->pose.orientation.y, message_->pose.orientation.z, message_->pose.orientation.w);
  }

private:
  std::string name_;
  geometry_msgs::PoseStampedConstPtr message_;
};

PoseDisplay::PoseDisplay()
  : Display()
  , color_( 1.0f, 0.1f, 0.0f )
  , head_radius_(0.2)
  , head_length_(0.3)
  , shaft_radius_(0.1)
  , shaft_length_(1.0)
  , axes_length_(1.0)
  , axes_radius_(0.1)
  , messages_received_(0)
{
}

void PoseDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*vis_manager_->getTFClient(), "", 5, update_nh_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&PoseDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);

  arrow_ = new rviz::Arrow(scene_manager_, scene_node_, shaft_length_, shaft_radius_, head_length_, head_radius_);
  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow_->setOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));

  axes_ = new rviz::Axes(scene_manager_, scene_node_, axes_length_, axes_radius_);

  setVisibility();

  Ogre::Quaternion quat(Ogre::Quaternion::IDENTITY);
  axes_->setOrientation(quat);

  SelectionManager* sel_manager = vis_manager_->getSelectionManager();
  coll_handler_.reset(new PoseDisplaySelectionHandler(name_));
  coll_ = sel_manager->createCollisionForObject(arrow_, coll_handler_);
  sel_manager->createCollisionForObject(axes_, coll_handler_, coll_);

  setShape(Arrow);
  setAlpha(1.0);
}

PoseDisplay::~PoseDisplay()
{
  unsubscribe();

  clear();

  SelectionManager* sel_manager = vis_manager_->getSelectionManager();
  sel_manager->removeObject(coll_);

  delete arrow_;
  delete axes_;
  delete tf_filter_;
}

void PoseDisplay::clear()
{
  tf_filter_->clear();
  latest_message_.reset();
  setVisibility();

  messages_received_ = 0;
  setStatus(status_levels::Warn, "Topic", "No messages received");
}

void PoseDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PoseDisplay::setColor( const Color& color )
{
  color_ = color;

  arrow_->setColor(color.r_, color.g_, color.b_, alpha_);
  axes_->setColor(color.r_, color.g_, color.b_, alpha_);

  propertyChanged(color_property_);

  causeRender();
}

void PoseDisplay::setAlpha( float a )
{
  alpha_ = a;

  arrow_->setColor(color_.r_, color_.g_, color_.b_, alpha_);
  axes_->setColor(color_.r_, color_.g_, color_.b_, alpha_);

  propertyChanged(alpha_property_);

  causeRender();
}

void PoseDisplay::setHeadRadius(float r)
{
  head_radius_ = r;
  arrow_->set(shaft_length_, shaft_radius_, head_length_, head_radius_);
  propertyChanged(head_radius_property_);
}

void PoseDisplay::setHeadLength(float l)
{
  head_length_ = l;
  arrow_->set(shaft_length_, shaft_radius_, head_length_, head_radius_);
  propertyChanged(head_length_property_);
}

void PoseDisplay::setShaftRadius(float r)
{
  shaft_radius_ = r;
  arrow_->set(shaft_length_, shaft_radius_, head_length_, head_radius_);
  propertyChanged(shaft_radius_property_);
}

void PoseDisplay::setShaftLength(float l)
{
  shaft_length_ = l;
  arrow_->set(shaft_length_, shaft_radius_, head_length_, head_radius_);
  propertyChanged(shaft_length_property_);
}

void PoseDisplay::setAxesRadius(float r)
{
  axes_radius_ = r;
  axes_->set(axes_length_, axes_radius_);
  propertyChanged(axes_radius_property_);
}

void PoseDisplay::setAxesLength(float l)
{
  axes_length_ = l;
  axes_->set(axes_length_, axes_radius_);
  propertyChanged(axes_length_property_);
}

void PoseDisplay::setShape(int shape)
{
  current_shape_ = (Shape)shape;

  setVisibility();

  propertyChanged(shape_property_);

  createShapeProperties();

  causeRender();
}

void PoseDisplay::setVisibility()
{
  arrow_->getSceneNode()->setVisible(false);
  axes_->getSceneNode()->setVisible(false);

  if (!latest_message_)
  {
    return;
  }

  switch (current_shape_)
  {
  case Arrow:
    arrow_->getSceneNode()->setVisible(true);
    break;
  case Axes:
    axes_->getSceneNode()->setVisible(true);
    break;
  }
}

void PoseDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe(update_nh_, topic_, 5);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void PoseDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PoseDisplay::onEnable()
{
  setVisibility();

  subscribe();
}

void PoseDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void PoseDisplay::createShapeProperties()
{
  if (!property_manager_)
  {
    return;
  }

  property_manager_->deleteProperty(shape_category_.lock());
  shape_category_ = property_manager_->createCategory("Shape Properties", property_prefix_, parent_category_, this);

  switch (current_shape_)
  {
  case Arrow:
    {
      color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PoseDisplay::getColor, this ),
                                                                          boost::bind( &PoseDisplay::setColor, this, _1 ), shape_category_, this );
      setPropertyHelpText(color_property_, "Color to draw the arrow.");
      alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &PoseDisplay::getAlpha, this ),
                                                                          boost::bind( &PoseDisplay::setAlpha, this, _1 ), shape_category_, this );
      setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the arrow.");
      FloatPropertyPtr float_prop = alpha_property_.lock();
      float_prop->setMin(0.0);
      float_prop->setMax(1.0);

      shaft_length_property_ = property_manager_->createProperty<FloatProperty>( "Shaft Length", property_prefix_, boost::bind( &PoseDisplay::getShaftLength, this ),
                                                                                 boost::bind( &PoseDisplay::setShaftLength, this, _1 ), shape_category_, this );
      setPropertyHelpText(shaft_length_property_, "Length of the arrow's shaft, in meters.");
      shaft_radius_property_ = property_manager_->createProperty<FloatProperty>( "Shaft Radius", property_prefix_, boost::bind( &PoseDisplay::getShaftRadius, this ),
                                                                                 boost::bind( &PoseDisplay::setShaftRadius, this, _1 ), shape_category_, this );
      setPropertyHelpText(shaft_radius_property_, "Radius of the arrow's shaft, in meters.");
      head_length_property_ = property_manager_->createProperty<FloatProperty>( "Head Length", property_prefix_, boost::bind( &PoseDisplay::getHeadLength, this ),
                                                                                  boost::bind( &PoseDisplay::setHeadLength, this, _1 ), shape_category_, this );
      setPropertyHelpText(head_length_property_, "Length of the arrow's head, in meters.");
      head_radius_property_ = property_manager_->createProperty<FloatProperty>( "Head Radius", property_prefix_, boost::bind( &PoseDisplay::getHeadRadius, this ),
                                                                                 boost::bind( &PoseDisplay::setHeadRadius, this, _1 ), shape_category_, this );
      setPropertyHelpText(head_radius_property_, "Radius of the arrow's head, in meters.");
    }
    break;
  case Axes:
    axes_length_property_ = property_manager_->createProperty<FloatProperty>( "Axes Length", property_prefix_, boost::bind( &PoseDisplay::getAxesLength, this ),
                                                                                boost::bind( &PoseDisplay::setAxesLength, this, _1 ), shape_category_, this );
    setPropertyHelpText(axes_length_property_, "Length of each axis, in meters.");
    axes_radius_property_ = property_manager_->createProperty<FloatProperty>( "Axes Radius", property_prefix_, boost::bind( &PoseDisplay::getAxesRadius, this ),
                                                                               boost::bind( &PoseDisplay::setAxesRadius, this, _1 ), shape_category_, this );
    setPropertyHelpText(axes_radius_property_, "Radius of each axis, in meters.");
    break;
  }

}

void PoseDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PoseDisplay::getTopic, this ),
                                                                                boost::bind( &PoseDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "geometry_msgs::PoseStamped topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<geometry_msgs::PoseStamped>());

  shape_property_ = property_manager_->createProperty<EnumProperty>( "Shape", property_prefix_, boost::bind( &PoseDisplay::getShape, this ),
                                                                     boost::bind( &PoseDisplay::setShape, this, _1 ), parent_category_, this );
  setPropertyHelpText(shape_property_, "Shape to display the pose as.");
  EnumPropertyPtr enum_prop = shape_property_.lock();
  enum_prop->addOption( "Arrow", Arrow );
  enum_prop->addOption( "Axes", Axes );

  createShapeProperties();
}

void PoseDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );
  clear();
}

void PoseDisplay::update(float wall_dt, float ros_dt)
{
}

void PoseDisplay::incomingMessage( const geometry_msgs::PoseStamped::ConstPtr& message )
{
  ++messages_received_;

  if (!validateFloats(*message))
  {
    setStatus(status_levels::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!vis_manager_->getFrameManager()->transform(message->header, message->pose, position, orientation))
  {
    ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'", name_.c_str(), message->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  latest_message_ = message;
  coll_handler_->setMessage(message);
  setVisibility();

  causeRender();
}

void PoseDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz
