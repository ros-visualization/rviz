#include "range_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{
RangeDisplay::RangeDisplay()
  : Display()
  , color_( 1.0f, 1.0f, 1.0f )
  , messages_received_(0)
{
}

void RangeDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<sensor_msgs::Range>(*vis_manager_->getTFClient(), "", 10, update_nh_);

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->setVisible( false );
  
  setBuffer( 1 );
  Ogre::Vector3 scale( 0, 0, 0);

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&RangeDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
  setAlpha( 0.5f );
}

RangeDisplay::~RangeDisplay()
{
  unsubscribe();
  clear();
  for (size_t i = 0; i < cones_.size(); i++) {
    delete cones_[i];
  }

  delete tf_filter_;
}

void RangeDisplay::clear()
{
  setBuffer( cones_.size() );
  tf_filter_->clear();
  messages_received_ = 0;
  setStatus(rviz::status_levels::Warn, "Topic", "No messages received");
}

void RangeDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void RangeDisplay::setColor( const rviz::Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage(current_message_);
  causeRender();
}

void RangeDisplay::setBuffer( int buffer )
{
  if(buffer < 1)
    buffer = 1;
  buffer_len_ = buffer;

  propertyChanged(bufferLen_property_);
  
  for (size_t i = 0; i < cones_.size(); i++) {
    delete cones_[i];
  }
  cones_.resize(buffer_len_);
  for (size_t i = 0; i < cones_.size(); i++) {
    cones_[i] = new Shape(Shape::Cone, vis_manager_->getSceneManager(), scene_node_);
    Shape* cone = cones_[i];
    
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    geometry_msgs::Pose pose;
    pose.position.z = pose.position.y = 0;
    pose.position.x = 0;
    pose.orientation.x = 0;
    pose.orientation.z = 0;
    Ogre::Vector3 scale( 0, 0, 0);
    cone->setScale(scale);
    cone->setColor(color_.r_, color_.g_, color_.b_, 0);
  }
}

void RangeDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void RangeDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe(update_nh_, topic_, 10);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void RangeDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void RangeDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void RangeDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void RangeDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_ );
  clear();
}

void RangeDisplay::update(float wall_dt, float ros_dt)
{
}


void RangeDisplay::processMessage(const sensor_msgs::Range::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;
  
  Shape* cone = cones_[messages_received_ % buffer_len_];

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz::status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = pose.position.y = 0;
  pose.position.x = msg->range/2 - .008824 * msg->range; // .008824 fudge factor measured, must be inaccuracy of cone model.
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.707;
  if( !vis_manager_->getFrameManager()->transform( msg->header.frame_id, msg->header.stamp, pose, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  cone->setPosition(position);
  cone->setOrientation(orientation); 

  double cone_width = 2.0 * msg->range * tan( msg->field_of_view / 2.0 );
  Ogre::Vector3 scale( cone_width, msg->range, cone_width );
  cone->setScale(scale);
  cone->setColor(color_.r_, color_.g_, color_.b_, alpha_);

}

void RangeDisplay::incomingMessage(const sensor_msgs::Range::ConstPtr& msg)
{
  processMessage(msg);
}

void RangeDisplay::reset()
{
  Display::reset();
  clear();
}

void RangeDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &RangeDisplay::getTopic, this ),
                                                                                boost::bind( &RangeDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::Range topic to subscribe to.");
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Range>());
  color_property_ = property_manager_->createProperty<rviz::ColorProperty>( "Color", property_prefix_, boost::bind( &RangeDisplay::getColor, this ),
                                                                      boost::bind( &RangeDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "Color to draw the range.");
  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Alpha", property_prefix_, boost::bind( &RangeDisplay::getAlpha, this ),
                                                                       boost::bind( &RangeDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the range.");
  bufferLen_property_ = property_manager_->createProperty<rviz::IntProperty>( "Buffer Length", property_prefix_, boost::bind( &RangeDisplay::getBuffer, this ),
                                                                       boost::bind( &RangeDisplay::setBuffer, this, _1 ), parent_category_, this );
  setPropertyHelpText(bufferLen_property_, "Number of prior measurements to display.");
  
}

const char* RangeDisplay::getDescription()
{
  return "Displays data from a sensor_msgs::Range message as a cone.";
}
} // namespace rviz
