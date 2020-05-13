#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/validate_floats.h>

#include <boost/foreach.hpp>

#include "screw_visual.h"
#include "screw_display.h"

template <class MessageType>
constexpr const char* linear()
{
  return "Linear";
}
template <class MessageType>
constexpr const char* angular()
{
  return "Angular";
}

template <>
constexpr const char* linear<geometry_msgs::WrenchStamped>()
{
  return "Force";
}
template <>
constexpr const char* angular<geometry_msgs::WrenchStamped>()
{
  return "Torque";
}

namespace rviz
{
template <class MessageType>
ScrewDisplay<MessageType>::ScrewDisplay()
{
  auto lin = linear<MessageType>();
  auto ang = angular<MessageType>();
  linear_color_property_ =
      new ColorProperty(QString("%1 Color").arg(lin), QColor(204, 51, 51),
                        QString("Color to draw the %1 arrows.").arg(QString(lin).toLower()));
  QObject::connect(linear_color_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  angular_color_property_ =
      new ColorProperty(QString("%1 Color").arg(ang), QColor(204, 204, 51),
                        QString("Color to draw the %1 arrows.").arg(QString(ang).toLower()));
  QObject::connect(angular_color_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  alpha_property_ = new FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.");
  QObject::connect(alpha_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  linear_scale_property_ =
      new FloatProperty(QString("%1 Arrow Scale").arg(lin), 2.0, QString("%1 arrow scale").arg(lin));
  QObject::connect(linear_scale_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  angular_scale_property_ =
      new FloatProperty(QString("%1 Arrow Scale").arg(ang), 2.0, QString("%1 arrow scale").arg(ang));
  QObject::connect(angular_scale_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  width_property_ = new FloatProperty("Arrow Width", 0.5, "arrow width");
  QObject::connect(width_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  history_length_property_ =
      new IntProperty("History Length", 1, "Number of prior measurements to display.");
  QObject::connect(history_length_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateHistoryLength);

  hide_small_values_property_ = new BoolProperty("Hide Small Values", true, "Hide small values");
  QObject::connect(hide_small_values_property_, &rviz::Property::changed, this,
                   &ScrewDisplay<MessageType>::updateProperties);

  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}


template <class MessageType>
void ScrewDisplay<MessageType>::onInitialize()
{
  MessageFilterDisplay<MessageType>::onInitialize();
  updateHistoryLength();
}

// Override rviz::Display's reset() function to add a call to clear().
template <class MessageType>
void ScrewDisplay<MessageType>::reset()
{
  MessageFilterDisplay<MessageType>::reset();
  visuals_.clear();
}

template <class MessageType>
void ScrewDisplay<MessageType>::updateProperties()
{
  float alpha = alpha_property_->getFloat();
  float linear_scale = linear_scale_property_->getFloat();
  float angular_scale = angular_scale_property_->getFloat();
  float width = width_property_->getFloat();
  bool hide_small_values = hide_small_values_property_->getBool();
  Ogre::ColourValue linear_color = linear_color_property_->getOgreColor();
  Ogre::ColourValue angular_color = angular_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setLinearColor(linear_color.r, linear_color.g, linear_color.b, alpha);
    visuals_[i]->setAngularColor(angular_color.r, angular_color.g, angular_color.b, alpha);
    visuals_[i]->setLinearScale(linear_scale);
    visuals_[i]->setAngularScale(angular_scale);
    visuals_[i]->setWidth(width);
    visuals_[i]->setHideSmallValues(hide_small_values);
  }
}

// Set the number of past visuals to show.
template <class MessageType>
void ScrewDisplay<MessageType>::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// This is our callback to handle an incoming message.
template <class MessageType>
void ScrewDisplay<MessageType>::processMessagePrivate(const std_msgs::Header& header,
                                                      const geometry_msgs::Vector3& linear,
                                                      const geometry_msgs::Vector3& angular)
{
  if (!(validateFloats(linear) && validateFloats(angular)))
  {
    Display::setStatus(rviz::StatusProperty::Error, "Topic",
                       "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!Display::context_->getFrameManager()->getTransform(header.frame_id, header.stamp, position,
                                                          orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", header.frame_id.c_str(),
              qPrintable(Display::fixed_frame_));
    return;
  }

  // We are keeping a circular buffer of visual pointers.
  // This gets the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<ScrewVisual> visual;
  if (visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ScrewVisual(Display::context_->getSceneManager(), Display::scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual->setScrew(linear, angular);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  float alpha = alpha_property_->getFloat();
  float linear_scale = linear_scale_property_->getFloat();
  float angular_scale = angular_scale_property_->getFloat();
  float width = width_property_->getFloat();
  Ogre::ColourValue linear_color = linear_color_property_->getOgreColor();
  Ogre::ColourValue angular_color = angular_color_property_->getOgreColor();
  visual->setLinearColor(linear_color.r, linear_color.g, linear_color.b, alpha);
  visual->setAngularColor(angular_color.r, angular_color.g, angular_color.b, alpha);
  visual->setLinearScale(linear_scale);
  visual->setAngularScale(angular_scale);
  visual->setWidth(width);
  visual->setScrew(linear, angular);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace rviz

// Tell pluginlib about these classes.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::AccelStampedDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(rviz::TwistStampedDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(rviz::WrenchStampedDisplay, rviz::Display)
