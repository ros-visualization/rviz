#ifndef SCREW_DISPLAY_H
#define SCREW_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <rviz/message_filter_display.h>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class ROSTopicStringProperty;
class FloatProperty;
class IntProperty;
} // namespace rviz

namespace rviz
{
class ScrewVisual;

template <class MessageType>
class RVIZ_EXPORT ScrewDisplay : public rviz::MessageFilterDisplay<MessageType>
{
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ScrewDisplay();
  ~ScrewDisplay() override = default;

protected:
  // Overrides of public virtual functions from the Display class.
  void onInitialize() override;
  void reset() override;

  // Helper function to properties for all visuals.
  void updateProperties();
  void updateHistoryLength();

  void processMessagePrivate(const std_msgs::Header& header,
                             const geometry_msgs::Vector3& linear,
                             const geometry_msgs::Vector3& angular);

  // Storage for the list of visuals par each joint intem
  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<ScrewVisual> > visuals_;

  // Property objects for user-editable properties.
  rviz::ColorProperty *linear_color_property_, *angular_color_property_;
  rviz::FloatProperty *alpha_property_, *linear_scale_property_, *angular_scale_property_,
      *width_property_;
  rviz::IntProperty* history_length_property_;
  rviz::BoolProperty* hide_small_values_property_;
};

class AccelStampedDisplay : public ScrewDisplay<geometry_msgs::AccelStamped>
{
  Q_OBJECT

  // Function to handle an incoming ROS message.
  void processMessage(const geometry_msgs::AccelStamped::ConstPtr& msg) override
  {
    processMessagePrivate(msg->header, msg->accel.linear, msg->accel.angular);
  }
};

class TwistStampedDisplay : public ScrewDisplay<geometry_msgs::TwistStamped>
{
  Q_OBJECT

  // Function to handle an incoming ROS message.
  void processMessage(const geometry_msgs::TwistStamped::ConstPtr& msg) override
  {
    processMessagePrivate(msg->header, msg->twist.linear, msg->twist.angular);
  }
};

class WrenchStampedDisplay : public ScrewDisplay<geometry_msgs::WrenchStamped>
{
  // Function to handle an incoming ROS message.
  void processMessage(const geometry_msgs::WrenchStamped::ConstPtr& msg) override
  {
    processMessagePrivate(msg->header, msg->wrench.force, msg->wrench.torque);
  }
};
} // namespace rviz

#endif // SCREW_DISPLAY_H
