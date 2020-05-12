#ifndef WRENCHSTAMPED_DISPLAY_H
#define WRENCHSTAMPED_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <geometry_msgs/WrenchStamped.h>
#include <rviz/message_filter_display.h>

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
class WrenchVisual;

class WrenchStampedDisplay : public rviz::MessageFilterDisplay<geometry_msgs::WrenchStamped>
{
  Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  WrenchStampedDisplay();
  ~WrenchStampedDisplay() override;

protected:
  // Overrides of public virtual functions from the Display class.
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  // Helper function to properties for all visuals.
  void updateProperties();
  void updateHistoryLength();

private:
  // Function to handle an incoming ROS message.
  void processMessage(const geometry_msgs::WrenchStamped::ConstPtr& msg) override;

  // Storage for the list of visuals par each joint intem
  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<WrenchVisual> > visuals_;

  // Property objects for user-editable properties.
  rviz::ColorProperty *force_color_property_, *torque_color_property_;
  rviz::FloatProperty *alpha_property_, *force_scale_property_, *torque_scale_property_,
      *width_property_;
  rviz::IntProperty* history_length_property_;
  rviz::BoolProperty* hide_small_values_property_;
};
} // namespace rviz

#endif // WRENCHSTAMPED_DISPLAY_H
