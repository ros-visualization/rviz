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
}

namespace rviz
{

class WrenchStampedVisual;

class WrenchStampedDisplay: public rviz::MessageFilterDisplay<geometry_msgs::WrenchStamped>
{
    Q_OBJECT
public:
    // Constructor.  pluginlib::ClassLoader creates instances by calling
    // the default constructor, so make sure you have one.
    WrenchStampedDisplay();
    virtual ~WrenchStampedDisplay();

protected:
    // Overrides of public virtual functions from the Display class.
    virtual void onInitialize();
    virtual void reset();

private Q_SLOTS:
    // Helper function to apply color and alpha to all visuals.
    void updateColorAndAlpha();
    void updateHistoryLength();

private:
    // Function to handle an incoming ROS message.
    void processMessage( const geometry_msgs::WrenchStamped::ConstPtr& msg );

    // Storage for the list of visuals par each joint intem
    // Storage for the list of visuals.  It is a circular buffer where
    // data gets popped from the front (oldest) and pushed to the back (newest)
    boost::circular_buffer<boost::shared_ptr<WrenchStampedVisual> > visuals_;

    // Property objects for user-editable properties.
    rviz::ColorProperty *force_color_property_, *torque_color_property_;
    rviz::FloatProperty *alpha_property_, *force_scale_property_, *torque_scale_property_, *width_property_;
    rviz::IntProperty *history_length_property_;
};
} // end namespace rviz_plugin_tutorials

#endif // WRENCHSTAMPED_DISPLAY_H
