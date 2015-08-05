#ifndef VECTOR3STAMPED_DISPLAY_H
#define VECTOR3STAMPED_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <geometry_msgs/Vector3Stamped.h>
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

    class Vector3StampedVisual;

    class Vector3StampedDisplay: public rviz::MessageFilterDisplay<geometry_msgs::Vector3Stamped>
    {
    Q_OBJECT
    public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling
	// the default constructor, so make sure you have one.
	Vector3StampedDisplay();
	virtual ~Vector3StampedDisplay();

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
	void processMessage( const geometry_msgs::Vector3Stamped::ConstPtr& msg );

	// Storage for the list of visuals par each joint intem
        // Storage for the list of visuals.  It is a circular buffer where
        // data gets popped from the front (oldest) and pushed to the back (newest)
        boost::circular_buffer<boost::shared_ptr<Vector3StampedVisual> > visuals_;

	// Property objects for user-editable properties.
        rviz::ColorProperty *vector_color_property_;
        rviz::FloatProperty *alpha_property_, *scale_property_, *width_property_;
	rviz::IntProperty *history_length_property_;
    };
} // end namespace rviz_plugin_tutorials

#endif // VECTOR3STAMPED_DISPLAY_H
