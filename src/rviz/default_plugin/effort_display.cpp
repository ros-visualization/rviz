#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <boost/foreach.hpp>

#include "effort_visual.h"

#include "effort_display.h"

#include <urdf/model.h>

namespace rviz
{

    JointInfo::JointInfo(const std::string name, rviz::Property* parent_category) {
        name_ = name;
        effort_ = 0;
        max_effort_ = 0;
        last_update_ = ros::Time::now();

        //info->category_ = new Property( QString::fromStdString( info->name_ ) , QVariant(), "", joints_category_);
        category_ = new rviz::Property( QString::fromStdString( name_ ) , true, "", parent_category, SLOT( updateVisibility() ), this);

        effort_property_ = new rviz::FloatProperty( "Effort", 0, "Effort value of this joint.", category_);
        effort_property_->setReadOnly( true );

        max_effort_property_ = new rviz::FloatProperty( "Max Effort", 0, "Max Effort value of this joint.", category_);
        max_effort_property_->setReadOnly( true );
    }

    JointInfo::~JointInfo() {
    }

    void JointInfo::updateVisibility() {
        bool enabled = getEnabled();
    }

    void JointInfo::setEffort(double e) {
        effort_property_->setFloat(e);
        effort_ = e;
    }
    double JointInfo::getEffort() { return effort_; }
    void JointInfo::setMaxEffort(double m) {
        max_effort_property_->setFloat(m);
        max_effort_ = m;
    }
    double JointInfo::getMaxEffort() { return max_effort_; }

    bool JointInfo::getEnabled() const
    {
        return category_->getValue().toBool();
    }

    JointInfo* EffortDisplay::getJointInfo( const std::string& joint)
    {
        M_JointInfo::iterator it = joints_.find( joint );
        if ( it == joints_.end() )
            {
                return NULL;
            }

        return it->second;
    }

    JointInfo* EffortDisplay::createJoint(const std::string &joint)
    {
        JointInfo *info = new JointInfo(joint, this);
        joints_.insert( std::make_pair( joint, info ) );
        return info;
    }

    ///
    EffortDisplay::EffortDisplay()
    {
	alpha_property_ =
	    new rviz::FloatProperty( "Alpha", 1.0,
                                     "0 is fully transparent, 1.0 is fully opaque.",
                                     this, SLOT( updateColorAndAlpha() ));

	width_property_ =
	    new rviz::FloatProperty( "Width", 0.02,
                                     "Width to drow effort circle",
                                     this, SLOT( updateColorAndAlpha() ));

	scale_property_ =
	    new rviz::FloatProperty( "Scale", 1.0,
                                     "Scale to drow effort circle",
                                     this, SLOT( updateColorAndAlpha() ));

	history_length_property_ =
	    new rviz::IntProperty( "History Length", 1,
                                   "Number of prior measurements to display.",
                                   this, SLOT( updateHistoryLength() ));
        history_length_property_->setMin( 1 );
        history_length_property_->setMax( 100000 );


        robot_description_property_ =
            new rviz::StringProperty( "Robot Description", "robot_description",
                                      "Name of the parameter to search for to load the robot description.",
                                      this, SLOT( updateRobotDescription() ) );


        joints_category_ =
            new rviz::Property("Joints", QVariant(), "", this);
    }

    void EffortDisplay::onInitialize()
    {
        MFDClass::onInitialize();
        updateHistoryLength();
    }

    EffortDisplay::~EffortDisplay()
    {
        //delete robot_model_; // sharead pointer
    }

    // Clear the visuals by deleting their objects.
    void EffortDisplay::reset()
    {
        MFDClass::reset();
        visuals_.clear();
    }

    void EffortDisplay::clear()
    {
        clearStatuses();
        robot_description_.clear();
    }

    // Set the current color and alpha values for each visual.
    void EffortDisplay::updateColorAndAlpha()
    {
        float width = width_property_->getFloat();
        float scale = scale_property_->getFloat();

        for( size_t i = 0; i < visuals_.size(); i++ )
            {
                visuals_[ i ]->setWidth( width );
                visuals_[ i ]->setScale( scale );
            }
    }

    void EffortDisplay::updateRobotDescription()
    {
        if( isEnabled() )
            {
                load();
                context_->queueRender();
            }
    }

    // Set the number of past visuals to show.
    void EffortDisplay::updateHistoryLength( )
    {
        visuals_.rset_capacity(history_length_property_->getInt());
    }

    void EffortDisplay::load()
    {
	// get robot_description
	std::string content;
	if (!update_nh_.getParam( robot_description_property_->getStdString(), content) )  {
            std::string loc;
            if( update_nh_.searchParam( robot_description_property_->getStdString(), loc ))
                {
                    update_nh_.getParam( loc, content );
                }
            else
                {
                    clear();
                    setStatus( rviz::StatusProperty::Error, "URDF",
                               "Parameter [" + robot_description_property_->getString()
                               + "] does not exist, and was not found by searchParam()" );
                    return;
                }
            }

        if( content.empty() )
            {
                clear();
                setStatus( rviz::StatusProperty::Error, "URDF", "URDF is empty" );
                return;
            }

        if( content == robot_description_ )
            {
                return;
            }

        robot_description_ = content;


	robot_model_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
	if (!robot_model_->initString(content))
	{
	    ROS_ERROR("Unable to parse URDF description!");
            setStatus( rviz::StatusProperty::Error, "URDF", "Unable to parse robot model description!");
	    return;
	}
        setStatus(rviz::StatusProperty::Ok, "URDF", "Robot model parserd Ok");
    for (std::map<std::string, urdf::JointSharedPtr >::iterator it = robot_model_->joints_.begin(); it != robot_model_->joints_.end(); it ++ ) {
        urdf::JointSharedPtr joint = it->second;
	    if ( joint->type == urdf::Joint::REVOLUTE ) {
                std::string joint_name = it->first;
                urdf::JointLimitsSharedPtr limit = joint->limits;
                joints_[joint_name] = createJoint(joint_name);
                //joints_[joint_name]->max_effort_property_->setFloat(limit->effort);
                //joints_[joint_name]->max_effort_property_->setReadOnly( true );
                joints_[joint_name]->setMaxEffort(limit->effort);
            }
        }
    }

    void EffortDisplay::onEnable()
    {
        load();
    }

    void EffortDisplay::onDisable()
    {
        clear();
    }

#if 0
    void EffortDisplay::setRobotDescription( const std::string& description_param )
    {
        description_param_ = description_param;

        propertyChanged(robot_description_property_);

        if ( isEnabled() )
            {
                load();
                unsubscribe();
                subscribe();
                causeRender();
            }
    }

    void EffortDisplay::setAllEnabled(bool enabled)
    {
        all_enabled_ = enabled;

        M_JointInfo::iterator it = joints_.begin();
        M_JointInfo::iterator end = joints_.end();
        for (; it != end; ++it)
            {
                JointInfo* joint = it->second;

                setJointEnabled(joint, enabled);
            }

        propertyChanged(all_enabled_property_);
    }

#endif
    // This is our callback to handle an incoming message.
    void EffortDisplay::processMessage( const sensor_msgs::JointState::ConstPtr& msg )
    {
        // Robot model might not be loaded already
        if (!robot_model_)
        {
            return;
        }
        // We are keeping a circular buffer of visual pointers.  This gets
        // the next one, or creates and stores it if the buffer is not full
        boost::shared_ptr<EffortVisual> visual;
        if (visuals_.full())
        {
            visual = visuals_.front();
        }
        else
        {
            visual.reset(new EffortVisual(context_->getSceneManager(), scene_node_, robot_model_));
        }

        V_string joints;
        int joint_num = msg->name.size();
        if (joint_num != msg->effort.size())
        {
            std::string tmp_error = "Received a joint state msg with different joint names and efforts size!";
            ROS_ERROR("%s", tmp_error.c_str());
            setStatus(rviz::StatusProperty::Error, "TOPIC", QString::fromStdString(tmp_error));
            return;
        }
        for (int i = 0; i < joint_num; ++i)
        {
            std::string joint_name = msg->name[i];
            JointInfo* joint_info = getJointInfo(joint_name);
            if ( !joint_info ) continue; // skip joints..

            // set effort
            joint_info->setEffort(msg->effort[i]);

            // update effort property ???
            if ((ros::Time::now() - joint_info->last_update_) > ros::Duration(0.2))
            {
                joint_info->last_update_ = ros::Time::now();
            }

	    const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
	    int joint_type = joint->type;
	    if ( joint_type == urdf::Joint::REVOLUTE )
	    {
		// we expects that parent_link_name equals to frame_id.
		std::string parent_link_name = joint->child_link_name;
		Ogre::Quaternion orientation;
		Ogre::Vector3 position;

		// Here we call the rviz::FrameManager to get the transform from the
		// fixed frame to the frame in the header of this Effort message.  If
		// it fails, we can't do anything else so we return.
		if( !context_->getFrameManager()->getTransform( parent_link_name,
                                                                ros::Time(),
                                                                //msg->header.stamp, // ???
                                                                position, orientation ))
		{
		    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
			       parent_link_name.c_str(), qPrintable( fixed_frame_) );
		    continue;
		}
;
		tf::Vector3 axis_joint(joint->axis.x, joint->axis.y, joint->axis.z);
		tf::Vector3 axis_z(0,0,1);
		tf::Quaternion axis_rotation(tf::tfCross(axis_joint, axis_z), tf::tfAngle(axis_joint, axis_z));
		if ( std::isnan(axis_rotation.x()) ||
		     std::isnan(axis_rotation.y()) ||
		     std::isnan(axis_rotation.z()) ) axis_rotation = tf::Quaternion::getIdentity();

		tf::Quaternion axis_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
		tf::Quaternion axis_rot = axis_orientation * axis_rotation;
		Ogre::Quaternion joint_orientation(Ogre::Real(axis_rot.w()), Ogre::Real(axis_rot.x()), Ogre::Real(axis_rot.y()), Ogre::Real(axis_rot.z()));
		visual->setFramePosition( joint_name, position );
		visual->setFrameOrientation( joint_name, joint_orientation );
                visual->setFrameEnabled( joint_name, joint_info->getEnabled() );
	    }
	}


	// Now set or update the contents of the chosen visual.
        float scale = scale_property_->getFloat();
        float width = width_property_->getFloat();
        visual->setWidth( width );
        visual->setScale( scale );
	visual->setMessage( msg );

        visuals_.push_back(visual);
    }

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::EffortDisplay, rviz::Display )


