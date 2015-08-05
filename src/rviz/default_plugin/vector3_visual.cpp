#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include "vector3_visual.h"

#include "cmath"

namespace rviz
{

    Vector3StampedVisual::Vector3StampedVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    {
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the WrenchStamped's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	arrow_vector_ = new rviz::Arrow( scene_manager_, frame_node_ );
    }

    Vector3StampedVisual::~Vector3StampedVisual()
    {
	// Delete the arrow to make it disappear.
	delete arrow_vector_;

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode( frame_node_ );
    }


    void Vector3StampedVisual::setMessage( const geometry_msgs::Vector3Stamped::ConstPtr& msg )
    {
        Ogre::Vector3 vector(msg->vector.x, msg->vector.y, msg->vector.z);
        double vector_length = vector.length() * scale_;
        if (isnan(scale_))  //if you pass a NaN into setScale below, an Ogre assertion triggers and rviz crashes.
        {
            ROS_ERROR("scale_ is nan");
            vector_length = 0.0;
        }
    	arrow_vector_->setScale(Ogre::Vector3(vector_length, width_, width_)); 
        arrow_vector_->setDirection(vector);
    }

    // Position and orientation are passed through to the SceneNode.
    void Vector3StampedVisual::setFramePosition( const Ogre::Vector3& position )
    {
	frame_node_->setPosition( position );
    }

    void Vector3StampedVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
	frame_node_->setOrientation( orientation );
    }

    // Color is passed through to the rviz object.
    void Vector3StampedVisual::setVectorColor( float r, float g, float b, float a )
    {
	arrow_vector_->setColor( r, g, b, a );
    }

    void  Vector3StampedVisual::setScale( float s ) {
      scale_ = s;
    }
    void  Vector3StampedVisual::setWidth( float w ) {
      width_ = w;
    }

} // end namespace rviz

