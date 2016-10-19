#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include <urdf/model.h>
#include "effort_visual.h"

namespace rviz
{

    EffortVisual::EffortVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, boost::shared_ptr<urdf::Model> urdf_model )
    {
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the Effort's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

        //
        urdf_model_ = urdf_model;

	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_model_->joints_.begin(); it != urdf_model_->joints_.end(); it ++ ) {
	    if ( it->second->type == urdf::Joint::REVOLUTE ) {
                std::string joint_name = it->first;
                effort_enabled_[joint_name] = true;
            }
        }
    }

    EffortVisual::~EffortVisual()
    {
	// Delete the arrow to make it disappear.
	for (std::map<std::string, rviz::BillboardLine*>::iterator it = effort_circle_.begin(); it != effort_circle_.end(); it ++ ) {
            delete (it->second);
        }
	for (std::map<std::string, rviz::Arrow*>::iterator it = effort_arrow_.begin(); it != effort_arrow_.end(); it ++ ) {
            delete (it->second);
        }

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode( frame_node_ );
    }


    void EffortVisual::getRainbowColor(float value, Ogre::ColourValue& color)
    {
	value = std::min(value, 1.0f);
	value = std::max(value, 0.0f);

	float h = value * 5.0f + 1.0f;
	int i = floor(h);
	float f = h - i;
	if ( !(i&1) ) f = 1 - f; // if i is even
	float n = 1 - f;

	if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
	else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
	else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
	else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
	else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
    }

    void EffortVisual::setMessage( const sensor_msgs::JointStateConstPtr& msg )
    {
	// for all joints...
	int joint_num = msg->name.size();
	for (int i = 0; i < joint_num; i++ )
	{
	    std::string joint_name = msg->name[i];
	    double effort = msg->effort[i];
	    const urdf::Joint* joint = urdf_model_->getJoint(joint_name).get();
	    int joint_type = joint->type;
	    if ( joint_type == urdf::Joint::REVOLUTE )
	    {
                // enable or disable draw
                if ( effort_circle_.find(joint_name) != effort_circle_.end() &&
                     !effort_enabled_[joint_name] ) // enable->disable
                    {
                        delete(effort_circle_[joint_name]);
                        delete(effort_arrow_[joint_name]);
                        effort_circle_.erase(joint_name);
                        effort_arrow_.erase(joint_name);
                    }
                if ( effort_circle_.find(joint_name) == effort_circle_.end() &&
                     effort_enabled_[joint_name] ) // disable -> enable
                    {
                        effort_circle_[joint_name] = new rviz::BillboardLine( scene_manager_, frame_node_ );
                        effort_arrow_[joint_name] = new rviz::Arrow( scene_manager_, frame_node_ );
                    }

                if ( ! effort_enabled_[joint_name] ) continue;

		//tf::Transform offset = poseFromJoint(joint);
		boost::shared_ptr<urdf::JointLimits> limit = joint->limits;
		double max_effort = limit->effort, effort_value = 0.05;

		if ( max_effort != 0.0 )
		{
		    effort_value = std::min(fabs(effort) / max_effort, 1.0) + 0.05;
		} else {
                    effort_value = fabs(effort) + 0.05;
                }

                effort_arrow_[joint_name]->set(0, width_*2, width_*2*1.0, width_*2*2.0);
                if ( effort > 0 ) {
                    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(-1,0,0));
                } else {
                    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3( 1,0,0));
                }
                effort_arrow_[joint_name]->setPosition(orientation_[joint_name] * Ogre::Vector3(0, 0.05+effort_value*scale_*0.5, 0) + position_[joint_name]);
                effort_circle_[joint_name]->clear();
                effort_circle_[joint_name]->setLineWidth(width_);
                for (int i = 0; i < 30; i++) {
                    Ogre::Vector3 point = Ogre::Vector3((0.05+effort_value*scale_*0.5)*sin(i*2*M_PI/32), (0.05+effort_value*scale_*0.5)*cos(i*2*M_PI/32), 0);
                    if ( effort < 0 ) point.x = -point.x;
                    effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);
                }
                Ogre::ColourValue color;
                getRainbowColor(effort_value, color);
                effort_arrow_[joint_name]->setColor(color.r, color.g, color.b, color.a);
                effort_circle_[joint_name]->setColor(color.r, color.g, color.b, color.a);
            }
        }
    }

    void EffortVisual::setFrameEnabled( const std::string joint_name, const bool e )
    {
        effort_enabled_[joint_name] = e;
    }

    // Position and orientation are passed through to the SceneNode.
    void EffortVisual::setFramePosition( const Ogre::Vector3& position )
    {
	frame_node_->setPosition( position );
    }

    void EffortVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
	frame_node_->setOrientation( orientation );
    }
    // Position and orientation are passed through to the SceneNode.
    void EffortVisual::setFramePosition( const std::string joint_name, const Ogre::Vector3& position )
    {
	position_[joint_name] = position;
    }

    void EffortVisual::setFrameOrientation( const std::string joint_name, const Ogre::Quaternion& orientation )
    {
	orientation_[joint_name] = orientation;
    }

    void EffortVisual::setWidth( float w )
    {
        width_ = w;
    }

    void EffortVisual::setScale( float s )
    {
        scale_ = s;
    }

} // end namespace rviz

