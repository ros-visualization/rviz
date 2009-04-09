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

#ifndef RVIZ_ROBOT_H_
#define RVIZ_ROBOT_H_

#include "properties/forwards.h"

#include <ogre_tools/object.h>

#include <string>
#include <map>

#include <mechanism_model/robot.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreAny.h>

#include <robot_msgs/JointState.h>
#include <robot_msgs/MechanismState.h>

namespace Ogre
{
class SceneManager;
class Entity;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class RibbonTrail;
}

namespace ogre_tools
{
class Object;
class Axes;
}

namespace planning_models
{
class KinematicModel;
}

namespace tf
{
class TransformListener;
}

namespace rviz
{

class Robot;
class RobotLink;
class VisualizationManager;

/**
 * \class Robot
 *
 * A helper class to draw a representation of a robot, as specified by a URDF.  Can display either the visual models of the robot,
 * or the collision models.
 */
class Robot : public ogre_tools::Object
{
public:
  Robot( VisualizationManager* manager, const std::string& name = "" );
  ~Robot();

  void setPropertyManager( PropertyManager* property_manager, const CategoryPropertyWPtr& parent );

  /**
   * \brief Loads meshes/primitives from a robot description.  Calls clear() before loading.
   *
   * @param descr The robot description to read from
   * @param visual Whether or not to load the visual representation
   * @param collision Whether or not to load the collision representation
   */
  void load( mechanism::Robot &descr, bool visual = true, bool collision = true );

  /**
   * \brief Clears all data loaded from a URDF
   */
  void clear();

  /**
   * \brief Updates positions/orientations from a tf listener
   *
   * @param tf_client The rosTF client to load from
   * @param target_frame The frame to transform into
   */
  void update( tf::TransformListener* tf, const std::string& target_frame );

  /**
   * \brief Updates positions/orientations from a kinematic planning model
   *
   * @param kinematic_model The model to load from
   * @param target_frame The frame to transform into
   */
  void update( planning_models::KinematicModel* kinematic_model, const std::string& target_frame );

  void update( const robot_msgs::MechanismState& state );

  /**
   * \brief Set the robot as a whole to be visible or not
   * @param visible Should we be visible?
   */
  void setVisible( bool visible );

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible( bool visible );

  /**
   * \brief Set whether the collision meshes/primitives of the robot should be visible
   * @param visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible( bool visible );

  /**
   * \brief Returns whether or not the visual representation is set to be visible
   */
  bool isVisualVisible();
  /**
   * \brief Returns whether or not the collision representation is set to be visible
   */
  bool isCollisionVisible();

  void setAlpha(float a);
  float getAlpha() { return alpha_; }

  RobotLink* getLink( const std::string& name );

  const std::string& getName() { return name_; }

  Ogre::SceneNode* getVisualNode() { return root_visual_node_; }
  Ogre::SceneNode* getCollisionNode() { return root_collision_node_; }
  Ogre::SceneNode* getOtherNode() { return root_other_node_; }

  CategoryPropertyWPtr getLinksCategory() { return links_category_; }

  // Overrides from ogre_tools::Object
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setScale( const Ogre::Vector3& scale );
  virtual void setColor( float r, float g, float b, float a );
  virtual void setUserData( const Ogre::Any& user_data );
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

protected:

  typedef std::map< std::string, RobotLink* > M_NameToLink;
  M_NameToLink links_;                      ///< Map of name to link info, stores all loaded links.

  typedef std::map< std::string, std::string > M_string;
  M_string joint_to_link_;

  Ogre::SceneNode* root_visual_node_;           ///< Node all our visual nodes are children of
  Ogre::SceneNode* root_collision_node_;        ///< Node all our collision nodes are children of
  Ogre::SceneNode* root_other_node_;

  bool visual_visible_;                         ///< Should we show the visual representation?
  bool collision_visible_;                      ///< Should we show the collision representation?

  VisualizationManager* vis_manager_;
  PropertyManager* property_manager_;
  CategoryPropertyWPtr parent_property_;
  CategoryPropertyWPtr links_category_;

  Ogre::Any user_data_;

  std::string name_;
  float alpha_;
};

} // namespace rviz

#endif /* RVIZ_ROBOT_H_ */
