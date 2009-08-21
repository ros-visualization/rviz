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

#ifndef RVIZ_ROBOT_LINK_H
#define RVIZ_ROBOT_LINK_H

#include "properties/forwards.h"
#include "selection/forwards.h"

#include <ogre_tools/object.h>

#include <string>
#include <map>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreAny.h>

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
class Shape;
class Axes;
}

namespace urdf
{
class Model;
class Link;
typedef boost::shared_ptr<const Link> LinkConstPtr;
class Geometry;
typedef boost::shared_ptr<const Geometry> GeometryConstPtr;
class Pose;
}

class TiXmlElement;

namespace rviz
{

class PropertyManager;
class Robot;
class VisualizationManager;
class RobotLinkSelectionHandler;
typedef boost::shared_ptr<RobotLinkSelectionHandler> RobotLinkSelectionHandlerPtr;

/**
 * \struct RobotLink
 * \brief Contains any data we need from a link in the robot.
 */
class RobotLink
{
public:
  RobotLink(Robot* parent, VisualizationManager* manager);
  ~RobotLink();

  void load(TiXmlElement* root_element, urdf::Model& descr, const urdf::LinkConstPtr& link, bool visual, bool collision);

  void setAlpha(float a);

  bool getShowTrail();
  void setShowTrail( bool show );

  bool getShowAxes();
  void setShowAxes( bool show );

  void setTransforms(const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                     const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation, bool applyOffsetTransforms);

  Ogre::Vector3 getPositionInRobotFrame();
  Ogre::Quaternion getOrientationInRobotFrame();

  const std::string& getName() { return name_; }

  void setToErrorMaterial();
  void setToNormalMaterial();

  void createProperties();

protected:

  void createEntityForGeometryElement(TiXmlElement* root_element, const urdf::LinkConstPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, Ogre::SceneNode* parent_node, Ogre::Entity*& entity, Ogre::SceneNode*& scene_node, Ogre::SceneNode*& offset_node);

  void createVisual(TiXmlElement* root_element, const urdf::LinkConstPtr& link);
  void createCollision(TiXmlElement* root_element, const urdf::LinkConstPtr& link);
  void createSelection(const urdf::Model& descr, const urdf::LinkConstPtr& link);


  Robot* parent_;
  Ogre::SceneManager* scene_manager_;
  PropertyManager* property_manager_;
  VisualizationManager* vis_manager_;

  std::string name_;                          ///< Name of this link
  std::string material_name_;                 ///< Name of the ogre material used by the meshes in this link

  Ogre::Entity* visual_mesh_;                 ///< The entity representing the visual mesh of this link (if it exists)
  Ogre::Entity* collision_mesh_;              ///< The entity representing the collision mesh of this link (if it exists)

  Ogre::SceneNode* visual_node_;              ///< The scene node the visual mesh is attached to
  Ogre::SceneNode* visual_offset_node_;
  Ogre::SceneNode* collision_node_;           ///< The scene node the collision mesh/primitive is attached to
  Ogre::SceneNode* collision_offset_node_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  Ogre::RibbonTrail* trail_;

  ogre_tools::Axes* axes_;

  // joint stuff
  std::string joint_name_;

  CollObjectHandle selection_object_;
  RobotLinkSelectionHandlerPtr selection_handler_;

  // properties
  Vector3PropertyWPtr position_property_;
  QuaternionPropertyWPtr orientation_property_;
  BoolPropertyWPtr trail_property_;
  BoolPropertyWPtr axes_property_;

  friend class RobotLinkSelectionHandler;
};

} // namespace rviz

#endif // RVIZ_ROBOT_LINK_H
