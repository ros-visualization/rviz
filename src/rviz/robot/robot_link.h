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

#include <string>
#include <map>

#include <QObject>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreAny.h>
#include <OGRE/OgreMaterial.h>

#include "rviz/ogre_helpers/object.h"
#include "rviz/selection/forwards.h"

namespace Ogre
{
class SceneManager;
class Entity;
class SubEntity;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class RibbonTrail;
}

namespace rviz
{
class Shape;
class Axes;
}

namespace urdf
{
class ModelInterface;
class Link;
typedef boost::shared_ptr<const Link> LinkConstPtr;
class Geometry;
typedef boost::shared_ptr<const Geometry> GeometryConstPtr;
class Pose;
}

namespace rviz
{

class DisplayContext;
class FloatProperty;
class Property;
class QuaternionProperty;
class Robot;
class RobotLinkSelectionHandler;
class VectorProperty;
typedef boost::shared_ptr<RobotLinkSelectionHandler> RobotLinkSelectionHandlerPtr;

/**
 * \struct RobotLink
 * \brief Contains any data we need from a link in the robot.
 */
class RobotLink: public QObject
{
Q_OBJECT
public:
  RobotLink( Robot* parent, DisplayContext* context, Property* parent_property );
  ~RobotLink();

  void load( const urdf::ModelInterface& descr, const urdf::LinkConstPtr& link, bool visual, bool collision );

  void setRobotAlpha(float a);

  void setTransforms(const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                     const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation);

  const std::string& getName() { return name_; }

  void setToErrorMaterial();
  void setToNormalMaterial();

  void setColor( float red, float green, float blue );
  void unsetColor();

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  bool isValid();

  /* If set to true, the link will only render to the depth channel
   * and be in render group 0, so it is rendered before anything else.
   * Thus, it will occlude other objects without being visible.
   */
  void setOnlyRenderDepth( bool onlyRenderDepth );

  bool getOnlyRenderDepth() { return only_render_depth_; }

public Q_SLOTS:
  /** @brief Update the visibility of the link elements: visual mesh, collision mesh, trail, and axes.
   *
   * Called by Robot when changing visual and collision visibilities,
   * since each link may be enabled or disabled. */
  void updateVisibility();

private Q_SLOTS:
  void updateAlpha();
  void updateTrail();
  void updateAxes();

private:
  void setRenderQueueGroup( Ogre::uint8 group );
  bool getEnabled() const;
  void createEntityForGeometryElement( const urdf::LinkConstPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, Ogre::SceneNode* parent_node, Ogre::Entity*& entity, Ogre::SceneNode*& scene_node, Ogre::SceneNode*& offset_node);

  void createVisual( const urdf::LinkConstPtr& link);
  void createCollision( const urdf::LinkConstPtr& link);
  void createSelection();
  Ogre::MaterialPtr getMaterialForLink( const urdf::LinkConstPtr& link );

  Robot* parent_;
  Ogre::SceneManager* scene_manager_;
  DisplayContext* context_;

  std::string name_;                          ///< Name of this link

  bool enabled_; ///< True if this link should be shown, false if not.

  typedef std::map<Ogre::SubEntity*, Ogre::MaterialPtr> M_SubEntityToMaterial;
  M_SubEntityToMaterial materials_;
  Ogre::MaterialPtr default_material_;
  std::string default_material_name_;

  Ogre::Entity* visual_mesh_;                 ///< The entity representing the visual mesh of this link (if it exists)
  Ogre::Entity* collision_mesh_;              ///< The entity representing the collision mesh of this link (if it exists)

  Ogre::SceneNode* visual_node_;              ///< The scene node the visual mesh is attached to
  Ogre::SceneNode* visual_offset_node_;
  Ogre::SceneNode* collision_node_;           ///< The scene node the collision mesh/primitive is attached to
  Ogre::SceneNode* collision_offset_node_;

  Ogre::RibbonTrail* trail_;

  Axes* axes_;

  float material_alpha_; ///< If material is not a texture, this saves the alpha value set in the URDF, otherwise is 1.0.
  float robot_alpha_; ///< Alpha value from top-level robot alpha Property (set via setRobotAlpha()).

  bool only_render_depth_;

  // joint stuff
  std::string joint_name_;

  RobotLinkSelectionHandlerPtr selection_handler_;

  Ogre::MaterialPtr color_material_;
  bool using_color_;

  // properties
  Property* link_property_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  Property* trail_property_;
  Property* axes_property_;
  FloatProperty* alpha_property_;

  friend class RobotLinkSelectionHandler;
};

} // namespace rviz

#endif // RVIZ_ROBOT_LINK_H
