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

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#endif

#include <urdf/model.h> // can be replaced later by urdf_model/types.h
#include <urdf_model/pose.h>

#include <rviz/ogre_helpers/object.h>
#include <rviz/selection/forwards.h>

#include <OgrePrerequisites.h>

namespace Ogre
{
class Any;
}

namespace rviz
{
class Shape;
class Axes;
class DisplayContext;
class FloatProperty;
class Property;
class BoolProperty;
class QuaternionProperty;
class Robot;
class RobotLinkSelectionHandler;
class VectorProperty;
class RobotJoint;
typedef boost::shared_ptr<RobotLinkSelectionHandler> RobotLinkSelectionHandlerPtr;


/**
 * \struct RobotLink
 * \brief Contains any data we need from a link in the robot.
 */
class RobotLink : public QObject
{
  Q_OBJECT
public:
  RobotLink(Robot* robot,
            const urdf::LinkConstSharedPtr& link,
            const std::string& parent_joint_name,
            bool visual,
            bool collision);
  ~RobotLink() override;

  virtual void setRobotAlpha(float a);

  virtual void setTransforms(const Ogre::Vector3& visual_position,
                             const Ogre::Quaternion& visual_orientation,
                             const Ogre::Vector3& collision_position,
                             const Ogre::Quaternion& collision_orientation);

  // access
  const std::string& getName() const
  {
    return name_;
  }
  const std::string& getParentJointName() const
  {
    return parent_joint_name_;
  }
  const std::vector<std::string>& getChildJointNames() const
  {
    return child_joint_names_;
  }
  Property* getLinkProperty() const
  {
    return link_property_;
  }
  Ogre::SceneNode* getVisualNode() const
  {
    return visual_node_;
  }
  Ogre::SceneNode* getCollisionNode() const
  {
    return collision_node_;
  }
  Robot* getRobot() const
  {
    return robot_;
  }

  // Remove link_property_ from its old parent and add to new_parent.  If new_parent==NULL then leav
  // unparented.
  void setParentProperty(Property* new_parent);

  // hide or show all sub properties (hide to make tree easier to see)
  virtual void hideSubProperties(bool hide);

  void setToErrorMaterial();
  void setToNormalMaterial();

  void setColor(float red, float green, float blue);
  void unsetColor();

  /// set whether the link is selectable.  If false objects behind/inside the link can be
  /// selected/manipulated.  Returns old value.
  bool setSelectable(bool selectable);
  bool getSelectable();

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  bool hasGeometry() const;

  /* If set to true, the link will only render to the depth channel
   * and be in render group 0, so it is rendered before anything else.
   * Thus, it will occlude other objects without being visible.
   */
  void setOnlyRenderDepth(bool onlyRenderDepth);
  bool getOnlyRenderDepth() const
  {
    return only_render_depth_;
  }

  // place subproperties as children of details_ or joint_property_
  void useDetailProperty(bool use_detail);

  // expand all sub properties
  void expandDetails(bool expand);

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
  void setRenderQueueGroup(Ogre::uint8 group);
  bool getEnabled() const;
  void createEntityForGeometryElement(const urdf::LinkConstSharedPtr& link,
                                      const urdf::Geometry& geom,
                                      const urdf::MaterialSharedPtr& material,
                                      const urdf::Pose& origin,
                                      Ogre::SceneNode* scene_node,
                                      Ogre::Entity*& entity);

  void createVisual(const urdf::LinkConstSharedPtr& link);
  void createCollision(const urdf::LinkConstSharedPtr& link);
  void createSelection();
  Ogre::MaterialPtr getMaterialForLink(const urdf::LinkConstSharedPtr& link,
                                       urdf::MaterialConstSharedPtr material);


protected:
  Robot* robot_;
  Ogre::SceneManager* scene_manager_;
  DisplayContext* context_;

  std::string name_; ///< Name of this link
  std::string parent_joint_name_;
  std::vector<std::string> child_joint_names_;


  // properties
  Property* link_property_;
  Property* details_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  Property* trail_property_;
  Property* axes_property_;
  FloatProperty* alpha_property_;

private:
  typedef std::map<Ogre::SubEntity*, Ogre::MaterialPtr> M_SubEntityToMaterial;
  M_SubEntityToMaterial materials_;
  Ogre::MaterialPtr default_material_;
  std::string default_material_name_;

  std::vector<Ogre::Entity*>
      visual_meshes_; ///< The entities representing the visual mesh of this link (if they exist)
  std::vector<Ogre::Entity*>
      collision_meshes_; ///< The entities representing the collision mesh of this link (if they exist)

  Ogre::SceneNode* visual_node_;    ///< The scene node the visual meshes are attached to
  Ogre::SceneNode* collision_node_; ///< The scene node the collision meshes are attached to

  Ogre::RibbonTrail* trail_;

  Axes* axes_;

  float material_alpha_; ///< If material is not a texture, this saves the alpha value set in the URDF,
                         /// otherwise is 1.0.
  float robot_alpha_;    ///< Alpha value from top-level robot alpha Property (set via setRobotAlpha()).

  bool only_render_depth_;
  bool is_selectable_;

  // joint stuff
  std::string joint_name_;

  RobotLinkSelectionHandlerPtr selection_handler_;

  Ogre::MaterialPtr color_material_;
  bool using_color_;

  friend class RobotLinkSelectionHandler;
};

} // namespace rviz

#endif // RVIZ_ROBOT_LINK_H
