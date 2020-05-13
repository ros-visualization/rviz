#ifndef SCREW_VISUAL_H
#define SCREW_VISUAL_H

#include <geometry_msgs/Vector3.h>
#include <OgrePrerequisites.h>

namespace rviz
{
class Arrow;
class BillboardLine;
} // namespace rviz

namespace rviz
{
// ScrewVisual visualizes a single screw, i.e. a wrench, twist, or acceleration
class RVIZ_EXPORT ScrewVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the scene, but in an unconfigured state.
  ScrewVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~ScrewVisual();

  // Configure the visual to show the given linear and angular vectors
  void setScrew(const Ogre::Vector3& linear, const Ogre::Vector3& angular);
  // Configure the visual to show the data in the message.
  void setScrew(const geometry_msgs::Vector3& linear, const geometry_msgs::Vector3& angular);

  // Set the pose of the coordinate frame the message refers to.
  // This could be done inside setMessage(), but that would require calls to FrameManager
  // and error handling inside setMessage(), which doesn't seem as clean.
  // This way ScrewVisual is only responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the message.
  void setLinearColor(float r, float g, float b, float a);
  void setAngularColor(float r, float g, float b, float a);
  void setLinearScale(float s);
  void setAngularScale(float s);
  void setWidth(float w);
  void setHideSmallValues(bool h);
  void setVisible(bool visible);

private:
  // The object implementing the circle
  rviz::Arrow* arrow_linear_;
  rviz::Arrow* arrow_angular_;
  rviz::BillboardLine* circle_angular_;
  rviz::Arrow* circle_arrow_angular_;
  float linear_scale_, angular_scale_, width_;
  bool hide_small_values_;

  // A SceneNode whose pose is set to match the coordinate frame of the message header.
  Ogre::SceneNode* frame_node_;
  // allow showing/hiding of linear / angular arrows
  Ogre::SceneNode* linear_node_;
  Ogre::SceneNode* angular_node_;

  // The SceneManager, kept here only so the destructor can ask it to destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

} // end namespace rviz

#endif // SCREW_VISUAL_H
