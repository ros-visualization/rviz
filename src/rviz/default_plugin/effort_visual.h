#ifndef EFFORT_VISUAL_H
#define EFFORT_VISUAL_H

#include <OgrePrerequisites.h>

namespace rviz
{
class Arrow;
class BillboardLine;
} // namespace rviz

namespace rviz
{
// Each instance of EffortVisual represents the visualization of a single
// sensor_msgs::Effort message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class EffortVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  EffortVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~EffortVisual();

  // set rainbow color
  void getRainbowColor(float value, Ogre::ColourValue& color);
  void setEffort(const std::string& joint_name, double effort, double max_effort);

  // set the pose of coordinates frame the each joint refers to.
  void setFramePosition(const std::string& joint_name, const Ogre::Vector3& position);
  void setFrameOrientation(const std::string& joint_name, const Ogre::Quaternion& orientation);

  void setFrameEnabled(const std::string& joint_name, const bool e);

  void setWidth(float w);

  void setScale(float s);

private:
  // The object implementing the effort circle
  std::map<std::string, rviz::BillboardLine*> effort_circle_;
  std::map<std::string, rviz::Arrow*> effort_arrow_;
  std::map<std::string, bool> effort_enabled_;

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* parent_node_;

  std::map<std::string, Ogre::Vector3> position_;
  std::map<std::string, Ogre::Quaternion> orientation_;

  float width_, scale_;
};

} // end namespace rviz

#endif // EFFORT_VISUAL_H
