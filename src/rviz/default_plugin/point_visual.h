#ifndef POINT_VISUAL_H
#define POINT_VISUAL_H

#include <geometry_msgs/PointStamped.h>

#include <OgrePrerequisites.h>

namespace rviz
{
class Shape;
}

namespace rviz
{
// Each instance of PointStampedVisual represents the visualization of a single
// sensor_msgs::Point message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class PointStampedVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  PointStampedVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~PointStampedVisual();

  // set rainbow color
  void getRainbowColor(float value, Ogre::ColourValue& color);
  // Configure the visual to show the data in the message.
  void setMessage(const geometry_msgs::PointStamped::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way PointStampedVisual is only
  // responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Point message.
  void setColor(float r, float g, float b, float a);

  void setRadius(float r);

private:
  // The object implementing the point circle
  rviz::Shape* point_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Point message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  float radius_;
};

} // end namespace rviz

#endif // POINT_VISUAL_H
