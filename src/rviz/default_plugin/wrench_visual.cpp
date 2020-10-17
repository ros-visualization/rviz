#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include "wrench_visual.h"

namespace rviz
{
WrenchVisual::WrenchVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
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
  force_node_ = frame_node_->createChildSceneNode();
  torque_node_ = frame_node_->createChildSceneNode();
  hide_small_values_ = true;

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  arrow_force_ = new rviz::Arrow(scene_manager_, force_node_);
  arrow_torque_ = new rviz::Arrow(scene_manager_, torque_node_);
  circle_torque_ = new rviz::BillboardLine(scene_manager_, torque_node_);
  circle_arrow_torque_ = new rviz::Arrow(scene_manager_, torque_node_);
}

WrenchVisual::~WrenchVisual()
{
  // Delete the arrow to make it disappear.
  delete arrow_force_;
  delete arrow_torque_;
  delete circle_torque_;
  delete circle_arrow_torque_;

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}


void WrenchVisual::setWrench(const geometry_msgs::Wrench& wrench)
{
  Ogre::Vector3 force(wrench.force.x, wrench.force.y, wrench.force.z);
  Ogre::Vector3 torque(wrench.torque.x, wrench.torque.y, wrench.torque.z);
  setWrench(force, torque);
}

void WrenchVisual::setWrench(const Ogre::Vector3& force, const Ogre::Vector3& torque)
{
  double force_length = force.length() * force_scale_;
  double torque_length = torque.length() * torque_scale_;
  // hide markers if they get too short and hide_small_values_ is activated
  // "too short" is defined as "force_length > width_"
  bool show_force = (force_length > width_) || !hide_small_values_;
  bool show_torque = (torque_length > width_) || !hide_small_values_;

  if (show_force)
  {
    arrow_force_->setScale(Ogre::Vector3(force_length, width_, width_));
    arrow_force_->setDirection(force);
  }
  force_node_->setVisible(show_force);

  if (show_torque)
  {
    arrow_torque_->setScale(Ogre::Vector3(torque_length, width_, width_));
    arrow_torque_->setDirection(torque);
    Ogre::Vector3 axis_z(0, 0, 1);
    Ogre::Quaternion orientation = axis_z.getRotationTo(torque);
    if (std::isnan(orientation.x) || std::isnan(orientation.y) || std::isnan(orientation.z))
      orientation = Ogre::Quaternion::IDENTITY;
    // circle_arrow_torque_->setScale(Ogre::Vector3(width_, width_, 0.05));
    circle_arrow_torque_->set(0, width_ * 0.1, width_ * 0.1 * 1.0, width_ * 0.1 * 2.0);
    circle_arrow_torque_->setDirection(orientation * Ogre::Vector3(0, 1, 0));
    circle_arrow_torque_->setPosition(orientation *
                                      Ogre::Vector3(torque_length / 4, 0, torque_length / 2));
    circle_torque_->clear();
    circle_torque_->setLineWidth(width_ * 0.05);
    for (int i = 4; i <= 32; i++)
    {
      Ogre::Vector3 point =
          Ogre::Vector3((torque_length / 4) * cos(i * 2 * M_PI / 32),
                        (torque_length / 4) * sin(i * 2 * M_PI / 32), torque_length / 2);
      circle_torque_->addPoint(orientation * point);
    }
  }
  torque_node_->setVisible(show_torque);
}

// Position and orientation are passed through to the SceneNode.
void WrenchVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void WrenchVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the rviz object.
void WrenchVisual::setForceColor(float r, float g, float b, float a)
{
  arrow_force_->setColor(r, g, b, a);
}
// Color is passed through to the rviz object.
void WrenchVisual::setTorqueColor(float r, float g, float b, float a)
{
  arrow_torque_->setColor(r, g, b, a);
  circle_torque_->setColor(r, g, b, a);
  circle_arrow_torque_->setColor(r, g, b, a);
}

void WrenchVisual::setForceScale(float s)
{
  force_scale_ = s;
}

void WrenchVisual::setTorqueScale(float s)
{
  torque_scale_ = s;
}

void WrenchVisual::setWidth(float w)
{
  width_ = w;
}

void WrenchVisual::setHideSmallValues(bool h)
{
  hide_small_values_ = h;
}


void WrenchVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
}

} // end namespace rviz
