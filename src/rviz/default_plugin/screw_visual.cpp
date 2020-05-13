#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include "screw_visual.h"

namespace rviz
{
ScrewVisual::ScrewVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the transform (position and orientation)
  // of itself relative to its parent.  Ogre does the math of combining those transforms for rendering.
  // Here we create a node to store the pose of the screw's header frame relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
  linear_node_ = frame_node_->createChildSceneNode();
  angular_node_ = frame_node_->createChildSceneNode();
  hide_small_values_ = true;

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  arrow_linear_ = new rviz::Arrow(scene_manager_, linear_node_);
  arrow_angular_ = new rviz::Arrow(scene_manager_, angular_node_);
  circle_angular_ = new rviz::BillboardLine(scene_manager_, angular_node_);
  circle_arrow_angular_ = new rviz::Arrow(scene_manager_, angular_node_);
}

ScrewVisual::~ScrewVisual()
{
  // Delete the arrow to make it disappear.
  delete arrow_linear_;
  delete arrow_angular_;
  delete circle_angular_;
  delete circle_arrow_angular_;

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}


void ScrewVisual::setScrew(const geometry_msgs::Vector3& linear, const geometry_msgs::Vector3& angular)
{
  setScrew(Ogre::Vector3(linear.x, linear.y, linear.z), Ogre::Vector3(angular.x, angular.y, angular.z));
}

void ScrewVisual::setScrew(const Ogre::Vector3& linear, const Ogre::Vector3& angular)
{
  double linear_length = linear.length() * linear_scale_;
  double angular_length = angular.length() * angular_scale_;
  // hide markers if they get too short and hide_small_values_ is activated
  // "too short" is defined as "linear_length > width_"
  bool show_linear = (linear_length > width_) || !hide_small_values_;
  bool show_angular = (angular_length > width_) || !hide_small_values_;

  if (show_linear)
  {
    arrow_linear_->setScale(Ogre::Vector3(linear_length, width_, width_));
    arrow_linear_->setDirection(linear);
  }
  linear_node_->setVisible(show_linear);

  if (show_angular)
  {
    arrow_angular_->setScale(Ogre::Vector3(angular_length, width_, width_));
    arrow_angular_->setDirection(angular);
    Ogre::Vector3 axis_z(0, 0, 1);
    Ogre::Quaternion orientation = axis_z.getRotationTo(angular);
    if (std::isnan(orientation.x) || std::isnan(orientation.y) || std::isnan(orientation.z))
      orientation = Ogre::Quaternion::IDENTITY;
    // circle_arrow_angular_->setScale(Ogre::Vector3(width_, width_, 0.05));
    circle_arrow_angular_->set(0, width_ * 0.1, width_ * 0.1 * 1.0, width_ * 0.1 * 2.0);
    circle_arrow_angular_->setDirection(orientation * Ogre::Vector3(0, 1, 0));
    circle_arrow_angular_->setPosition(orientation *
                                       Ogre::Vector3(angular_length / 4, 0, angular_length / 2));
    circle_angular_->clear();
    circle_angular_->setLineWidth(width_ * 0.05);
    for (int i = 4; i <= 32; i++)
    {
      Ogre::Vector3 point =
          Ogre::Vector3((angular_length / 4) * cos(i * 2 * M_PI / 32),
                        (angular_length / 4) * sin(i * 2 * M_PI / 32), angular_length / 2);
      circle_angular_->addPoint(orientation * point);
    }
  }
  angular_node_->setVisible(show_angular);
}

// Position and orientation are passed through to the SceneNode.
void ScrewVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ScrewVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the rviz object.
void ScrewVisual::setLinearColor(float r, float g, float b, float a)
{
  arrow_linear_->setColor(r, g, b, a);
}
// Color is passed through to the rviz object.
void ScrewVisual::setAngularColor(float r, float g, float b, float a)
{
  arrow_angular_->setColor(r, g, b, a);
  circle_angular_->setColor(r, g, b, a);
  circle_arrow_angular_->setColor(r, g, b, a);
}

void ScrewVisual::setLinearScale(float s)
{
  linear_scale_ = s;
}

void ScrewVisual::setAngularScale(float s)
{
  angular_scale_ = s;
}

void ScrewVisual::setWidth(float w)
{
  width_ = w;
}

void ScrewVisual::setHideSmallValues(bool h)
{
  hide_small_values_ = h;
}


void ScrewVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
}

} // end namespace rviz
