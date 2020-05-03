#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include <utility>

#include "effort_visual.h"

namespace rviz
{
EffortVisual::EffortVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  : scene_manager_(scene_manager), parent_node_(parent_node)
{
}

EffortVisual::~EffortVisual()
{
  // Delete the arrow to make it disappear.
  for (auto& pair : effort_circle_)
    delete pair.second;

  for (auto& pair : effort_arrow_)
    delete pair.second;
}

void EffortVisual::getRainbowColor(float value, Ogre::ColourValue& color)
{
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  float n = 1 - f;

  if (i <= 1)
    color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2)
    color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3)
    color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4)
    color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5)
    color[0] = 1, color[1] = n, color[2] = 0;
}

void EffortVisual::setEffort(const std::string& joint_name, double effort, double max_effort)
{
  bool enabled = effort_enabled_.insert(std::make_pair(joint_name, true)).first->second;

  // enable or disable draw
  if (effort_circle_.find(joint_name) != effort_circle_.end() && !enabled) // enable->disable
  {
    delete (effort_circle_[joint_name]);
    delete (effort_arrow_[joint_name]);
    effort_circle_.erase(joint_name);
    effort_arrow_.erase(joint_name);
  }
  if (effort_circle_.find(joint_name) == effort_circle_.end() && enabled) // disable -> enable
  {
    effort_circle_[joint_name] = new rviz::BillboardLine(scene_manager_, parent_node_);
    effort_arrow_[joint_name] = new rviz::Arrow(scene_manager_, parent_node_);
  }

  if (!enabled)
    return;

  double effort_value;

  if (max_effort != 0.0)
  {
    effort_value = std::min(fabs(effort) / max_effort, 1.0) + 0.05;
  }
  else
  {
    effort_value = fabs(effort) + 0.05;
  }

  effort_arrow_[joint_name]->set(0, width_ * 2, width_ * 2 * 1.0, width_ * 2 * 2.0);
  if (effort > 0)
  {
    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(-1, 0, 0));
  }
  else
  {
    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(1, 0, 0));
  }
  effort_arrow_[joint_name]->setPosition(orientation_[joint_name] *
                                             Ogre::Vector3(0, 0.05 + effort_value * scale_ * 0.5, 0) +
                                         position_[joint_name]);
  effort_circle_[joint_name]->clear();
  effort_circle_[joint_name]->setLineWidth(width_);
  for (int i = 0; i < 30; i++)
  {
    Ogre::Vector3 point =
        Ogre::Vector3((0.05 + effort_value * scale_ * 0.5) * sin(i * 2 * M_PI / 32),
                      (0.05 + effort_value * scale_ * 0.5) * cos(i * 2 * M_PI / 32), 0);
    if (effort < 0)
      point.x = -point.x;
    effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);
  }
  Ogre::ColourValue color;
  getRainbowColor(effort_value, color);
  effort_arrow_[joint_name]->setColor(color.r, color.g, color.b, color.a);
  effort_circle_[joint_name]->setColor(color.r, color.g, color.b, color.a);
}

void EffortVisual::setFrameEnabled(const std::string& joint_name, const bool e)
{
  effort_enabled_[joint_name] = e;
}

// Position and orientation are passed through to the SceneNode.
void EffortVisual::setFramePosition(const std::string& joint_name, const Ogre::Vector3& position)
{
  position_[joint_name] = position;
}

void EffortVisual::setFrameOrientation(const std::string& joint_name, const Ogre::Quaternion& orientation)
{
  orientation_[joint_name] = orientation;
}

void EffortVisual::setWidth(float w)
{
  width_ = w;
}

void EffortVisual::setScale(float s)
{
  scale_ = s;
}

} // end namespace rviz
