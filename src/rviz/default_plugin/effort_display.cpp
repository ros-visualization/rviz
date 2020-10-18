#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <QTimer>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/helpers/tf_prefix.h>

#include "effort_visual.h"
#include "effort_display.h"

#include <urdf/model.h>

namespace rviz
{
JointInfo::JointInfo(const std::string& name, rviz::Property* parent_category)
{
  name_ = name;
  effort_ = 0;
  max_effort_ = 0;

  // info->category_ = new Property( QString::fromStdString( info->name_ ) , QVariant(), "",
  // joints_category_);
  category_ = new rviz::Property(QString::fromStdString(name_), true, "", parent_category,
                                 SLOT(updateVisibility()), this);

  effort_property_ = new rviz::FloatProperty("Effort", 0, "Effort value of this joint.", category_);
  effort_property_->setReadOnly(true);

  max_effort_property_ =
      new rviz::FloatProperty("Max Effort", 0, "Max Effort value of this joint.", category_);
  max_effort_property_->setReadOnly(true);
}

JointInfo::~JointInfo()
{
}

void JointInfo::updateVisibility()
{
}

void JointInfo::setEffort(double e)
{
  effort_property_->setFloat(e);
  effort_ = e;
}
void JointInfo::setMaxEffort(double m)
{
  max_effort_property_->setFloat(m);
  max_effort_ = m;
}

bool JointInfo::getEnabled() const
{
  return category_->getValue().toBool();
}

JointInfo* EffortDisplay::getJointInfo(const std::string& joint)
{
  M_JointInfo::iterator it = joints_.find(joint);
  if (it == joints_.end())
  {
    return nullptr;
  }

  return it->second;
}

JointInfo* EffortDisplay::createJoint(const std::string& joint)
{
  JointInfo* info = new JointInfo(joint, joints_category_);
  joints_.insert(std::make_pair(joint, info));
  return info;
}

EffortDisplay::EffortDisplay()
{
  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateColorAndAlpha()));

  width_property_ = new rviz::FloatProperty("Width", 0.02, "Width to drow effort circle", this,
                                            SLOT(updateColorAndAlpha()));

  scale_property_ = new rviz::FloatProperty("Scale", 1.0, "Scale to drow effort circle", this,
                                            SLOT(updateColorAndAlpha()));

  history_length_property_ =
      new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.", this,
                            SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);

  robot_description_property_ =
      new rviz::StringProperty("Robot Description", "robot_description",
                               "Name of the parameter to search for to load the robot "
                               "description.",
                               this, SLOT(updateRobotDescription()));

  tf_prefix_property_ = new StringProperty(
      "TF Prefix", "",
      "Robot Model normally assumes the link name is the same as the tf frame name. "
      "This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
      this, SLOT(updateTfPrefix()));

  joints_category_ = new rviz::Property("Joints", QVariant(), "", this);
}

void EffortDisplay::onInitialize()
{
  MFDClass::onInitialize();
  // skip tf_filter_ (resetting it)
  delete tf_filter_;
  tf_filter_ = new tf2_ros::MessageFilter<sensor_msgs::JointState>(*context_->getTF2BufferPtr(),
                                                                   std::string(), 1, update_nh_);

  // but directly process messages
  sub_.registerCallback(boost::bind(&EffortDisplay::incomingMessage, this, _1));
  updateHistoryLength();
}

EffortDisplay::~EffortDisplay()
{
}

// Clear the visuals by deleting their objects.
void EffortDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void EffortDisplay::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

void EffortDisplay::clear()
{
  clearStatuses();
  robot_description_.clear();
}

// Set the current color and alpha values for each visual.
void EffortDisplay::updateColorAndAlpha()
{
  float width = width_property_->getFloat();
  float scale = scale_property_->getFloat();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setWidth(width);
    visuals_[i]->setScale(scale);
  }
}

void EffortDisplay::updateRobotDescription()
{
  if (isEnabled())
  {
    load();
    context_->queueRender();
  }
}

// Set the number of past visuals to show.
void EffortDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

void EffortDisplay::load()
{
  // get robot_description
  std::string content;
  try
  {
    if (!update_nh_.getParam(robot_description_property_->getStdString(), content))
    {
      std::string loc;
      if (update_nh_.searchParam(robot_description_property_->getStdString(), loc))
        update_nh_.getParam(loc, content);
      else
      {
        clear();
        setStatus(StatusProperty::Error, "URDF",
                  QString("Parameter [%1] does not exist, and was not found by searchParam()")
                      .arg(robot_description_property_->getString()));
        // try again in a second
        QTimer::singleShot(1000, this, SLOT(updateRobotDescription()));
        return;
      }
    }
  }
  catch (const ros::InvalidNameException& e)
  {
    clear();
    setStatus(StatusProperty::Error, "URDF",
              QString("Invalid parameter name: %1.\n%2")
                  .arg(robot_description_property_->getString(), e.what()));
    return;
  }

  if (content.empty())
  {
    clear();
    setStatus(rviz::StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }

  if (content == robot_description_)
  {
    return;
  }

  robot_description_ = content;

  robot_model_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
  if (!robot_model_->initString(content))
  {
    ROS_ERROR("Unable to parse URDF description!");
    setStatus(rviz::StatusProperty::Error, "URDF", "Unable to parse robot model description!");
    return;
  }
  setStatus(rviz::StatusProperty::Ok, "URDF", "Robot model parsed Ok");
  for (std::map<std::string, urdf::JointSharedPtr>::iterator it = robot_model_->joints_.begin();
       it != robot_model_->joints_.end(); it++)
  {
    urdf::JointSharedPtr joint = it->second;
    if (joint->type == urdf::Joint::REVOLUTE)
    {
      std::string joint_name = it->first;
      urdf::JointLimitsSharedPtr limit = joint->limits;
      joints_[joint_name] = createJoint(joint_name);
      joints_[joint_name]->setMaxEffort(limit->effort);
    }
  }
}

void EffortDisplay::onEnable()
{
  load();
}

void EffortDisplay::onDisable()
{
  clear();
}

// This is our callback to handle an incoming message.
void EffortDisplay::processMessage(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Robot model might not be loaded already
  if (!robot_model_)
  {
    return;
  }
  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<EffortVisual> visual;
  if (visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new EffortVisual(context_->getSceneManager(), scene_node_));
  }
  visual->setWidth(width_property_->getFloat());
  visual->setScale(scale_property_->getFloat());

  V_string joints;
  size_t joint_num = msg->name.size();
  if (joint_num != msg->effort.size())
  {
    setStatus(rviz::StatusProperty::Error, "Topic",
              "Received a joint state msg with different joint names and efforts size!");
    return;
  }
  for (size_t i = 0; i < joint_num; ++i)
  {
    const std::string& joint_name = msg->name[i];
    JointInfo* joint_info = getJointInfo(joint_name);
    if (!joint_info)
      continue; // skip joints..

    // update effort property
    joint_info->setEffort(msg->effort[i]);
    joint_info->last_update_ = msg->header.stamp;

    const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
    int joint_type = joint->type;
    if (joint_type == urdf::Joint::REVOLUTE)
    {
      std::string tf_frame_id = concat(tf_prefix_property_->getStdString(), joint->child_link_name);
      Ogre::Quaternion orientation;
      Ogre::Vector3 position;

      // Call rviz::FrameManager to get the transform from the fixed frame to the joint's frame.
      if (!context_->getFrameManager()->getTransform(tf_frame_id, ros::Time(), position, orientation))
      {
        setStatus(rviz::StatusProperty::Error, QString::fromStdString(joint_name),
                  QString("Error transforming from frame '%1' to frame '%2'")
                      .arg(tf_frame_id.c_str(), qPrintable(fixed_frame_)));
        continue;
      };
      tf2::Vector3 axis_joint(joint->axis.x, joint->axis.y, joint->axis.z);
      tf2::Vector3 axis_z(0, 0, 1);
      tf2::Quaternion axis_rotation(axis_joint.cross(axis_z), axis_joint.angle(axis_z));
      if (std::isnan(axis_rotation.x()) || std::isnan(axis_rotation.y()) || std::isnan(axis_rotation.z()))
        axis_rotation = tf2::Quaternion::getIdentity();

      tf2::Quaternion axis_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
      tf2::Quaternion axis_rot = axis_orientation * axis_rotation;
      Ogre::Quaternion joint_orientation(Ogre::Real(axis_rot.w()), Ogre::Real(axis_rot.x()),
                                         Ogre::Real(axis_rot.y()), Ogre::Real(axis_rot.z()));
      visual->setFramePosition(joint_name, position);
      visual->setFrameOrientation(joint_name, joint_orientation);
      visual->setFrameEnabled(joint_name, joint_info->getEnabled());

      if (!validateFloats(joint_info->getEffort()))
      {
        setStatus(rviz::StatusProperty::Error, QString::fromStdString(joint_name),
                  QString("Invalid effort: %1").arg(joint_info->getEffort()));
        visual->setFrameEnabled(joint_name, false);
      }
      else
        setStatus(rviz::StatusProperty::Ok, QString::fromStdString(joint_name), QString());

      visual->setEffort(joint_name, joint_info->getEffort(), joint_info->getMaxEffort());
    }
  }
  visuals_.push_back(visual);
}

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::EffortDisplay, rviz::Display)
