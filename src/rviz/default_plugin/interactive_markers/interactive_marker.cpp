/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <boost/make_shared.hpp>

#include <QMenu>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSubEntity.h>
#include <OgreMath.h>
#include <OgreRenderWindow.h>

#include <ros/ros.h>
#include <interactive_markers/tools.h>
#include <tf2_msgs/TF2Error.h>

#include <rviz/frame_manager.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/render_panel.h>
#include <rviz/geometry.h>
#include <rviz/validate_quaternions.h>

#include <rviz/default_plugin/interactive_markers/integer_action.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>

namespace rviz
{
InteractiveMarker::InteractiveMarker(Ogre::SceneNode* scene_node, DisplayContext* context)
  : context_(context)
  , pose_changed_(false)
  , time_since_last_feedback_(0)
  , dragging_(false)
  , pose_update_requested_(false)
  , heart_beat_t_(0)
  , show_visual_aids_(false)
{
  reference_node_ = scene_node->createChildSceneNode();
  axes_ = new Axes(context->getSceneManager(), reference_node_, 1, 0.05);
}

InteractiveMarker::~InteractiveMarker()
{
  delete axes_;
  context_->getSceneManager()->destroySceneNode(reference_node_);
}

void InteractiveMarker::processMessage(const visualization_msgs::InteractiveMarkerPose& message)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  Ogre::Vector3 position(message.pose.position.x, message.pose.position.y, message.pose.position.z);
  Ogre::Quaternion orientation(message.pose.orientation.w, message.pose.orientation.x,
                               message.pose.orientation.y, message.pose.orientation.z);

  if (orientation.w == 0 && orientation.x == 0 && orientation.y == 0 && orientation.z == 0)
  {
    orientation.w = 1;
  }

  reference_time_ = message.header.stamp;
  reference_frame_ = message.header.frame_id;
  frame_locked_ = (message.header.stamp == ros::Time(0));

  requestPoseUpdate(position, orientation);
  context_->queueRender();
}

bool InteractiveMarker::processMessage(const visualization_msgs::InteractiveMarker& message)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  // copy values
  name_ = message.name;
  description_ = message.description;

  if (message.controls.empty())
  {
    Q_EMIT statusUpdate(StatusProperty::Ok, name_, "Marker empty.");
    return false;
  }

  scale_ = message.scale;

  reference_frame_ = message.header.frame_id;
  reference_time_ = message.header.stamp;
  frame_locked_ = (message.header.stamp == ros::Time(0));

  position_ = Ogre::Vector3(message.pose.position.x, message.pose.position.y, message.pose.position.z);

  normalizeQuaternion(message.pose.orientation, orientation_);

  pose_changed_ = false;
  time_since_last_feedback_ = 0;

  // setup axes
  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);
  axes_->set(scale_, scale_ * 0.05);

  has_menu_ = !message.menu_entries.empty();

  updateReferencePose();

  // Instead of just erasing all the old controls and making new ones
  // here, we want to preserve as much as possible from the old ones,
  // so that we don't lose the drag action in progress if a control is
  // being dragged when this update comes in.
  //
  // Controls are stored in a map from control name to control
  // pointer, so we loop over the incoming control messages looking
  // for names we already know about.  When we find them, we just call
  // the control's processMessage() function to update it.  When we
  // don't find them, we create a new Control.  We also keep track of
  // which control names we used to have but which are not present in
  // the incoming message, which we use to delete the unwanted
  // controls.

  // Make set of old-names called old-names-to-delete.
  std::set<std::string> old_names_to_delete;
  M_ControlPtr::const_iterator ci;
  for (ci = controls_.begin(); ci != controls_.end(); ci++)
  {
    old_names_to_delete.insert((*ci).first);
  }

  // Loop over new array:
  for (unsigned i = 0; i < message.controls.size(); i++)
  {
    const visualization_msgs::InteractiveMarkerControl& control_message = message.controls[i];
    M_ControlPtr::iterator search_iter = controls_.find(control_message.name);
    InteractiveMarkerControlPtr control;

    // If message->name in map,
    if (search_iter != controls_.end())
    {
      // Use existing control
      control = (*search_iter).second;
    }
    else
    {
      // Else make new control
      control = boost::make_shared<InteractiveMarkerControl>(context_, reference_node_, this);
      controls_[control_message.name] = control;
    }
    // Update the control with the message data
    control->processMessage(control_message);
    control->setShowVisualAids(show_visual_aids_);

    // Remove message->name from old-names-to-delete
    old_names_to_delete.erase(control_message.name);
  }

  // Loop over old-names-to-delete
  std::set<std::string>::iterator si;
  for (si = old_names_to_delete.begin(); si != old_names_to_delete.end(); si++)
  {
    // Remove Control object from map for name-to-delete
    controls_.erase(*si);
  }

  description_control_ = boost::make_shared<InteractiveMarkerControl>(context_, reference_node_, this);

  if (!message.description.empty())
    description_control_->processMessage(interactive_markers::makeTitle(message));

  // create menu
  menu_entries_.clear();
  menu_.reset();
  if (has_menu_)
  {
    menu_.reset(new QMenu());
    top_level_menu_ids_.clear();

    // Put all menu entries into the menu_entries_ map and create the
    // tree of menu entry ids.
    for (unsigned m = 0; m < message.menu_entries.size(); m++)
    {
      const visualization_msgs::MenuEntry& entry = message.menu_entries[m];
      MenuNode node;
      node.entry = entry;
      menu_entries_[entry.id] = node;
      if (entry.parent_id == 0)
      {
        top_level_menu_ids_.push_back(entry.id);
      }
      else
      {
        // Find the parent node and add this entry to the parent's list of children.
        std::map<uint32_t, MenuNode>::iterator parent_it = menu_entries_.find(entry.parent_id);
        if (parent_it == menu_entries_.end())
        {
          ROS_ERROR("interactive marker menu entry %u found before its parent id %u.  Ignoring.",
                    entry.id, entry.parent_id);
        }
        else
        {
          (*parent_it).second.child_ids.push_back(entry.id);
        }
      }
    }
    populateMenu(menu_.get(), top_level_menu_ids_);
  }

  if (frame_locked_)
  {
    std::ostringstream s;
    s << "Locked to frame " << reference_frame_;
    Q_EMIT statusUpdate(StatusProperty::Ok, name_, s.str());
  }
  else
  {
    Q_EMIT statusUpdate(StatusProperty::Ok, name_, "Position is fixed.");
  }
  return true;
}

// Recursively append menu and submenu entries to menu, based on a
// vector of menu entry id numbers describing the menu entries at the
// current level.
void InteractiveMarker::populateMenu(QMenu* menu, std::vector<uint32_t>& ids)
{
  for (size_t id_index = 0; id_index < ids.size(); id_index++)
  {
    uint32_t id = ids[id_index];
    std::map<uint32_t, MenuNode>::iterator node_it = menu_entries_.find(id);
    ROS_ASSERT_MSG(node_it != menu_entries_.end(),
                   "interactive marker menu entry %u not found during populateMenu().", id);
    MenuNode node = (*node_it).second;

    if (node.child_ids.empty())
    {
      IntegerAction* action =
          new IntegerAction(makeMenuString(node.entry.title), menu, (int)node.entry.id);
      connect(action, SIGNAL(triggered(int)), this, SLOT(handleMenuSelect(int)));
      menu->addAction(action);
    }
    else
    {
      // make sub-menu
      QMenu* sub_menu = menu->addMenu(makeMenuString(node.entry.title));
      populateMenu(sub_menu, node.child_ids);
    }
  }
}

QString InteractiveMarker::makeMenuString(const std::string& entry)
{
  QString menu_entry;
  if (entry.find("[x]") == 0)
  {
    menu_entry = QChar(0x2611) + QString::fromStdString(entry.substr(3));
  }
  else if (entry.find("[ ]") == 0)
  {
    menu_entry = QChar(0x2610) + QString::fromStdString(entry.substr(3));
  }
  else
  {
    menu_entry = QChar(0x3000) + QString::fromStdString(entry);
  }
  return menu_entry;
}


void InteractiveMarker::updateReferencePose()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  Ogre::Vector3 reference_position;
  Ogre::Quaternion reference_orientation;

  // if we're frame-locked, we need to find out what the most recent transformation time
  // actually is so we send back correct feedback
  if (frame_locked_)
  {
    std::string fixed_frame = context_->getFrameManager()->getFixedFrame();
    if (reference_frame_ == fixed_frame)
    {
      // if the two frames are identical, we don't need to do anything.
      // This should be ros::Time::now(), but then the computer running
      // RViz has to be time-synced with the server
      reference_time_ = ros::Time();
    }
    else
    {
      auto tf = context_->getFrameManager()->getTF2BufferPtr();
      tf2::CompactFrameID target_id = tf->_lookupFrameNumber(reference_frame_);
      tf2::CompactFrameID source_id = tf->_lookupFrameNumber(fixed_frame);
      std::string error;
      int retval = tf->_getLatestCommonTime(target_id, source_id, reference_time_, &error);

      if (retval != tf2_msgs::TF2Error::NO_ERROR)
      {
        std::ostringstream s;
        s << "Error getting time of latest transform between " << reference_frame_ << " and "
          << fixed_frame << ": " << error << " (error code: " << retval << ")";
        Q_EMIT statusUpdate(StatusProperty::Error, name_, s.str());
        reference_node_->setVisible(false);
        return;
      }
    }
  }

  if (!context_->getFrameManager()->getTransform(reference_frame_, ros::Time(), reference_position,
                                                 reference_orientation))
  {
    std::string error;
    context_->getFrameManager()->transformHasProblems(reference_frame_, ros::Time(), error);
    Q_EMIT statusUpdate(StatusProperty::Error, name_, error);
    reference_node_->setVisible(false);
    return;
  }

  reference_node_->setPosition(reference_position);
  reference_node_->setOrientation(reference_orientation);
  reference_node_->setVisible(true, false);

  context_->queueRender();
}


void InteractiveMarker::update(float wall_dt)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  time_since_last_feedback_ += wall_dt;
  if (frame_locked_)
  {
    updateReferencePose();
  }

  M_ControlPtr::iterator it;
  for (it = controls_.begin(); it != controls_.end(); it++)
  {
    (*it).second->update();
  }
  if (description_control_)
  {
    description_control_->update();
  }

  if (dragging_)
  {
    if (pose_changed_)
    {
      publishPose();
    }
    else if (time_since_last_feedback_ > 0.25)
    {
      // send keep-alive so we don't use control over the marker
      visualization_msgs::InteractiveMarkerFeedback feedback;
      feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE;
      publishFeedback(feedback);
    }
  }
}

void InteractiveMarker::publishPose()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  visualization_msgs::InteractiveMarkerFeedback feedback;
  feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
  feedback.control_name = last_control_name_;
  publishFeedback(feedback);
  pose_changed_ = false;
}

void InteractiveMarker::requestPoseUpdate(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if (dragging_)
  {
    pose_update_requested_ = true;
    requested_position_ = position;
    requested_orientation_ = orientation;
  }
  else
  {
    updateReferencePose();
    setPose(position, orientation, "");
  }
}

void InteractiveMarker::setPose(Ogre::Vector3 position,
                                Ogre::Quaternion orientation,
                                const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  position_ = position;
  orientation_ = orientation;
  pose_changed_ = true;
  last_control_name_ = control_name;

  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);

  M_ControlPtr::iterator it;
  for (it = controls_.begin(); it != controls_.end(); it++)
  {
    (*it).second->interactiveMarkerPoseChanged(position_, orientation_);
  }
  if (description_control_)
  {
    description_control_->interactiveMarkerPoseChanged(position_, orientation_);
  }
}

void InteractiveMarker::setShowDescription(bool show)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if (description_control_.get())
  {
    description_control_->setVisible(show);
  }
}

void InteractiveMarker::setShowAxes(bool show)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  axes_->getSceneNode()->setVisible(show);
}

void InteractiveMarker::setShowVisualAids(bool show)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  M_ControlPtr::iterator it;
  for (it = controls_.begin(); it != controls_.end(); it++)
  {
    (*it).second->setShowVisualAids(show);
  }
  show_visual_aids_ = show;
}

void InteractiveMarker::translate(Ogre::Vector3 delta_position, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  setPose(position_ + delta_position, orientation_, control_name);
}

void InteractiveMarker::rotate(Ogre::Quaternion delta_orientation, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  setPose(position_, delta_orientation * orientation_, control_name);
}

void InteractiveMarker::startDragging()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  dragging_ = true;
  pose_changed_ = false;
}

void InteractiveMarker::stopDragging()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  dragging_ = false;
  if (pose_update_requested_)
  {
    updateReferencePose();
    setPose(requested_position_, requested_orientation_, "");
    pose_update_requested_ = false;
  }
}

bool InteractiveMarker::handle3DCursorEvent(ViewportMouseEvent& event,
                                            const Ogre::Vector3& cursor_pos,
                                            const Ogre::Quaternion& /*cursor_rot*/,
                                            const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (event.acting_button == Qt::LeftButton)
  {
    Ogre::Vector3 point_rel_world = cursor_pos;
    bool got_3D_point = true;

    visualization_msgs::InteractiveMarkerFeedback feedback;
    feedback.control_name = control_name;
    feedback.marker_name = name_;

    // make sure we've published a last pose update
    feedback.event_type = ((uint8_t)visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
    publishFeedback(feedback, got_3D_point, point_rel_world);

    feedback.event_type = (event.type == QEvent::MouseButtonPress ?
                               (uint8_t)visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN :
                               (uint8_t)visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    publishFeedback(feedback, got_3D_point, point_rel_world);
  }

  if (!dragging_ && menu_.get())
  {
    // Event.right() will be false during a right-button-up event.  We
    // want to swallow (with the "return true") all other
    // right-button-related mouse events.
    if (event.right())
    {
      return true;
    }
    if (event.rightUp() && event.buttons_down == Qt::NoButton)
    {
      // Save the 3D mouse point to send with the menu feedback, if any.
      Ogre::Vector3 three_d_point = cursor_pos;
      bool valid_point = true;
      Ogre::Vector2 mouse_pos = project3DPointToViewportXY(event.viewport, cursor_pos);
      QCursor::setPos(event.panel->mapToGlobal(QPoint(mouse_pos.x, mouse_pos.y)));
      showMenu(event, control_name, three_d_point, valid_point);
      return true;
    }
  }

  return false;
}

bool InteractiveMarker::handleMouseEvent(ViewportMouseEvent& event, const std::string& control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (event.acting_button == Qt::LeftButton)
  {
    Ogre::Vector3 point_rel_world;
    bool got_3D_point =
        context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, point_rel_world);

    visualization_msgs::InteractiveMarkerFeedback feedback;
    feedback.control_name = control_name;
    feedback.marker_name = name_;

    // make sure we've published a last pose update
    feedback.event_type = ((uint8_t)visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
    publishFeedback(feedback, got_3D_point, point_rel_world);

    feedback.event_type = (event.type == QEvent::MouseButtonPress ?
                               (uint8_t)visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN :
                               (uint8_t)visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    publishFeedback(feedback, got_3D_point, point_rel_world);
  }

  if (!dragging_ && menu_.get())
  {
    // Event.right() will be false during a right-button-up event.  We
    // want to swallow (with the "return true") all other
    // right-button-related mouse events.
    if (event.right())
    {
      return true;
    }
    if (event.rightUp() && event.buttons_down == Qt::NoButton)
    {
      // Save the 3D mouse point to send with the menu feedback, if any.
      Ogre::Vector3 three_d_point;
      bool valid_point =
          context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, three_d_point);
      showMenu(event, control_name, three_d_point, valid_point);
      return true;
    }
  }

  return false;
}


void InteractiveMarker::showMenu(ViewportMouseEvent& event,
                                 const std::string& control_name,
                                 const Ogre::Vector3& three_d_point,
                                 bool valid_point)
{
  // Save the 3D mouse point to send with the menu feedback, if any.
  got_3d_point_for_menu_ = valid_point;
  three_d_point_for_menu_ = three_d_point;

  event.panel->showContextMenu(menu_);
  last_control_name_ = control_name;
}


void InteractiveMarker::handleMenuSelect(int menu_item_id)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  std::map<uint32_t, MenuNode>::iterator it = menu_entries_.find(menu_item_id);

  if (it != menu_entries_.end())
  {
    visualization_msgs::MenuEntry& entry = it->second.entry;

    std::string command = entry.command;
    uint8_t command_type = entry.command_type;

    if (command_type == visualization_msgs::MenuEntry::FEEDBACK)
    {
      visualization_msgs::InteractiveMarkerFeedback feedback;
      feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT;
      feedback.menu_entry_id = entry.id;
      feedback.control_name = last_control_name_;
      publishFeedback(feedback, got_3d_point_for_menu_, three_d_point_for_menu_);
    }
    else if (command_type == visualization_msgs::MenuEntry::ROSRUN)
    {
      std::string sys_cmd = "rosrun " + command;
      ROS_INFO_STREAM("Running system command: " << sys_cmd);
      sys_thread_ =
          boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&system, sys_cmd.c_str())));
      // system( sys_cmd.c_str() );
    }
    else if (command_type == visualization_msgs::MenuEntry::ROSLAUNCH)
    {
      std::string sys_cmd = "roslaunch " + command;
      ROS_INFO_STREAM("Running system command: " << sys_cmd);
      sys_thread_ =
          boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&system, sys_cmd.c_str())));
      // system( sys_cmd.c_str() );
    }
  }
}


void InteractiveMarker::publishFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback,
                                        bool mouse_point_valid,
                                        const Ogre::Vector3& mouse_point_rel_world)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  feedback.marker_name = name_;

  if (frame_locked_)
  {
    // frame-locked IMs will return their pose in the same coordinate frame
    // as they were set up with (the "reference frame").
    // The transformation's timestamp will be the one used for placing
    // the reference frame into the fixed frame
    feedback.header.frame_id = reference_frame_;
    feedback.header.stamp = reference_time_;
    feedback.pose.position.x = position_.x;
    feedback.pose.position.y = position_.y;
    feedback.pose.position.z = position_.z;
    feedback.pose.orientation.x = orientation_.x;
    feedback.pose.orientation.y = orientation_.y;
    feedback.pose.orientation.z = orientation_.z;
    feedback.pose.orientation.w = orientation_.w;

    feedback.mouse_point_valid = mouse_point_valid;
    if (mouse_point_valid)
    {
      Ogre::Vector3 mouse_rel_reference =
          reference_node_->convertWorldToLocalPosition(mouse_point_rel_world);
      feedback.mouse_point.x = mouse_rel_reference.x;
      feedback.mouse_point.y = mouse_rel_reference.y;
      feedback.mouse_point.z = mouse_rel_reference.z;
    }
  }
  else
  {
    // Timestamped IMs will return feedback in RViz's fixed frame
    feedback.header.frame_id = context_->getFixedFrame().toStdString();
    // This should be ros::Time::now(), but then the computer running
    // RViz has to be time-synced with the server
    feedback.header.stamp = ros::Time();

    Ogre::Vector3 world_position = reference_node_->convertLocalToWorldPosition(position_);
    Ogre::Quaternion world_orientation = reference_node_->convertLocalToWorldOrientation(orientation_);

    feedback.pose.position.x = world_position.x;
    feedback.pose.position.y = world_position.y;
    feedback.pose.position.z = world_position.z;
    feedback.pose.orientation.x = world_orientation.x;
    feedback.pose.orientation.y = world_orientation.y;
    feedback.pose.orientation.z = world_orientation.z;
    feedback.pose.orientation.w = world_orientation.w;

    feedback.mouse_point_valid = mouse_point_valid;
    feedback.mouse_point.x = mouse_point_rel_world.x;
    feedback.mouse_point.y = mouse_point_rel_world.y;
    feedback.mouse_point.z = mouse_point_rel_world.z;
  }

  Q_EMIT userFeedback(feedback);

  time_since_last_feedback_ = 0;
}

} // end namespace rviz
