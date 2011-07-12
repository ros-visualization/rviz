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

#include "interactive_marker.h"

#include "interactive_markers/tools.h"

#include "rviz/frame_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/default_plugin/interactive_marker_display.h"
#include "rviz/render_panel.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMath.h>

#include <boost/make_shared.hpp>
#include <wx/menu.h>

#include <ros/ros.h>

namespace rviz
{

InteractiveMarker::InteractiveMarker( InteractiveMarkerDisplay *owner, VisualizationManager *vis_manager, std::string topic_ns, std::string client_id ) :
  owner_(owner)
, vis_manager_(vis_manager)
, pose_changed_(false)
, time_since_last_feedback_(0)
, dragging_(false)
, pose_update_requested_(false)
, heart_beat_t_(0)
, topic_ns_(topic_ns)
, client_id_(client_id)
{
  ros::NodeHandle nh;
  std::string feedback_topic = topic_ns+"/feedback";
  feedback_pub_ = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>( feedback_topic, 100, false );

  reference_node_ = vis_manager->getSceneManager()->getRootSceneNode()->createChildSceneNode();

  axes_node_ = reference_node_->createChildSceneNode();
  axes_ = new ogre_tools::Axes( vis_manager->getSceneManager(), axes_node_, 1, 0.05 );
}

InteractiveMarker::~InteractiveMarker()
{
  delete axes_;
  vis_manager_->getSceneManager()->destroySceneNode( axes_node_ );
  vis_manager_->getSceneManager()->destroySceneNode( reference_node_ );
}

void InteractiveMarker::reset()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  controls_.clear();
  menu_entries_.clear();
}

void InteractiveMarker::processMessage( visualization_msgs::InteractiveMarkerPoseConstPtr message )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  Ogre::Vector3 position( message->pose.position.x, message->pose.position.y, message->pose.position.z );
  Ogre::Quaternion orientation( message->pose.orientation.w, message->pose.orientation.x,
      message->pose.orientation.y, message->pose.orientation.z );

  if ( orientation.w == 0 && orientation.x == 0 && orientation.y == 0 && orientation.z == 0 )
  {
    orientation.w = 1;
  }

  reference_time_ = message->header.stamp;
  reference_frame_ = message->header.frame_id;
  frame_locked_ = (message->header.stamp == ros::Time(0));

  requestPoseUpdate( position, orientation );
  vis_manager_->queueRender();
}

bool InteractiveMarker::processMessage( visualization_msgs::InteractiveMarkerConstPtr message )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  reset();

  visualization_msgs::InteractiveMarker auto_message = *message;
  interactive_markers::autoComplete( auto_message );

  // copy values

  name_ = auto_message.name;
  description_ = auto_message.description;

  if ( auto_message.controls.size() == 0 )
  {
    owner_->setStatus( status_levels::Ok, name_, "Marker empty.");
    return false;
  }

  scale_ = auto_message.scale;

  reference_frame_ = auto_message.header.frame_id;
  reference_time_ = auto_message.header.stamp;
  frame_locked_ = (auto_message.header.stamp == ros::Time(0));

  position_ = Ogre::Vector3(
      auto_message.pose.position.x,
      auto_message.pose.position.y,
      auto_message.pose.position.z );

  orientation_ = Ogre::Quaternion(
      auto_message.pose.orientation.w,
      auto_message.pose.orientation.x,
      auto_message.pose.orientation.y,
      auto_message.pose.orientation.z );

  pose_changed_ =false;
  time_since_last_feedback_ = 0;

  // setup axes
  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);
  axes_->set( scale_, scale_*0.05 );

  updateReferencePose();

  for ( unsigned i=0; i<auto_message.controls.size(); i++ )
  {
    controls_.push_back( boost::make_shared<InteractiveMarkerControl>(
        vis_manager_, auto_message.controls[i], reference_node_, this ) );
  }

  description_control_ = boost::make_shared<InteractiveMarkerControl>(
      vis_manager_, interactive_markers::makeTitle( auto_message ), reference_node_, this );
  controls_.push_back( description_control_ );


  //create menu
  if ( message->menu_entries.size() > 0 )
  {
    menu_.reset( new wxMenu() );
    menu_entries_.clear();
    top_level_menu_ids_.clear();

    // Put all menu entries into the menu_entries_ map and create the
    // tree of menu entry ids.
    for ( unsigned m=0; m < message->menu_entries.size(); m++ )
    {
      const visualization_msgs::MenuEntry& entry = message->menu_entries[ m ];
      MenuNode node;
      node.entry = entry;
      menu_entries_[ entry.id ] = node;
      if( entry.parent_id == 0 )
      {
        top_level_menu_ids_.push_back( entry.id );
      }
      else
      {
        // Find the parent node and add this entry to the parent's list of children.
        std::map< uint32_t, MenuNode >::iterator parent_it = menu_entries_.find( entry.parent_id );
        if( parent_it == menu_entries_.end() ) {
          ROS_ERROR("interactive marker menu entry %u found before its parent id %u.  Ignoring.", entry.id, entry.parent_id);
        }
        else
        {
          (*parent_it).second.child_ids.push_back( entry.id );
        }
      }
    }      

    populateMenu( menu_.get(), top_level_menu_ids_ );
  }

  owner_->setStatus( status_levels::Ok, name_, "OK");
  return true;
}

// Recursively append menu and submenu entries to menu, based on a
// vector of menu entry id numbers describing the menu entries at the
// current level.
void InteractiveMarker::populateMenu( wxMenu* menu, std::vector<uint32_t>& ids )
{
  for( size_t id_index = 0; id_index < ids.size(); id_index++ )
  {
    uint32_t id = ids[ id_index ];
    std::map< uint32_t, MenuNode >::iterator node_it = menu_entries_.find( id );
    ROS_ASSERT_MSG(node_it != menu_entries_.end(), "interactive marker menu entry %u not found during populateMenu().", id);
    MenuNode node = (*node_it).second;

    if ( node.child_ids.empty() )
    {
      // make regular entry.  MenuEntry id numbers are re-used as wx menu command identifiers.
      menu->Append( (int) node.entry.id, makeMenuString( node.entry.title ));
    }
    else
    {
      // make sub-menu
      wxMenu* sub_menu = new wxMenu();
      populateMenu( sub_menu, node.child_ids );
      menu->AppendSubMenu( sub_menu, makeMenuString( node.entry.title ));
    }
  }
  menu->Connect(wxEVT_COMMAND_MENU_SELECTED,
                (wxObjectEventFunction)&InteractiveMarker::handleMenuSelect, NULL, this);
}

wxString InteractiveMarker::makeMenuString( const std::string &entry )
{
  wxString menu_entry;
  if ( entry.find( "[x]" ) == 0 )
  {
    menu_entry = wxString::FromUTF8("\u2611") + wxString::FromAscii( entry.substr( 3 ).c_str() );
  }
  else if ( entry.find( "[ ]" ) == 0 )
  {
    menu_entry = wxString::FromUTF8("\u2610") + wxString::FromAscii( entry.substr( 3 ).c_str() );
  }
  else
  {
    menu_entry = wxString::FromUTF8("\u3000 ") + wxString::FromAscii( entry.c_str() );
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
  if ( frame_locked_ )
  {
    std::string fixed_frame = FrameManager::instance()->getFixedFrame();
    if ( reference_frame_ == fixed_frame )
    {
      // if the two frames are identical, we don't need to do anything.
      reference_time_ = ros::Time::now();
    }
    else
    {
      std::string error;
      int retval = FrameManager::instance()->getTFClient()->getLatestCommonTime(
          reference_frame_, fixed_frame, reference_time_, &error );
      if ( retval != tf::NO_ERROR )
      {
        std::ostringstream s;
        s <<"Error getting time of latest transform between " << reference_frame_
            << " and " << fixed_frame << ": " << error << " (error code: " << retval << ")";
        owner_->setStatus( status_levels::Error, name_, s.str() );
        reference_node_->setVisible( false );
        return;
      }
    }
  }

  if (!FrameManager::instance()->getTransform( reference_frame_, reference_time_,
      reference_position, reference_orientation ))
  {
    std::string error;
    FrameManager::instance()->transformHasProblems(reference_frame_, reference_time_, error);
    owner_->setStatus( status_levels::Error, name_, error);
    reference_node_->setVisible( false );
    return;
  }

  reference_node_->setPosition( reference_position );
  reference_node_->setOrientation( reference_orientation );
  reference_node_->setVisible( true, false );

  vis_manager_->queueRender();
}

void InteractiveMarker::update(float wall_dt)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  time_since_last_feedback_ += wall_dt;
  if ( frame_locked_ )
  {
    updateReferencePose();
  }
  if ( dragging_ )
  {
    if ( pose_changed_ )
    {
      publishPose();
    }
    else if ( time_since_last_feedback_ > 0.25 )
    {
      //send keep-alive so we don't use control over the marker
      visualization_msgs::InteractiveMarkerFeedback feedback;
      feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE;
      publishFeedback( feedback );
    }
  }
}

void InteractiveMarker::publishPose()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  visualization_msgs::InteractiveMarkerFeedback feedback;
  feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
  feedback.control_name = last_control_name_;
  publishFeedback( feedback );
  pose_changed_ = false;
}

void InteractiveMarker::requestPoseUpdate( Ogre::Vector3 position, Ogre::Quaternion orientation )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if ( dragging_ )
  {
    pose_update_requested_ = true;
    requested_position_ = position;
    requested_orientation_ = orientation;
  }
  else
  {
    updateReferencePose();
    setPose( position, orientation, "" );
  }
}

void InteractiveMarker::setPose( Ogre::Vector3 position, Ogre::Quaternion orientation, const std::string &control_name )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  position_ = position;
  orientation_ = orientation;
  pose_changed_ = true;
  last_control_name_ = control_name;

  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);

  std::list<InteractiveMarkerControlPtr>::iterator it;
  for ( it = controls_.begin(); it != controls_.end(); it++ )
  {
    (*it)->interactiveMarkerPoseChanged( position_, orientation_ );
  }
}

void InteractiveMarker::setShowDescription( bool show )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if ( description_control_.get() )
  {
    description_control_->setVisible( show );
  }
}

void InteractiveMarker::setShowAxes( bool show )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  axes_node_->setVisible( show );
}

void InteractiveMarker::translate( Ogre::Vector3 delta_position, const std::string &control_name )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  setPose( position_+delta_position, orientation_, control_name );
}

void InteractiveMarker::rotate( Ogre::Quaternion delta_orientation, const std::string &control_name )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  setPose( position_, delta_orientation * orientation_, control_name );
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
  if ( pose_changed_ )
  {
    publishPose();
  }
  dragging_ = false;
  if ( pose_update_requested_ )
  {
    updateReferencePose();
    setPose( requested_position_, requested_orientation_, "" );
    pose_update_requested_ = false;
  }
}

bool InteractiveMarker::handleMouseEvent(ViewportMouseEvent& event, const std::string &control_name)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (event.event.LeftDown() || event.event.LeftUp())
  {
    Ogre::Vector3 point_rel_world;
    bool got_3D_point =
      vis_manager_->getSelectionManager()->get3DPoint( event.viewport,
                                                       event.event.GetX(), event.event.GetY(),
                                                       point_rel_world );

    visualization_msgs::InteractiveMarkerFeedback feedback;
    feedback.event_type = (event.event.LeftDown() ?
                           visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN :
                           visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
                           
    feedback.control_name = control_name;
    feedback.marker_name = name_;
    publishFeedback( feedback, got_3D_point, point_rel_world );
  }

  if ( menu_.get() )
  {
    if ( event.event.RightDown() || event.event.RightIsDown() )
    {
      return true;
    }
    if ( event.event.RightUp() )
    {
      event.panel->setContextMenu( menu_ );
      wxContextMenuEvent context_event( wxEVT_CONTEXT_MENU, 0, event.event.GetPosition() );
      event.panel->AddPendingEvent( context_event );
      last_control_name_ = control_name;
      return true;
    }
  }

  return false;
}


void InteractiveMarker::handleMenuSelect(wxCommandEvent &evt)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  std::map< uint32_t, MenuNode >::iterator it = menu_entries_.find( (uint32_t)evt.GetId() );

  if ( it != menu_entries_.end() )
  {
    visualization_msgs::MenuEntry& entry = it->second.entry;

    std::string command = entry.command;
    uint8_t command_type = entry.command_type;

    if ( command_type == visualization_msgs::MenuEntry::FEEDBACK )
    {
      visualization_msgs::InteractiveMarkerFeedback feedback;
      feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT;
      feedback.menu_entry_id = entry.id;
      feedback.control_name = last_control_name_;
      publishFeedback( feedback );
    }
    else if ( command_type == visualization_msgs::MenuEntry::ROSRUN )
    {
      std::string sys_cmd = "rosrun " + command;
      ROS_INFO_STREAM( "Running system command: " << sys_cmd );
      sys_thread_ = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &system, sys_cmd.c_str() ) ) );
      //system( sys_cmd.c_str() );
    }
    else if ( command_type == visualization_msgs::MenuEntry::ROSLAUNCH )
    {
      std::string sys_cmd = "roslaunch " + command;
      ROS_INFO_STREAM( "Running system command: " << sys_cmd );
      sys_thread_ = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &system, sys_cmd.c_str() ) ) );
      //system( sys_cmd.c_str() );
    }
  }
}


void InteractiveMarker::publishFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback,
                                        bool mouse_point_valid,
                                        const Ogre::Vector3& mouse_point_rel_world )
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  feedback.client_id = client_id_;
  feedback.marker_name = name_;

  if ( frame_locked_ )
  {
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
    if( mouse_point_valid )
    {
      Ogre::Vector3 mouse_rel_reference = reference_node_->convertWorldToLocalPosition( mouse_point_rel_world );
      feedback.mouse_point.x = mouse_rel_reference.x;
      feedback.mouse_point.y = mouse_rel_reference.y;
      feedback.mouse_point.z = mouse_rel_reference.z;
    }
  }
  else
  {
    feedback.header.frame_id = vis_manager_->getFixedFrame();
    feedback.header.stamp = ros::Time::now();

    Ogre::Vector3 world_position = reference_node_->convertLocalToWorldPosition( position_ );
    Ogre::Quaternion world_orientation = reference_node_->convertLocalToWorldOrientation( orientation_ );

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

  feedback_pub_.publish( feedback );

  time_since_last_feedback_ = 0;
}

}
