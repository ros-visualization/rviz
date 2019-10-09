/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include "rviz/default_plugin/marker_display.h"
#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/marker_scale_property.h"
#include "rviz/properties/color_property.h"

#include "rviz/default_plugin/markers/marker_selection_handler.h"

namespace rviz
{

MarkerSelectionHandler::MarkerSelectionHandler( const MarkerBase* marker, MarkerID id, DisplayContext* context )
  : SelectionHandler( context )
  , marker_( marker )
  , marker_id_( QString::fromStdString( id.first ) + "/" + QString::number( id.second ))
{
}

MarkerSelectionHandler::~MarkerSelectionHandler()
{
}

Ogre::Vector3 MarkerSelectionHandler::getPosition()
{
  return Ogre::Vector3( marker_->getMessage()->pose.position.x,
                        marker_->getMessage()->pose.position.y,
                        marker_->getMessage()->pose.position.z );
}

Ogre::Quaternion MarkerSelectionHandler::getOrientation()
{
  return Ogre::Quaternion( marker_->getMessage()->pose.orientation.w,
                           marker_->getMessage()->pose.orientation.x,
                           marker_->getMessage()->pose.orientation.y,
                           marker_->getMessage()->pose.orientation.z );
}

Ogre::Vector3 MarkerSelectionHandler::getScale()
{
  return Ogre::Vector3( marker_->getMessage()->scale.x,
                        marker_->getMessage()->scale.y,
                        marker_->getMessage()->scale.z );
}

QColor MarkerSelectionHandler::getColor()
{
  return QColor( (int)(marker_->getMessage()->color.r * 255),
                 (int)(marker_->getMessage()->color.g * 255),
                 (int)(marker_->getMessage()->color.b * 255),
                 (int)(marker_->getMessage()->color.a * 255) );
}

QString MarkerSelectionHandler::getMarkerTypeName()
{
  QString marker_type_name;
  switch ( marker_->getMessage()->type ) {
  case visualization_msgs::Marker::ARROW:
    marker_type_name = "Arrow";
    break;
  case visualization_msgs::Marker::CUBE:
    marker_type_name = "Cube";
    break;
  case visualization_msgs::Marker::CUBE_LIST:
    marker_type_name = "Cube List";
    break;
  case visualization_msgs::Marker::TRIANGLE_LIST:
    marker_type_name = "Triangle List";
    break;
  case visualization_msgs::Marker::SPHERE:
    marker_type_name = "Sphere";
    break;
  case visualization_msgs::Marker::SPHERE_LIST:
    marker_type_name = "Sphere List";
    break;
  case visualization_msgs::Marker::CYLINDER:
    marker_type_name = "Cylinder";
    break;
  case visualization_msgs::Marker::LINE_STRIP:
    marker_type_name = "Line Strip";
    break;
  case visualization_msgs::Marker::LINE_LIST:
    marker_type_name = "Line List";
    break;
  case visualization_msgs::Marker::POINTS:
    marker_type_name = "Points";
    break;
  case visualization_msgs::Marker::TEXT_VIEW_FACING:
    marker_type_name = "Text View Facing";
    break;
  case visualization_msgs::Marker::MESH_RESOURCE:
    marker_type_name = "Mesh Resource";
    break;
  }
  return marker_type_name;
}

void MarkerSelectionHandler::createProperties( const Picked& obj, Property* parent_property )
{
  Property* group = new Property( "Marker " + marker_id_ + " (" + getMarkerTypeName() + ")", QVariant(), "", parent_property );
  properties_.push_back( group );

  position_property_ = new VectorProperty( "Position", getPosition(), "", group );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", getOrientation(), "", group );
  orientation_property_->setReadOnly( true );

  scale_property_ = new MarkerScaleProperty( "Scale", getScale(), marker_->getMessage()->type, "", group );
  scale_property_->setReadOnly( true );

  color_property_ = new ColorProperty( "Color", getColor(), "", group );
  color_property_->setReadOnly( true );

  group->expand();
}

void MarkerSelectionHandler::updateProperties()
{
  position_property_->setVector( getPosition() );
  orientation_property_->setQuaternion( getOrientation() );
  scale_property_->setScale( getScale(), marker_->getMessage()->type );
  color_property_->setColor( getColor() );
  properties_[0]->setName( "Marker " + marker_id_ + " (" + getMarkerTypeName() + ")" );
}

} // end namespace rviz
