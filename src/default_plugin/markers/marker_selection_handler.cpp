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

#include "marker_selection_handler.h"
#include "marker_base.h"
#include "marker_display.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/bind.hpp>

namespace rviz
{

MarkerSelectionHandler::MarkerSelectionHandler(const MarkerBase* marker, MarkerID id)
: marker_(marker)
, id_(id)
{
}

MarkerSelectionHandler::~MarkerSelectionHandler()
{
}

Ogre::Vector3 MarkerSelectionHandler::getPosition()
{
  return Ogre::Vector3(marker_->getMessage()->pose.position.x, marker_->getMessage()->pose.position.y, marker_->getMessage()->pose.position.z);
}

Ogre::Quaternion MarkerSelectionHandler::getOrientation()
{
  return Ogre::Quaternion(marker_->getMessage()->pose.orientation.w, marker_->getMessage()->pose.orientation.x, marker_->getMessage()->pose.orientation.y, marker_->getMessage()->pose.orientation.z);
}

void MarkerSelectionHandler::createProperties(const Picked& obj, PropertyManager* property_manager)
{
  std::stringstream prefix;
  prefix << "Marker " << id_.first << "/" << id_.second;
  CategoryPropertyWPtr cat = property_manager->createCategory(prefix.str(), prefix.str());
  properties_.push_back(property_manager->createProperty<StringProperty>("ID", prefix.str(), boost::bind(&MarkerSelectionHandler::getId, this), StringProperty::Setter(), cat));
  properties_.push_back(property_manager->createProperty<Vector3Property>("Position", prefix.str(), boost::bind(&MarkerSelectionHandler::getPosition, this), Vector3Property::Setter(), cat));
  properties_.push_back(property_manager->createProperty<QuaternionProperty>("Orientation", prefix.str(), boost::bind(&MarkerSelectionHandler::getOrientation, this), QuaternionProperty::Setter(), cat));
}

}
