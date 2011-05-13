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

#ifndef RVIZ_INTERACTIVE_MARKER_TOOLS_H
#define RVIZ_INTERACTIVE_MARKER_TOOLS_H

#include <visualization_msgs/InteractiveMarker.h>

namespace rviz
{

/// @brief fill in default values & insert default controls when none are specified
/// @param msg      interactive marker to be completed
void autoComplete( visualization_msgs::InteractiveMarker &msg );

/// @brief fill in default values & insert default controls when none are specified
/// @param msg      interactive marker which contains the control
/// @param control  the control to be completed
void autoComplete( const visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control );

/// make a quaternion with a fixed local x axis.
/// The rotation around that axis will be chosen automatically.
/// @param x,y,z    the designated x axis
geometry_msgs::Quaternion makeQuaternion( float x, float y, float z );


/// --- marker helpers ---


/// @brief make a default-style arrow marker based on the properties of the given interactive marker
/// @param msg      the interactive marker that this will go into
/// @param control  the control where to insert the arrow marker
/// @param pos      how far from the center should the arrow be, and on which side
void makeArrow( const visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control, float pos );

/// @brief make a default-style disc marker (e.g for rotating) based on the properties of the given interactive marker
/// @param msg      the interactive marker that this will go into
/// @param width    width of the disc, relative to its inner radius
void makeDisc( const visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control, float width = 0.3 );

/// assign an RGB value to the given marker based on the given orientation
void assignDefaultColor(visualization_msgs::Marker &marker, const geometry_msgs::Quaternion &quat );

}

#endif
