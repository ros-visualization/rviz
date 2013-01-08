/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 /*
 * measure_tool.cpp
 *
 *  Created on: Aug 8, 2012
 *      Author: gossow
 */

#include "measure_tool.h"

#include "rviz/ogre_helpers/shape.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"

#include <OgreSceneNode.h>

namespace rviz
{

MeasureTool::MeasureTool()
{
}

MeasureTool::~MeasureTool()
{
}

void MeasureTool::onInitialize()
{
  sphere_ = new Shape(Shape::Sphere,
      scene_manager_, 0 );
  sphere_->setColor( 1,1,0,1.0 );
  const float s = 0.1;
  sphere_->setScale( Ogre::Vector3(s) );
  sphere_->getRootNode()->setVisible(true);
}

void MeasureTool::activate()
{
  sphere_->getRootNode()->setVisible(true);
}

void MeasureTool::deactivate()
{
  sphere_->getRootNode()->setVisible(false);
}

int MeasureTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  Ogre::Vector3 pos;

  setStatus( "Click on two points to measure their distance." );

  if( event.leftUp() )
  {
    bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
    sphere_->getRootNode()->setVisible(success);

    if ( !success )
    {
      return flags;
    }

    sphere_->setPosition( pos );
    flags |= Render;
  }

  return flags;
}

} /* namespace rviz */


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::MeasureTool, rviz::Tool )
