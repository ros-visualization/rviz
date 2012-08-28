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
  sphere_->setColor( 1,1,1,0.3 );
  const float s = 0.1;
  sphere_->setScale( Ogre::Vector3(s) );
  sphere_->getRootNode()->setVisible(false);
}

void MeasureTool::activate()
{
  setStatus( "Click on two points to measure their distance." );
}

void MeasureTool::deactivate()
{
  sphere_->getRootNode()->setVisible(false);
}

int MeasureTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  Ogre::Vector3 pos;

  bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
  sphere_->getRootNode()->setVisible(success);

  if ( !success )
  {
    return flags;
  }

  sphere_->setPosition( pos );

  if( event.leftUp() )
  {
    //flags |= (Finished|Render);
  }

  return flags;
}

} /* namespace rviz */


#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz, Measure, rviz::MeasureTool, rviz::Tool )
