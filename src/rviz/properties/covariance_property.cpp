/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "covariance_property.h"

#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"

#include <QColor>

namespace rviz
{

CovarianceProperty::CovarianceProperty( const QString& name,
                            bool default_value,
                            const QString& description,
                            Property* parent,
                            const char *changed_slot,
                            QObject* receiver )
  : BoolProperty( name, default_value, description, parent, changed_slot, receiver )
{
  position_color_property_ = new ColorProperty( "Position Color", QColor( 204, 51, 204 ),
                                             "Color to draw the covariance ellipse.",
                                             this, SIGNAL( childrenChanged() ) );
  
  position_alpha_property_ = new FloatProperty( "Position Alpha", 0.3f,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SIGNAL( childrenChanged() ) );
  position_alpha_property_->setMin( 0 );
  position_alpha_property_->setMax( 1 );
  
  position_scale_property_ = new FloatProperty( "Position Scale", 1.0f,
                                             "Scale factor to be applied to covariance ellipse",
                                             this, SIGNAL( childrenChanged() ) );

  orientation_color_property_ = new ColorProperty( "Orientation Color", QColor( 255, 255, 127 ),
                                             "Color to draw the covariance ellipse.",
                                             this, SIGNAL( childrenChanged() ) );
  
  orientation_alpha_property_ = new FloatProperty( "Orientation Alpha", 0.5f,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SIGNAL( childrenChanged() ) );
  orientation_alpha_property_->setMin( 0 );
  orientation_alpha_property_->setMax( 1 );
  
  orientation_scale_property_ = new FloatProperty( "Orientation Scale", 1.0f,
                                             "Scale factor to be applied to covariance ellipse",
                                             this, SIGNAL( childrenChanged() ) );

  setDisableChildrenIfFalse( true );
}

CovarianceProperty::~CovarianceProperty()
{
}

QColor CovarianceProperty::getPositionColor() const
{ 
  return position_color_property_->getColor();
}

Ogre::ColourValue CovarianceProperty::getPositionOgreColor()
{
  return position_color_property_->getOgreColor();
}

float CovarianceProperty::getPositionAlpha()
{
  position_alpha_property_->getFloat();
}

float CovarianceProperty::getPositionScale()
{
  position_scale_property_->getFloat();
}

QColor CovarianceProperty::getOrientationColor() const
{ 
  return orientation_color_property_->getColor();
}

Ogre::ColourValue CovarianceProperty::getOrientationOgreColor()
{
  return orientation_color_property_->getOgreColor();
}

float CovarianceProperty::getOrientationAlpha()
{
  orientation_alpha_property_->getFloat();
}

float CovarianceProperty::getOrientationScale()
{
  orientation_scale_property_->getFloat();
}

} // end namespace rviz
