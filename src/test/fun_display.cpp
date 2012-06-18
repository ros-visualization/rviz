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

#include <stdlib.h> // for random
#include <stdio.h> // for printf

#include <QTimer>

#include "fun_display.h"

// static member
FunDisplay* FunDisplay::previous_fun_ = NULL;

FunDisplay::FunDisplay()
  : count_( 0 )
{
  size_ = new Property( "Size", 10, "How big, in units", this );
  connect( size_, SIGNAL( changed() ), this, SLOT( onSizeChanged() ));

  master_ = new Property( "Master", "chunky", "Type of butter.", this );
  connect( master_, SIGNAL( changed() ), this, SLOT( onMasterChanged() ));

  if( previous_fun_ )
  {
    connect( previous_fun_->master_, SIGNAL( changed() ), this, SLOT( onMasterChanged() ));
  }
  previous_fun_ = this;

  new Property( "Double", 1.0, "double", this );
  new Property( "Float", 1.0f, "float", this );

  slave_ = new Property( "Slave", "chunky", "Slave to the butter.", this );

  mood_ = new EnumProperty( "Mood", "sad", "Feelings, nothing more than feelings...", this );
  mood_->addOption( "sad", 0 );
  mood_->addOption( "happy", 1 );
  mood_->addOption( "angry", 2 );
  mood_->addOption( "jumpy", 3 );
  mood_->addOption( "squirmy", 4 );
  connect( mood_, SIGNAL( changed() ), this, SLOT( onMoodChanged() ));

  dance_ = new EditableEnumProperty( "Dance", "clown", "Stomple stomple", this );
  connect( dance_, SIGNAL( requestOptions( EditableEnumProperty* )), this, SLOT( makeDances( EditableEnumProperty* )));

  QTimer* timer = new QTimer( this );
  connect( timer, SIGNAL( timeout() ), this, SLOT( onTimerTick() ));
  timer->start( 1000 );
}

void FunDisplay::onTimerTick()
{
  count_++;
  slave_->setValue( count_ );
  if( (count_ / 10) % 2 )
  {
    setStatus( StatusProperty::Warn, "Slave", "Too odd." );
  }
  else
  {
    setStatus( StatusProperty::Ok, "Slave", "Even enough." );
  }
}

void FunDisplay::onMasterChanged()
{
  slave_->setValue( master_->getValue() );
}

void FunDisplay::onSizeChanged()
{
  if( size_->getValue().toInt() > 10 )
  {
    setStatus( StatusProperty::Error, "Size", "Too large." );
  }
  else if( size_->getValue().toInt() < 5 )
  {
    setStatus( StatusProperty::Warn, "Size", "Too small." );
  }
  else
  {
    setStatus( StatusProperty::Ok, "Size", "Just fine." );
  }
}

void FunDisplay::onMoodChanged()
{
  printf( "Mood is now %d.\n", mood_->getOptionInt() );
}

void FunDisplay::makeDances( EditableEnumProperty* prop )
{
  QStringList dances;
  dances.push_back( "Robot/Turtlebot" );
  dances.push_back( "Robot/PR2" );
  dances.push_back( "Macarena/Fast" );
  dances.push_back( "Macarena/Slow" );
  dances.push_back( "Twist" );
  dances.push_back( "Tango" );
  dances.push_back( "Swing/Lindy" );
  dances.push_back( "Swing/Hop/Small" );
  dances.push_back( "Swing/Hop/Big" );
  dances.push_back( "Stumble" );
  dances.push_back( "Stomple" );

  prop->clearOptions();
  for( int i = 0; i < 9; i++ )
  {
    int index = random() % dances.size();
    prop->addOption( dances[ index ]);
  }
}
