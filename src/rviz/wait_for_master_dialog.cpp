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

// Apparently OSX #defines 'check' to be an empty string somewhere.  
// That was fun to figure out.
#ifdef check
#undef check
#endif

#include <QTimer>

#include <ros/ros.h>

#include "rviz/wait_for_master_dialog.h"

namespace rviz
{

WaitForMasterDialog::WaitForMasterDialog( QWidget* parent )
  : QMessageBox( parent )
{
  setIcon( QMessageBox::Critical );

  const std::string& master_uri = ros::master::getURI();
  std::stringstream ss;
  ss << "Could not contact ROS master at [" << master_uri << "], retrying...";

  setText( QString::fromStdString( ss.str() ));
  setWindowTitle( "RViz: waiting for master" );
  setStandardButtons( QMessageBox::Cancel );

  QTimer* timer = new QTimer( this );
  connect( timer, SIGNAL( timeout() ), this, SLOT( onTimer() ));
  timer->start( 1000 );
}

void WaitForMasterDialog::onTimer()
{
  if( ros::master::check() )
  {
    accept();
  }
}

} // end namespace rviz
