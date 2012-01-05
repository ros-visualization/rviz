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

#include "rviz/properties/ros_topic_dialog.h"
#include "rviz/properties/ros_topic_tree.h"

#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QPushButton>

namespace rviz
{

RosTopicDialog::RosTopicDialog( const std::string& message_type, std::string* topic_name_output, QWidget* parent )
  : QDialog( parent )
  , message_type_( message_type )
  , topic_name_output_( topic_name_output )
{
  topic_tree_ = new RosTopicTree( message_type_, this );

  button_box_ = new QDialogButtonBox( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this );
  connect( button_box_, SIGNAL( accepted() ), this, SLOT( accept() ));
  connect( button_box_, SIGNAL( rejected() ), this, SLOT( reject() ));
  button_box_->button( QDialogButtonBox::Ok )->setEnabled( false );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( topic_tree_ );
  layout->addWidget( button_box_ );

  setLayout( layout );

  connect( topic_tree_, SIGNAL( selectedTopicChanged( std::string )),
           this, SLOT( onTopicChanged( std::string )));

  connect( topic_tree_, SIGNAL( topicActivated( std::string )),
           this, SLOT( onTopicActivated( std::string )));
}

void RosTopicDialog::onTopicChanged( std::string new_topic )
{
  topic_name_ = new_topic;
  bool selection_is_valid = (new_topic != "");
  button_box_->button( QDialogButtonBox::Ok )->setEnabled( selection_is_valid );
}

void RosTopicDialog::onTopicActivated( std::string new_topic )
{
  if( new_topic != "" )
  {
    topic_name_ = new_topic;
    Q_EMIT accept();
  }
}

void RosTopicDialog::accept()
{
  *topic_name_output_ = topic_name_;
  QDialog::accept();
}

}; // end namespace rviz
