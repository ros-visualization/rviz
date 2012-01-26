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

#include <QPushButton>

#include "rviz/properties/ros_topic_editor.h"
#include "rviz/properties/ros_topic_dialog.h"

namespace rviz
{

RosTopicEditor::RosTopicEditor( QWidget* parent )
  : LineEditWithButton( parent )
{
}

void RosTopicEditor::onButtonClick()
{
  Q_EMIT startPersistence();
  std::string result;
  // When creating the dialog here, it is crucial that we pass *this*
  // as the parent of the dialog.  Otherwise when the focus shifts
  // from this to the dialog, the QTreeWidget will destroy the editor
  // (*this*).  Then after dialog->exec() returns, *this* will have
  // been deleted and the program crashes.
  RosTopicDialog* dialog = new RosTopicDialog( topic_.datatype, &result, this );
  if( dialog->exec() == QDialog::Accepted )
  {
    topic_.name = result;
    QString old_text = text();
    setText( QString::fromStdString( topic_.name ));
    setModified( old_text != text() );
  }
  Q_EMIT endPersistence();
}

void RosTopicEditor::setTopic( ros::master::TopicInfo new_topic )
{
  topic_ = new_topic;
  setText( QString::fromStdString( topic_.name ));
}

ros::master::TopicInfo RosTopicEditor::getTopic()
{
  topic_.name = text().toStdString();
  return topic_;
}

} // end namespace rviz
