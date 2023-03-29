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

#include <rviz/properties/line_edit_with_button.h>

#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QApplication>

namespace rviz
{
LineEditWithButton::LineEditWithButton(QWidget* parent) : QLineEdit(parent)
{
  button_ = new QPushButton(this);
  button_->setText("...");
  button_->setCursor(Qt::ArrowCursor);
  button_->setDefault(false);
  button_->setAutoDefault(false);
  button_->setFocusPolicy(Qt::NoFocus);

  connect(button_, &QPushButton::clicked, this, &LineEditWithButton::onButtonClick);
}

void LineEditWithButton::resizeEvent(QResizeEvent* event)
{
  // This widget does not use a QLayout object, because I want the
  // child button to be sized and positioned a specific way.  This
  // function lays out the child button based on the new size of the
  // LineEdit.

  int padding = 1;

  // Make sure the text area doesn't go under the button.
  setTextMargins(padding, padding, height(), padding);

  // Call the original resize handler.
  QLineEdit::resizeEvent(event);

  // Make the button square, matching the height of this widget minus
  // padding, and located all the way to the right.
  int button_width = height() - 2 * padding;
  int button_height = button_width;
  button_->setGeometry(width() - button_width - padding, padding, button_width, button_height);
}

} // end namespace rviz
