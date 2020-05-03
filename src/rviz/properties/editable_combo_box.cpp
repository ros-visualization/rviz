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

#include <stdio.h>

#include <QCompleter>
#include <QKeyEvent>
#include <QLineEdit>

#include <rviz/properties/editable_combo_box.h>

namespace rviz
{
EditableComboBox::EditableComboBox(QWidget* parent) : ComboBox(parent)
{
  setEditable(true);
  completer()->setCompletionMode(QCompleter::PopupCompletion);
  completer()->setCaseSensitivity(Qt::CaseInsensitive);
}

QString findMaxCommonPrefix(const QStringList& strings)
{
  if (strings.empty())
  {
    return "";
  }
  if (strings.size() == 1)
  {
    return strings[0];
  }
  QString common_prefix;
  int char_index = 0;

  // loop over character index
  while (true)
  {
    if (char_index >= strings[0].size())
    {
      return common_prefix;
    }
    const QChar c = strings[0][char_index];

    // loop over strings
    for (int string_index = 1; string_index < strings.size(); string_index++)
    {
      const QString& str = strings[string_index];
      if (char_index >= str.size() || str[char_index] != c)
      {
        return common_prefix;
      }
    }
    common_prefix += c;
    char_index++;
  }
  return ""; // just to satisfy compiler... I know it will never reach this.
}

bool EditableComboBox::event(QEvent* event)
{
  if (event->type() == QEvent::KeyPress)
  {
    QKeyEvent* k = (QKeyEvent*)event;
    if (k->key() == Qt::Key_Tab && k->modifiers() == Qt::NoModifier)
    {
      QCompleter* comp = completer();

      QStringList completions;
      for (int i = 0; comp->setCurrentRow(i); i++)
      {
        completions.push_back(comp->currentCompletion());
      }
      QString max_common_prefix = findMaxCommonPrefix(completions);
      if (max_common_prefix.size() > currentText().size())
      {
        setEditText(max_common_prefix);
        lineEdit()->setCursorPosition(max_common_prefix.size());
      }

      event->accept();
      return true;
    }
  }
  return ComboBox::event(event);
}

} // end namespace rviz
