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

#include "rviz/properties/property_tree_model.h"

#include "rviz/properties/status_list.h"

namespace rviz
{
StatusList::StatusList(const QString& name, Property* parent) : StatusProperty("", "", Ok, parent)
{
  setName(name);
  setShouldBeSaved(false);
}

void StatusList::setName(const QString& name)
{
  name_prefix_ = name;
  updateLabel();
}

void StatusList::setStatus(Level level, const QString& name, const QString& text)
{
  QHash<QString, StatusProperty*>::iterator child_iter = status_children_.find(name);
  StatusProperty* child;
  if (child_iter == status_children_.end())
  {
    child = new StatusProperty(name, text, level, this);
    status_children_.insert(name, child);
  }
  else
  {
    child = child_iter.value();
    child->setLevel(level);
    child->setValue(text);
  }
  if (level > level_)
  {
    setLevel(level);
  }
  else if (level < level_)
  {
    updateLevel();
  }
}

void StatusList::deleteStatus(const QString& name)
{
  StatusProperty* child = status_children_.take(name);
  if (child)
  {
    delete child;
    updateLevel();
  }
}

void StatusList::clear()
{
  int num_rows = numChildren();
  if (num_rows > 0)
  {
    QList<StatusProperty*> to_be_deleted = status_children_.values();

    status_children_.clear();

    // It is important here to clear the status_children_ list before
    // deleting its contents.  On Macs the deletion can indirectly
    // trigger a call to setStatus(), and status_children_ should not
    // contain any pointers to deleted memory at that time.
    for (int i = 0; i < to_be_deleted.size(); i++)
    {
      delete to_be_deleted[i];
    }
  }
  setLevel(Ok);
}

void StatusList::updateLevel()
{
  Level new_level = Ok;

  QHash<QString, StatusProperty*>::iterator iter;
  for (iter = status_children_.begin(); iter != status_children_.end(); iter++)
  {
    Level child_level = iter.value()->getLevel();
    if (child_level > new_level)
    {
      new_level = child_level;
    }
  }
  setLevel(new_level);
}

void StatusList::setLevel(Level new_level)
{
  StatusProperty::setLevel(new_level);
  updateLabel();
}

void StatusList::updateLabel()
{
  StatusProperty::setName(name_prefix_ + ": " + statusWord(getLevel()));
}

} // end namespace rviz
