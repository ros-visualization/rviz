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
#ifndef RVIZ_PROPERTY_TREE_WITH_HELP_H
#define RVIZ_PROPERTY_TREE_WITH_HELP_H

#include <QSplitter>

#include "rviz/config.h"

class QTextBrowser;

namespace rviz
{
class Property;
class PropertyTreeWidget;

/** A PropertyTreeWidget with built-in help info display. */
class PropertyTreeWithHelp : public QSplitter
{
  Q_OBJECT
public:
  PropertyTreeWithHelp(QWidget* parent = nullptr);

  PropertyTreeWidget* getTree()
  {
    return property_tree_;
  }

  /** @brief Write state to the given Config. */
  void save(Config config) const;

  /** @brief Read state from the given Config. */
  void load(const Config& config);

private Q_SLOTS:
  void showHelpForProperty(const Property* property);

private:
  PropertyTreeWidget* property_tree_;
  QTextBrowser* help_;
};

} // end namespace rviz

#endif // RVIZ_PROPERTY_TREE_WITH_HELP_H
