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

#include <QTextBrowser>

#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_widget.h>

#include <rviz/properties/property_tree_with_help.h>

namespace rviz
{
PropertyTreeWithHelp::PropertyTreeWithHelp(QWidget* parent) : QSplitter(parent)
{
  setOrientation(Qt::Vertical);

  property_tree_ = new PropertyTreeWidget;

  help_ = new QTextBrowser;
  help_->setOpenExternalLinks(true);

  addWidget(property_tree_);
  addWidget(help_);

  setStretchFactor(0, 1000);
  setCollapsible(0, false);

  QList<int> _sizes;
  _sizes.push_back(1000);
  _sizes.push_back(1);
  setSizes(_sizes);

  connect(property_tree_, SIGNAL(currentPropertyChanged(const Property*)), this,
          SLOT(showHelpForProperty(const Property*)));
}

void PropertyTreeWithHelp::showHelpForProperty(const Property* property)
{
  if (property)
  {
    QString body_text = property->getDescription();
    QString heading = property->getName();
    QString html = "<html><body><strong>" + heading + "</strong><br>" + body_text + "</body></html>";
    help_->setHtml(html);
  }
  else
  {
    help_->setHtml("");
  }
}

void PropertyTreeWithHelp::save(Config config) const
{
  property_tree_->save(config.mapMakeChild("Property Tree Widget"));

  QList<int> _sizes = sizes();
  config.mapSetValue("Tree Height", _sizes.at(0));
  config.mapSetValue("Help Height", _sizes.at(1));
}

void PropertyTreeWithHelp::load(const Config& config)
{
  property_tree_->load(config.mapGetChild("Property Tree Widget"));

  int tree_height;
  int help_height;
  if (config.mapGetInt("Tree Height", &tree_height) && config.mapGetInt("Help Height", &help_height))
  {
    QList<int> _sizes;
    _sizes.push_back(tree_height);
    _sizes.push_back(help_height);
    setSizes(_sizes);
  }
}

} // end namespace rviz
