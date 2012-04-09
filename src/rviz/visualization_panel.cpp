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

#include <OGRE/OgreLogManager.h>

#include <ros/package.h>
#include <ros/console.h>

#include <ogre_helpers/initialization.h>

#include "view_controller.h"
#include "render_panel.h"
#include "displays_panel.h"
#include "visualization_manager.h"
#include "config.h"

#include "visualization_panel.h"

namespace rviz
{

VisualizationPanel::VisualizationPanel(QWidget* parent)
  : QSplitter( parent )
{
  Ogre::LogManager* log_manager = new Ogre::LogManager();
  log_manager->createLog( "Ogre.log", false, false, true );

  if( !ros::isInitialized() )
  {
    int argc = 0;
    ros::init(argc, 0, "rviz", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  displays_panel_ = new DisplaysPanel( this );
  render_panel_ = new RenderPanel( this );

  QList<int> sizes;
  sizes.push_back( 300 );
  sizes.push_back( 500 );
  setSizes( sizes );

  std::string package_path = ros::package::getPath("rviz");
  V_string paths;
  paths.push_back(package_path + "/ogre_media/textures");
  initializeResources( paths );

  manager_ = new VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  displays_panel_->initialize( manager_ );

  manager_->initialize();
  manager_->startUpdate();
}

VisualizationPanel::~VisualizationPanel()
{
  // TODO: make Properties, PropertyWidgetItems, and things that hold
  // Ogre Node pointers less tightly connected so they can be moved
  // around easier.  The tricky dependencies are a big source of bugs.

  // Have to remove displays before destroying DisplaysPanel, because
  // Displays own Properties, and Properties own children of
  // DisplaysPanel (PropertyWidgetItems).  DisplaysPanel notices when
  // PropertyWidgetItems are destroyed, but Properties don't notice
  // when PropertyWidgetItems are destroyed, so must destroy in the
  // right order.
  if( manager_ )
  {
    manager_->removeAllDisplays();
  }

  delete render_panel_;
  // Have to delete render_panel_ before manager_ because
  // ~VisualizationManager destroys ogre SceneManager which destroys
  // all attached nodes.  RenderPanel indirectly holds pointers to
  // nodes which it destroys.
  delete manager_;
}

void VisualizationPanel::loadDisplayConfig(const std::string& filepath)
{
  manager_->removeAllDisplays();

  boost::shared_ptr<Config> config( new Config() );
  config->readFromFile( filepath );
  manager_->loadDisplayConfig( config );
}

void VisualizationPanel::setViewControllerType( const std::string& view_type_name )
{
  manager_->setCurrentViewControllerType( view_type_name );
}

void VisualizationPanel::setViewString( const std::string& view_string )
{
  manager_->getCurrentViewController()->fromString( view_string );
}

void VisualizationPanel::setTargetFrame( const std::string& target_frame )
{
  manager_->getCurrentViewController()->setTargetFrame( target_frame );
}

}
