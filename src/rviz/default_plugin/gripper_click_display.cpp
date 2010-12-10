/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "gripper_click_display.h"

#include "rviz/window_manager_interface.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"

#include <OGRE/OgreRenderWindow.h>

namespace rviz {

GripperClickDisplay::GripperClickDisplay( const std::string& name, VisualizationManager* manager )
  : Display( name, manager )
{
  ROS_INFO("Gripper click display created");
  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  ROS_ASSERT(wm);
  wxWindow* parent = wm->getParentWindow();
  ROS_ASSERT(parent);

  render_panel_ = new RenderPanel(parent, false);
  render_panel_->SetSize(wxSize(640, 480));
  wm->addPane(name, render_panel_);

  render_panel_->createRenderWindow();
  /*
  render_panel_->initialize(scene_manager_, vis_manager_);
  render_panel_->setAutoRender(false);
  render_panel_->getViewport()->setOverlaysEnabled(false);
  render_panel_->getViewport()->setClearEveryFrame(true);
  render_panel_->getRenderWindow()->setActive(false);
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );
  */
}
  
GripperClickDisplay::~GripperClickDisplay()
{
  ROS_INFO("Gripper click display destroyed");
  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  wm->removePane(render_panel_);
}

void GripperClickDisplay::onEnable()
{
  ROS_INFO("Gripper click display enabled");
  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  wm->showPane(render_panel_);
  render_panel_->getRenderWindow()->setActive(true);
}

void GripperClickDisplay::onDisable()
{
  ROS_INFO("Gripper click display disabled");
  render_panel_->getRenderWindow()->setActive(false);
  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  wm->closePane(render_panel_);
}

}
