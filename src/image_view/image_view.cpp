/*
 * Copyright (c) 2008, Willow Garage, Inc.
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


#include <wx/wx.h>
#include <wx/timer.h>

#include "ogre_tools/wx_ogre_render_window.h"
#include "ogre_tools/initialization.h"
#include "rviz/image/ros_image_texture.h"

#include "ros/ros.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureUnitState.h>

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace ogre_tools;
using namespace rviz;

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent)
  : wxFrame(parent, wxID_ANY, wxT("RViZ Image Viewer"), wxDefaultPosition, wxSize(800,600), wxDEFAULT_FRAME_STYLE)
  , timer_(this)
  {
    ogre_tools::initializeOgre();
    ogre_tools::initializeResources( ogre_tools::V_string() );

    root_ = Ogre::Root::getSingletonPtr();

    try
    {
      scene_manager_ = root_->createSceneManager( Ogre::ST_GENERIC, "TestSceneManager" );

      render_window_ = new ogre_tools::wxOgreRenderWindow( root_, this );
      render_window_->SetSize( this->GetSize() );

      camera_ = scene_manager_->createCamera("Camera");

      render_window_->getViewport()->setCamera( camera_ );

      texture_ = new ROSImageTexture(nh_);
      texture_->setTopic("image");

      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create( "Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
      material->setCullingMode(Ogre::CULL_NONE);
      material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
      material->getTechnique(0)->setLightingEnabled(false);
      Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
      tu->setTextureName(texture_->getTexture()->getName());
      tu->setTextureFiltering( Ogre::TFO_NONE );

      Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
      rect->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
      rect->setMaterial(material->getName());
      rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
      Ogre::AxisAlignedBox aabb;
      aabb.setInfinite();
      rect->setBoundingBox(aabb);

      Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
      node->attachObject(rect);
      node->setVisible(true);
    }
    catch ( Ogre::Exception& e )
    {
      printf( "Fatal error: %s\n", e.what() );
      exit(1);
    }

    Connect(timer_.GetId(), wxEVT_TIMER, wxTimerEventHandler(MyFrame::onTimer), NULL, this);
    timer_.Start(16);

    render_window_->Refresh();
  }

  void onTimer(wxTimerEvent&)
  {
    static bool first = true;
    if (texture_->update())
    {
      if (first)
      {
        first = false;

        render_window_->SetSize(texture_->getWidth(), texture_->getHeight());
        Fit();
      }

      render_window_->Refresh();
    }

    if (!nh_.ok())
    {
      Close();
    }
  }

  ~MyFrame()
  {
    delete texture_;
    render_window_->Destroy();
    delete root_;
  }

private:

  Ogre::Root* root_;
  Ogre::SceneManager* scene_manager_;
  ogre_tools::wxOgreRenderWindow* render_window_;
  Ogre::Camera* camera_;
  ROSImageTexture* texture_;
  wxTimer timer_;

  ros::NodeHandle nh_;
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp
{
public:

  bool OnInit()
  {
#ifdef __WXMAC__
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
    SetFrontProcess(&PSN);
#endif

    // create our own copy of argv, with regular char*s.
    char** local_argv =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv, "rviz_image_view", ros::init_options::AnonymousName);

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }

  int OnExit()
  {
    ogre_tools::cleanupOgre();
    return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);
