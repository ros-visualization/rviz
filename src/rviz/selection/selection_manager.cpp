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

#include "selection_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "visualization_manager.h"
#include "render_panel.h"
#include "view_controller.h"

#include <ogre_helpers/shape.h>
#include <ogre_helpers/axes.h>
#include <ogre_helpers/arrow.h>
#include <ogre_helpers/qt_ogre_render_window.h>

#include <ros/assert.h>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderTexture.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreWireBoundingBox.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>

#include <boost/scoped_array.hpp>

#include <algorithm>

namespace rviz
{

SelectionManager::SelectionManager(VisualizationManager* manager)
: vis_manager_(manager)
, highlight_enabled_(false)
, uid_counter_(0)
, interaction_enabled_(false)
, current_viewport_(NULL)
{
  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    pixel_boxes_[i].data = 0;
  }
  depth_pixel_box_.data = 0;
}

SelectionManager::~SelectionManager()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  setSelection(M_Picked());

  highlight_node_->getParentSceneNode()->removeAndDestroyChild(highlight_node_->getName());
  delete highlight_rectangle_;

  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    delete [] (uint8_t*)pixel_boxes_[i].data;
  }
  delete [] (uint8_t*)depth_pixel_box_.data;

  vis_manager_->getSceneManager()->destroyCamera( camera_ );
}

void SelectionManager::initialize( bool debug )
{
  debug_mode_ = debug;

  // Create our render textures
  setTextureSize(1);

  // Create our highlight rectangle
  Ogre::SceneManager* scene_manager = vis_manager_->getSceneManager();
  highlight_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();

  std::stringstream ss;
  static int count = 0;
  ss << "SelectionRect" << count++;
  highlight_rectangle_ = new Ogre::Rectangle2D(true);

  const static uint32_t texture_data[1] = { 0xffff0080 };
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 4 ));

  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(ss.str() + "Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1, Ogre::PF_R8G8B8A8, Ogre::TEX_TYPE_2D, 0);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  //material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
  highlight_rectangle_->setMaterial(material->getName());
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  highlight_rectangle_->setBoundingBox(aabInf);
  highlight_rectangle_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setCullingMode(Ogre::CULL_NONE);

  Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tex_unit->setTextureName(tex->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  highlight_node_->attachObject(highlight_rectangle_);

  // create picking camera
  camera_= scene_manager->createCamera( ss.str()+"_camera" );

  // create fallback picking material
  fallback_pick_material_ = Ogre::MaterialManager::getSingleton().create( "SelectionManagerFallbackMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  addPickTechnique( 0, fallback_pick_material_ );
  fallback_pick_material_->load();
  fallback_pick_technique_ = 0;

  for (uint32_t i = 0; i < fallback_pick_material_->getNumTechniques(); ++i)
  {
    Ogre::Technique* tech = fallback_pick_material_->getTechnique(i);

    if (tech->getSchemeName() == "Pick")
    {
      fallback_pick_technique_ = tech;
    }
  }
}

void SelectionManager::initDepthFinder()
{
  ROS_DEBUG("SelectionManager::initDepthFinder()");
  std::string tex_name = "DepthTexture";

  if( depth_render_texture_.get() )
  {
    Ogre::TextureManager::getSingleton().remove( tex_name );
  }

  depth_texture_size_ = 1;
  depth_render_texture_ =
    Ogre::TextureManager::getSingleton().createManual( tex_name,
                                                       Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                       Ogre::TEX_TYPE_2D, depth_texture_size_, depth_texture_size_, 0,
                                                       Ogre::PF_R8G8B8,
                                                       Ogre::TU_RENDERTARGET );
  Ogre::RenderTexture* render_texture = depth_render_texture_->getBuffer()->getRenderTarget();
  render_texture->setAutoUpdated(false);

  if( debug_mode_ )
  {
    if ( debug_depth_material_.get() )
    {
      debug_depth_material_->removeAllTechniques();
    }
    else
    {
      Ogre::Rectangle2D* mini_screen = new Ogre::Rectangle2D(true);
      float size = 0.6;

      float left = 1.0-size;
      float top = 1.0 - size * (float)2 * 1.02;
      float right = left + size;
      float bottom = top - size;

      mini_screen->setCorners(left,top,right,bottom);
      Ogre::AxisAlignedBox aabInf;
      aabInf.setInfinite();
      mini_screen->setBoundingBox(aabInf);
      Ogre::SceneNode* mini_screen_node = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode(tex_name + "MiniScreenNode");
      mini_screen_node->attachObject(mini_screen);
      debug_depth_node_ = mini_screen_node;

      debug_depth_material_ = Ogre::MaterialManager::getSingleton().create(tex_name + "RttMat",
                                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      mini_screen->setMaterial(debug_depth_material_->getName());
    }

    Ogre::Technique *technique = debug_depth_material_->createTechnique();
    technique->createPass();
    technique->getPass(0)->setLightingEnabled(false);
    technique->getPass(0)->createTextureUnitState(depth_render_texture_->getName());
    technique->getPass(0)->setTextureFiltering( Ogre::TFO_NONE );
  }
}

void SelectionManager::setDebugVisibility( bool visible )
{
  if (debug_mode_)
  {
    for (unsigned i = 0; i < s_num_render_textures_; ++i)
    {
      debug_nodes_[i]->setVisible( visible );
    }
    debug_depth_node_->setVisible( visible );
  }
}

bool SelectionManager::get3DPoint( Ogre::Viewport* viewport, int x, int y, Ogre::Vector3& result_point )
{
  ROS_DEBUG("SelectionManager.get3DPoint()");

  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  setDebugVisibility( false );
  current_viewport_ = viewport;

  M_CollisionObjectToSelectionHandler::iterator handler_it = objects_.begin();
  M_CollisionObjectToSelectionHandler::iterator handler_end = objects_.end();
  for (; handler_it != handler_end; ++handler_it)
  {
    const SelectionHandlerPtr& handler = handler_it->second;
    handler->preRenderPass(0);
  }

  bool success = false;
  if( render( viewport, depth_render_texture_, x, y, x + 1, y + 1, depth_pixel_box_, "Depth", depth_texture_size_ ))
  {
    uint8_t* data_ptr = (uint8_t*) depth_pixel_box_.data;
    uint8_t a = *data_ptr++;
    uint8_t b = *data_ptr++;
    uint8_t c = *data_ptr++;

    int int_depth = (c << 16) | (b << 8) | a;
    float normalized_depth = ((float) int_depth) / (float) 0xffffff;

    float depth = normalized_depth * camera_->getFarClipDistance();
    
    if( depth != 0 )
    {
      Ogre::Matrix4 projection = camera_->getProjectionMatrix();
      if( projection[3][3] == 0.0 ) // If this is a perspective projection
      {
        // We don't use camera_->getCameraToViewportRay() here because
        // it normalizes the ray direction vector.  We need the scale
        // of the direction vector to account for the fact that the
        // depth value we get is not a distance from the camera, it is
        // a depth coordinate.  If we used the normalized vector, a
        // sweep along a plane parallel to the camera plane would
        // yield an arc of points instead of a line.
        Ogre::Matrix4 view = camera_->getViewMatrix();
        Ogre::Matrix4 pv = projection * view;
        Ogre::Matrix4 ip = pv.inverse();

        Ogre::Vector4 near_point(0, 0, -1, 1);
        Ogre::Vector4 far_point(0, 0, 0, 1);

        Ogre::Vector4 ray_origin = ip * near_point;
        Ogre::Vector4 ray_target = ip * far_point;
      
        ray_origin /= ray_origin[3];
        ray_target /= ray_target[3];

        Ogre::Vector3 origin3( ray_origin[0], ray_origin[1], ray_origin[2] );
        Ogre::Vector3 target3( ray_target[0], ray_target[1], ray_target[2] );

        Ogre::Vector3 dir = target3 - origin3;

        // TODO: Not sure where this scale factor actually comes from nor its precise value. (hersh)
        float magic_scale_factor = 100;
        result_point = target3 + dir * magic_scale_factor * depth;
      }
      else // else this must be an orthographic projection.
      {
        // For orthographic projection, getCameraToViewportRay() does
        // the right thing for us, and the above math does not work.
        Ogre::Ray ray;
        camera_->getCameraToViewportRay( 0.5, 0.5, &ray );

        result_point = ray.getPoint( depth );
      }

      ROS_DEBUG("SelectionManager.get3DPoint(): point = %f, %f, %f", result_point.x, result_point.y, result_point.z);

      success = true;
    }
  }

  handler_it = objects_.begin();
  handler_end = objects_.end();
  for (; handler_it != handler_end; ++handler_it)
  {
    const SelectionHandlerPtr& handler = handler_it->second;
    handler->postRenderPass(0);
  }
  current_viewport_ = NULL;
  setDebugVisibility( true );
  return success;
}

void SelectionManager::setTextureSize( unsigned size )
{
  if ( size > 1024 )
  {
    size = 1024;
  }

  texture_size_ = size;

  for (uint32_t pass = 0; pass < s_num_render_textures_; ++pass)
  {
    // check if we need to change the texture size
    if ( !render_textures_[pass].get() || render_textures_[pass]->getWidth() != size )
    {
      std::string tex_name;
      if ( render_textures_[pass].get() )
      {
        ROS_INFO_STREAM( "Texture for pass " << pass << " must be resized to " << size << " x " << size );
        tex_name = render_textures_[pass]->getName();

        // destroy old
        Ogre::TextureManager::getSingleton().remove( tex_name );
      }
      else
      {
        ROS_INFO_STREAM( "Texture for pass " << pass << ": creating with size " << size << " x " << size );
        std::stringstream ss;
        static int count = 0;
        ss << "SelectionTexture" << count++;
        tex_name = ss.str();
      }

      // create new texture
      render_textures_[pass] = Ogre::TextureManager::getSingleton().createManual( tex_name,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, size, size, 0,
          Ogre::PF_R8G8B8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);

      Ogre::RenderTexture* render_texture = render_textures_[pass]->getBuffer()->getRenderTarget();
      render_texture->setAutoUpdated(false);

      if (debug_mode_)
      {
        if ( debug_material_[pass].get() )
        {
          debug_material_[pass]->removeAllTechniques();
        }
        else
        {
          Ogre::Rectangle2D* mini_screen = new Ogre::Rectangle2D(true);
          float size = 0.6;

          float left = 1.0-size;
          float top = 1.0 - size * (float)pass * 1.02;
          float right = left + size;
          float bottom = top - size;

          mini_screen->setCorners(left,top,right,bottom);
          Ogre::AxisAlignedBox aabInf;
          aabInf.setInfinite();
          mini_screen->setBoundingBox(aabInf);
          Ogre::SceneNode* mini_screen_node = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode(tex_name + "MiniScreenNode");
          mini_screen_node->attachObject(mini_screen);
          debug_nodes_[pass] = mini_screen_node;

          debug_material_[pass] = Ogre::MaterialManager::getSingleton().create(tex_name + "RttMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
          mini_screen->setMaterial(debug_material_[pass]->getName());
        }

        Ogre::Technique *technique = debug_material_[pass]->createTechnique();
        technique->createPass();
        technique->getPass(0)->setLightingEnabled(false);
        technique->getPass(0)->createTextureUnitState(render_textures_[pass]->getName());
        technique->getPass(0)->setTextureFiltering( Ogre::TFO_NONE );
      }
    }
  }

  initDepthFinder();
}

void SelectionManager::clearHandlers()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  objects_.clear();
}

void SelectionManager::enableInteraction( bool enable )
{
  interaction_enabled_ = enable;
  M_CollisionObjectToSelectionHandler::iterator handler_it = objects_.begin();
  M_CollisionObjectToSelectionHandler::iterator handler_end = objects_.end();
  for (; handler_it != handler_end; ++handler_it)
  {
    const SelectionHandlerPtr& handler = handler_it->second;
    InteractiveObjectPtr object = handler->getInteractiveObject().lock();
    if( object )
    {
      object->enableInteraction( enable );
    }
  }
}

CollObjectHandle SelectionManager::createHandle()
{
  uid_counter_++;
  if (uid_counter_ > 0x00ffffff)
  {
    uid_counter_ = 0;
  }

  uint32_t handle = 0;

  // shuffle around the bits so we get lots of colors
  // when we're displaying the selection buffer
  for ( unsigned int i=0; i<24; i++ )
  {
    uint32_t shift = (((23-i)%3)*8) + (23-i)/3;
    uint32_t bit = ( (uint32_t)(uid_counter_ >> i) & (uint32_t)1 ) << shift;
    handle |= bit;
  }

  return handle;
}

void SelectionManager::addObject(CollObjectHandle obj, const SelectionHandlerPtr& handler)
{
  if (!obj)
  {
//    ROS_BREAK();
    return;
  }

  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  handler->initialize(vis_manager_);

  InteractiveObjectPtr object = handler->getInteractiveObject().lock();
  if( object )
  {
    object->enableInteraction( interaction_enabled_ );
  }

  bool inserted = objects_.insert(std::make_pair(obj, handler)).second;
  ROS_ASSERT(inserted);
}

void SelectionManager::removeObject(CollObjectHandle obj)
{
  if (!obj)
  {
    return;
  }

  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked::iterator it = selection_.find(obj);
  if (it != selection_.end())
  {
    M_Picked objs;
    objs.insert(std::make_pair(it->first, it->second));

    removeSelection(objs);
  }

  objects_.erase(obj);
}

void SelectionManager::update()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  highlight_node_->setVisible(highlight_enabled_);

  if (highlight_enabled_)
  {
    setHighlightRect(highlight_.viewport, highlight_.x1, highlight_.y1, highlight_.x2, highlight_.y2);

#if 0
    M_Picked results;
    highlight_node_->setVisible(false);
    pick(highlight_.viewport, highlight_.x1, highlight_.y1, highlight_.x2, highlight_.y2, results);
    highlight_node_->setVisible(true);
#endif
  }
}

void SelectionManager::highlight(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  highlight_enabled_ = true;

  highlight_.viewport = viewport;
  highlight_.x1 = x1;
  highlight_.y1 = y1;
  highlight_.x2 = x2;
  highlight_.y2 = y2;
}

void SelectionManager::removeHighlight()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  highlight_enabled_ = false;
}

void SelectionManager::select(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, SelectType type)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  highlight_enabled_ = false;
  highlight_node_->setVisible(false);

  M_Picked results;
  pick(viewport, x1, y1, x2, y2, results);

  if (type == Add)
  {
    addSelection(results);
  }
  else if (type == Remove)
  {
    removeSelection(results);
  }
  else if (type == Replace)
  {
    setSelection(results);
  }
}

void SelectionManager::setHighlightRect(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2)
{
  float nx1 = ((float)x1 / viewport->getActualWidth()) * 2 - 1;
  float nx2 = ((float)x2 / viewport->getActualWidth()) * 2 - 1;
  float ny1 = -(((float)y1 / viewport->getActualHeight()) * 2 - 1);
  float ny2 = -(((float)y2 / viewport->getActualHeight()) * 2 - 1);

  nx1 = nx1 < -1 ? -1 : (nx1 > 1 ? 1 : nx1);
  ny1 = ny1 < -1 ? -1 : (ny1 > 1 ? 1 : ny1);
  nx2 = nx2 < -1 ? -1 : (nx2 > 1 ? 1 : nx2);
  ny2 = ny2 < -1 ? -1 : (ny2 > 1 ? 1 : ny2);

  highlight_rectangle_->setCorners(nx1, ny1, nx2, ny2);
}

void SelectionManager::unpackColors( const Ogre::PixelBox& box, V_Pixel& pixels)
{
  int w = box.getWidth();
  int h = box.getHeight();

  pixels.clear();
  pixels.reserve( w*h );

  for (int y = 0; y < h; y ++)
  {
    for (int x = 0; x < w; x ++)
    {
      uint32_t pos = (x + y*w) * 4;

      uint32_t pix_val = *(uint32_t*)((uint8_t*)box.data + pos);
      uint32_t handle = colorToHandle(box.format, pix_val);

      //ROS_INFO( "%d %d = %u", x, y, handle );

      Pixel p;
      p.x = x;
      p.y = y;
      p.handle = handle;

      pixels.push_back(p);
    }
  }
}

void SelectionManager::renderAndUnpack(Ogre::Viewport* viewport, uint32_t pass, int x1, int y1, int x2, int y2, V_Pixel& pixels)
{
  ROS_ASSERT(pass < s_num_render_textures_);

  std::stringstream scheme;
  scheme << "Pick";
  if (pass > 0)
  {
    scheme << pass;
  }

  if( render( viewport, render_textures_[pass], x1, y1, x2, y2, pixel_boxes_[pass], scheme.str(), texture_size_ ))
  {
    unpackColors(pixel_boxes_[pass], pixels);
  }
}

bool SelectionManager::render(Ogre::Viewport* viewport, Ogre::TexturePtr tex,
                              int x1, int y1, int x2, int y2,
                              Ogre::PixelBox& dst_box, std::string material_scheme,
                              unsigned texture_size)
{
  vis_manager_->lockRender();

  if ( x2==x1 ) x2++;
  if ( y2==y1 ) y2++;

  if ( x1 > x2 ) std::swap( x1, x2 );
  if ( y1 > y2 ) std::swap( y1, y2 );

  if ( x1 < 0 ) x1 = 0;
  if ( y1 < 0 ) y1 = 0;
  if ( x1 > viewport->getActualWidth() ) x1 = viewport->getActualWidth();
  if ( y1 > viewport->getActualHeight() ) y1 = viewport->getActualHeight();
  if ( x2 < 0 ) x2 = 0;
  if ( y2 < 0 ) y2 = 0;
  if ( x2 > viewport->getActualWidth() ) x2 = viewport->getActualWidth();
  if ( y2 > viewport->getActualHeight() ) y2 = viewport->getActualHeight();

  if ( x2==x1 || y2==y1 )
  {
    ROS_WARN("SelectionManager::render(): not rendering 0 size area.");
    vis_manager_->unlockRender();
    return false;
  }

  unsigned w = x2-x1;
  unsigned h = y2-y1;

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
  Ogre::RenderTexture* render_texture = pixel_buffer->getRenderTarget();

  Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  Ogre::Matrix4 scale_matrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 trans_matrix = Ogre::Matrix4::IDENTITY;

  float x1_rel = (float)x1 / (float)(viewport->getActualWidth()-1) - 0.5;
  float y1_rel = (float)y1 / (float)(viewport->getActualHeight()-1) - 0.5;
  float x2_rel = (float)x2 / (float)(viewport->getActualWidth()-1) - 0.5;
  float y2_rel = (float)y2 / (float)(viewport->getActualHeight()-1) - 0.5;

  scale_matrix[0][0] = 1.0 / (x2_rel-x1_rel);
  scale_matrix[1][1] = 1.0 / (y2_rel-y1_rel);

  trans_matrix[0][3] -= x1_rel+x2_rel;
  trans_matrix[1][3] += y1_rel+y2_rel;

  camera_->setCustomProjectionMatrix( true, scale_matrix * trans_matrix * proj_matrix );
  camera_->setPosition( viewport->getCamera()->getDerivedPosition() );
  camera_->setOrientation( viewport->getCamera()->getDerivedOrientation() );

  // Note: if you change this far-clip distance, update
  // fixed_orientation_ortho_view_controller.cpp where it sets the
  // camera position Z value ot half of this.
  camera_->setFarClipDistance( 1000 );
  camera_->setNearClipDistance( 0.1 );

  // create a viewport if there is none
  if (render_texture->getNumViewports() == 0)
  {
    render_texture->removeAllViewports();
    render_texture->addViewport( camera_ );
    Ogre::Viewport* render_viewport = render_texture->getViewport(0);
    render_viewport->setClearEveryFrame(true);
    render_viewport->setBackgroundColour( Ogre::ColourValue::Black );
    render_viewport->setOverlaysEnabled(false);
    render_viewport->setMaterialScheme(material_scheme);
  }

  unsigned render_w = w;
  unsigned render_h = h;

  if ( w>h )
  {
    if ( render_w > texture_size )
    {
      render_w = texture_size;
      render_h = round( float(h) * (float)texture_size / (float)w );
    }
  }
  else
  {
    if ( render_h > texture_size )
    {
      render_h = texture_size;
      render_w = round( float(w) * (float)texture_size / (float)h );
    }
  }

  // safety clamping in case of rounding errors
  if ( render_w > texture_size ) render_w = texture_size;
  if ( render_h > texture_size ) render_h = texture_size;

  // set viewport to render to a subwindow of the texture
  Ogre::Viewport* render_viewport = render_texture->getViewport(0);
  render_viewport->setDimensions( 0, 0,
                                  (float)render_w / (float)texture_size,
                                  (float)render_h / (float)texture_size );

  ros::WallTime start = ros::WallTime::now();

  // update & force ogre to render the scene
  Ogre::MaterialManager::getSingleton().addListener(this);

  render_texture->update();

  // For some reason we need to pretend to render the main window in
  // order to get the picking render to show up in the pixelbox below.
  // If we don't do this, it will show up there the *next* time we
  // pick something, but not this time.  This object as a
  // render queue listener tells the scene manager to skip every
  // render step, so nothing actually gets drawn.
  // 
  // TODO: find out what part of _renderScene() actually makes this work.
  Ogre::Viewport* main_view = vis_manager_->getRenderPanel()->getViewport();
  vis_manager_->getSceneManager()->addRenderQueueListener(this);
  vis_manager_->getSceneManager()->_renderScene(main_view->getCamera(), main_view, false);
  vis_manager_->getSceneManager()->removeRenderQueueListener(this);

  ros::WallTime end = ros::WallTime::now();
  ros::WallDuration d = end - start;
//  ROS_DEBUG("Render took [%f] msec", d.toSec() * 1000.0f);

  Ogre::MaterialManager::getSingleton().removeListener(this);

  render_w = render_viewport->getActualWidth();
  render_h = render_viewport->getActualHeight();

  Ogre::PixelFormat format = pixel_buffer->getFormat();

  int size = Ogre::PixelUtil::getMemorySize(render_w, render_h, 1, format);
  uint8_t* data = new uint8_t[size];

  delete [] (uint8_t*)dst_box.data;
  dst_box = Ogre::PixelBox(render_w, render_h, 1, format, data);

  pixel_buffer->blitToMemory(dst_box,dst_box);

  vis_manager_->unlockRender();
  return true;
}

void SelectionManager::renderQueueStarted( uint8_t queueGroupId,
                                           const std::string& invocation, 
                                           bool& skipThisInvocation )
{
  // This render queue listener function tells the scene manager to
  // skip every render step, so nothing actually gets drawn.

//  ROS_DEBUG("SelectionManager renderQueueStarted(%d, '%s') returning skip = true.", (int)queueGroupId, invocation.c_str());
  skipThisInvocation = true;
}

void SelectionManager::pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, M_Picked& results, bool single_render_pass)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  setDebugVisibility( false );

  bool need_additional_render = false;
  current_viewport_ = viewport;

  V_CollObject handles_by_pixel;
  S_CollObject need_additional;

  V_Pixel& pixels = pixel_buffer_;

  // First render is special... does the initial object picking, determines which objects have been selected
  // After that, individual handlers can specify that they need additional renders (max # defined in s_num_render_textures_)
  {
    M_CollisionObjectToSelectionHandler::iterator handler_it = objects_.begin();
    M_CollisionObjectToSelectionHandler::iterator handler_end = objects_.end();
    for (; handler_it != handler_end; ++handler_it)
    {
      const SelectionHandlerPtr& handler = handler_it->second;

      handler->preRenderPass(0);
    }

    renderAndUnpack(viewport, 0, x1, y1, x2, y2, pixels);

    handler_it = objects_.begin();
    handler_end = objects_.end();
    for (; handler_it != handler_end; ++handler_it)
    {
      const SelectionHandlerPtr& handler = handler_it->second;

      handler->postRenderPass(0);
    }

    handles_by_pixel.reserve(pixels.size());
    V_Pixel::iterator it = pixels.begin();
    V_Pixel::iterator end = pixels.end();
    for (; it != end; ++it)
    {
      const Pixel& p = *it;

      CollObjectHandle handle = p.handle;

      handles_by_pixel.push_back(handle);

      if (handle == 0)
      {
        continue;
      }

      SelectionHandlerPtr handler = getHandler(handle);

      if (handle && handler)
      {
        std::pair<M_Picked::iterator, bool> insert_result = results.insert(std::make_pair(handle, Picked(handle)));
        if (insert_result.second)
        {
          if (handler->needsAdditionalRenderPass(1) && !single_render_pass)
          {
            need_additional.insert(handle);
            need_additional_render = true;
          }
        }
        else
        {
          insert_result.first->second.pixel_count++;
        }
      }
    }
  }

  uint32_t pass = 1;

  V_uint64 extra_by_pixel;
  extra_by_pixel.resize(handles_by_pixel.size());
  while (need_additional_render && pass < s_num_render_textures_)
  {
    {
      S_CollObject::iterator need_it = need_additional.begin();
      S_CollObject::iterator need_end = need_additional.end();
      for (; need_it != need_end; ++need_it)
      {
        SelectionHandlerPtr handler = getHandler(*need_it);
        ROS_ASSERT(handler);

        handler->preRenderPass(pass);
      }
    }

    renderAndUnpack(viewport, pass, x1, y1, x2, y2, pixels);

    {
      S_CollObject::iterator need_it = need_additional.begin();
      S_CollObject::iterator need_end = need_additional.end();
      for (; need_it != need_end; ++need_it)
      {
        SelectionHandlerPtr handler = getHandler(*need_it);
        ROS_ASSERT(handler);

        handler->postRenderPass(pass);
      }
    }

    int i = 0;
    V_Pixel::iterator pix_it = pixels.begin();
    V_Pixel::iterator pix_end = pixels.end();
    for (; pix_it != pix_end; ++pix_it, ++i)
    {
      const Pixel& p = *pix_it;

      CollObjectHandle handle = handles_by_pixel[i];

      if (pass == 1)
      {
        extra_by_pixel[i] = 0;
      }

      if (need_additional.find(handle) != need_additional.end())
      {
        CollObjectHandle extra_handle = p.handle;
        extra_by_pixel[i] |= extra_handle << (32 * (pass-1));
      }
      else
      {
        extra_by_pixel[i] = 0;
      }
    }

    need_additional_render = false;
    need_additional.clear();
    M_Picked::iterator handle_it = results.begin();
    M_Picked::iterator handle_end = results.end();
    for (; handle_it != handle_end; ++handle_it)
    {
      CollObjectHandle handle = handle_it->first;

      if (getHandler(handle)->needsAdditionalRenderPass(pass + 1))
      {
        need_additional_render = true;
        need_additional.insert(handle);
      }
    }
  }

  int i = 0;
  V_uint64::iterator pix_2_it = extra_by_pixel.begin();
  V_uint64::iterator pix_2_end = extra_by_pixel.end();
  for (; pix_2_it != pix_2_end; ++pix_2_it, ++i)
  {
    CollObjectHandle handle = handles_by_pixel[i];

    if (handle == 0)
    {
      continue;
    }

    M_Picked::iterator picked_it = results.find(handle);
    if (picked_it == results.end())
    {
      continue;
    }

    Picked& picked = picked_it->second;

    if (*pix_2_it)
    {
      picked.extra_handles.insert(*pix_2_it);
    }
  }

  current_viewport_ = NULL;
  setDebugVisibility( true );
}

Ogre::Technique *SelectionManager::handleSchemeNotFound(unsigned short scheme_index,
    const Ogre::String& scheme_name,
    Ogre::Material* original_material,
    unsigned short lod_index,
    const Ogre::Renderable* rend )
{
  return fallback_pick_technique_;
}

Ogre::Technique *SelectionManager::addPickTechnique(CollObjectHandle handle, const Ogre::MaterialPtr& material)
{
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( &handle, 3 ));

  Ogre::Technique* technique = 0;

  // Look for a technique in the material that has a "Pick" scheme.
  uint32_t num_techs = material->getNumTechniques();
  for (uint32_t i = 0; i < num_techs; ++i)
  {
    Ogre::Technique* tech = material->getTechnique(i);

    if (tech->getSchemeName() == "Pick")
    {
      technique = tech;
      break;
    }
  }

  // If we did not find a "Pick" techique, create one and add it to the material.
  if (!technique)
  {
    // try to preserve the culling mode
    Ogre::CullingMode culling_mode = Ogre::CULL_CLOCKWISE;
    if ( material->getTechnique(0) && material->getTechnique(0)->getNumPasses() > 0 )
    {
      culling_mode = material->getTechnique(0)->getPass(0)->getCullingMode();
    }

    technique = material->createTechnique();
    technique->setSchemeName("Pick");
    Ogre::Pass* pass = technique->createPass();
    pass->setLightingEnabled(false);
    pass->setSceneBlending(Ogre::SBT_REPLACE);
    pass->setCullingMode( culling_mode );

    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(material->getName() + "PickTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1, Ogre::PF_R8G8B8, Ogre::TEX_TYPE_2D, 0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();
    tex_unit->setTextureName(tex->getName());
    tex_unit->setTextureFiltering( Ogre::TFO_NONE );
    tex_unit->setColourOperation(Ogre::LBO_REPLACE);
  }
  else
  {
    // We *did* find a Pick technique, so just set the texture data
    // (in pixel_stream) to be the single pixel from the
    // CollObjectHandle.
    Ogre::TextureUnitState* tex_unit = technique->getPass(0)->getTextureUnitState(0);
    std::string tex_name = tex_unit->getTextureName();

    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().getByName(tex_name);
    tex->unload();
    tex->loadRawData(pixel_stream, 1, 1, Ogre::PF_R8G8B8);
  }

  technique->getPass(0)->_dirtyHash();

  //----- now add a technique for finding depth -----

  // Look for a technique in the material that has a "Depth" scheme.
  bool has_depth = false;
  num_techs = material->getNumTechniques();
  for (uint32_t i = 0; i < num_techs; ++i)
  {
    Ogre::Technique* tech = material->getTechnique(i);

    if (tech->getSchemeName() == "Depth")
    {
      has_depth = true;
      break;
    }
  }

  if( !has_depth )
  {
    // try to preserve the culling mode
    Ogre::CullingMode culling_mode = Ogre::CULL_CLOCKWISE;
    if ( material->getTechnique(0) && material->getTechnique(0)->getNumPasses() > 0 )
    {
      culling_mode = material->getTechnique(0)->getPass(0)->getCullingMode();
    }

    technique = material->createTechnique();
    technique->setSchemeName("Depth");
    Ogre::Pass* pass = technique->createPass();
    pass->setLightingEnabled(false);
    pass->setSceneBlending(Ogre::SBT_REPLACE);
    pass->setCullingMode( culling_mode );
    pass->setVertexProgram( "rviz/DepthVP" );
    pass->setFragmentProgram( "rviz/DepthFP" );
  }
  material->load(false);

  return technique;
}

CollObjectHandle SelectionManager::createCollisionForObject(Object* obj, const SelectionHandlerPtr& handler, CollObjectHandle coll)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  bool use_original = false;

  if (coll)
  {
    use_original = true;
  }
  else
  {
    coll = createHandle();
  }

  if (Shape* shape = dynamic_cast<Shape*>(obj))
  {
    createCollisionForEntity(shape->getEntity(), handler, coll);
    if (!use_original)
    {
      handler->addTrackedObject(shape->getEntity());
    }
  }
  else if (Axes* axes = dynamic_cast<Axes*>(obj))
  {
    createCollisionForEntity(axes->getXShape()->getEntity(), handler, coll);
    createCollisionForEntity(axes->getYShape()->getEntity(), handler, coll);
    createCollisionForEntity(axes->getZShape()->getEntity(), handler, coll);

    if (!use_original)
    {
      handler->addTrackedObject(axes->getXShape()->getEntity());
      handler->addTrackedObject(axes->getYShape()->getEntity());
      handler->addTrackedObject(axes->getZShape()->getEntity());
    }
  }
  else if (Arrow* arrow = dynamic_cast<Arrow*>(obj))
  {
    createCollisionForEntity(arrow->getHead()->getEntity(), handler, coll);
    createCollisionForEntity(arrow->getShaft()->getEntity(), handler, coll);

    if (!use_original)
    {
      handler->addTrackedObject(arrow->getHead()->getEntity());
      handler->addTrackedObject(arrow->getShaft()->getEntity());
    }
  }

  if (coll)
  {
    if (!use_original)
    {
      addObject(coll, handler);
    }
  }

  return coll;
}

CollObjectHandle SelectionManager::createCollisionForEntity(Ogre::Entity* entity, const SelectionHandlerPtr& handler, CollObjectHandle coll)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  bool use_original = false;

  if (coll)
  {
    use_original = true;
  }
  else
  {
    coll = createHandle();
  }

  typedef std::set<Ogre::Material*> M_Material;
  M_Material materials;

  uint32_t num_sub_entities = entity->getNumSubEntities();
  for (uint32_t i = 0; i < num_sub_entities; ++i)
  {
    Ogre::SubEntity* sub = entity->getSubEntity(i);

    Ogre::MaterialPtr material = sub->getMaterial();

    if (materials.insert(material.get()).second)
    {
      addPickTechnique(coll, material);
    }
  }

  if (!use_original)
  {
    handler->addTrackedObject(entity);
    addObject(coll, handler);
  }

  return coll;
}

SelectionHandlerPtr SelectionManager::getHandler(CollObjectHandle obj)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_CollisionObjectToSelectionHandler::iterator it = objects_.find(obj);
  if (it != objects_.end())
  {
    return it->second;
  }

  return SelectionHandlerPtr();
}

void SelectionManager::removeSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked::const_iterator it = objs.begin();
  M_Picked::const_iterator end = objs.end();
  for (; it != end; ++it)
  {
    removeSelection(it->second);
  }

  Q_EMIT selectionRemoved( objs );
}

void SelectionManager::addSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked added;
  M_Picked::const_iterator it = objs.begin();
  M_Picked::const_iterator end = objs.end();
  for (; it != end; ++it)
  {
    std::pair<Picked, bool> ppb = addSelection(it->second);
    if (ppb.second)
    {
      added.insert(std::make_pair(it->first, ppb.first));
    }
  }

  Q_EMIT selectionAdded( added );
}

void SelectionManager::setSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  Q_EMIT selectionSetting();

  M_Picked original(selection_.begin(), selection_.end());

  removeSelection(original);
  addSelection(objs);

  Q_EMIT selectionSet( original, selection_ );
}

std::pair<Picked, bool> SelectionManager::addSelection(const Picked& obj)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  std::pair<M_Picked::iterator, bool> pib = selection_.insert(std::make_pair(obj.handle, obj));

  SelectionHandlerPtr handler = getHandler(obj.handle);

  if (pib.second)
  {
    handler->onSelect(obj);
    return std::make_pair(obj, true);
  }
  else
  {
    Picked& cur = pib.first->second;
    Picked added(cur.handle);

    S_uint64::iterator it = obj.extra_handles.begin();
    S_uint64::iterator end = obj.extra_handles.end();
    for (; it != end; ++it)
    {
      if (cur.extra_handles.insert(*it).second)
      {
        added.extra_handles.insert(*it);
      }
    }

    if (!added.extra_handles.empty())
    {
      handler->onSelect(added);

      return std::make_pair(added, true);
    }
  }

  return std::make_pair(Picked(0), false);
}

void SelectionManager::removeSelection(const Picked& obj)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked::iterator sel_it = selection_.find(obj.handle);
  if (sel_it != selection_.end())
  {
    S_uint64::iterator extra_it = obj.extra_handles.begin();
    S_uint64::iterator extra_end = obj.extra_handles.end();
    for (; extra_it != extra_end; ++extra_it)
    {
      sel_it->second.extra_handles.erase(*extra_it);
    }

    if (sel_it->second.extra_handles.empty())
    {
      selection_.erase(sel_it);
    }
  }

  SelectionHandlerPtr handler = getHandler(obj.handle);
  handler->onDeselect(obj);
}

void SelectionManager::focusOnSelection()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  if (selection_.empty())
  {
    return;
  }

  Ogre::AxisAlignedBox combined;

  M_Picked::iterator it = selection_.begin();
  M_Picked::iterator end = selection_.end();
  for (; it != end; ++it)
  {
    const Picked& p = it->second;

    SelectionHandlerPtr handler = getHandler(p.handle);

    V_AABB aabbs;
    handler->getAABBs(p, aabbs);

    V_AABB::iterator aabb_it = aabbs.begin();
    V_AABB::iterator aabb_end = aabbs.end();
    for (; aabb_it != aabb_end; ++aabb_it)
    {
      combined.merge(*aabb_it);
    }
  }

  if (!combined.isInfinite() && !combined.isNull())
  {
    Ogre::Vector3 center = combined.getCenter();
    ViewController* controller = vis_manager_->getCurrentViewController();
    controller->lookAt(center);
  }
}

} // namespace rviz
