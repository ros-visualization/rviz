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

#include <ogre_tools/shape.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/arrow.h>
#include <ogre_tools/wx_ogre_render_window.h>

#include <ros/assert.h>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
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
{
  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    pixel_boxes_[i].data = 0;
  }
}

SelectionManager::~SelectionManager()
{
  setSelection(M_Picked());

  clearHandlers();

  highlight_node_->getParentSceneNode()->removeAndDestroyChild(highlight_node_->getName());
  delete highlight_rectangle_;

  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    delete [] (uint8_t*)pixel_boxes_[i].data;
  }
}

void SelectionManager::initialize()
{
  // Create our render textures
  for (uint32_t pass = 0; pass < s_num_render_textures_; ++pass)
  {
    std::stringstream ss;
    static int count = 0;
    ss << "SelectionTexture" << count++;

    render_textures_[pass] = Ogre::TextureManager::getSingleton().createManual(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, 1, 1, 0, Ogre::PF_R8G8B8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* render_texture = render_textures_[pass]->getBuffer()->getRenderTarget();
    render_texture->setAutoUpdated(false);

#if defined(PICKING_DEBUG)
    Ogre::Rectangle2D* mini_screen = new Ogre::Rectangle2D(true);
    mini_screen->setCorners(0.0, pass, 1.0, -1.0 + (pass));
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    mini_screen->setBoundingBox(aabInf);
    Ogre::SceneNode* mini_screen_node = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode(ss.str() + "MiniScreenNode");
    mini_screen_node->attachObject(mini_screen);
    debug_nodes_[pass] = mini_screen_node;

    debug_material_[pass] = Ogre::MaterialManager::getSingleton().create(ss.str() + "RttMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Technique *technique = debug_material_[pass]->createTechnique();
    technique->createPass();
    debug_material_[pass]->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    debug_material_[pass]->getTechnique(0)->getPass(0)->createTextureUnitState(render_textures_[pass]->getName());
    debug_material_[pass]->getTechnique(0)->getPass(0)->setTextureFiltering( Ogre::TFO_NONE );

    mini_screen->setMaterial(debug_material_[pass]->getName());
#endif
  }

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
    handler->enableInteraction(enable);
  }
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
  handler->enableInteraction(interaction_enabled_);

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

  vis_manager_->lockRender();

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

  vis_manager_->unlockRender();
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

  // we ignore the last row
  // see renderAndUnpack

  for (int y = 0; y < h-1; y ++)
  {
    for (int x = 0; x < w; x ++)
    {
      uint32_t pos = (x + y*w) * 4;

      uint32_t pix_val = *(uint32_t*)((uint8_t*)box.data + pos);
      uint32_t handle = colorToHandle(box.format, pix_val);

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

  if ( x1 > x2 )
  {
    int temp = x1;
    x1 = x2;
    x2 = temp;
  }
  if ( y1 > y2 )
  {
    int temp = y1;
    y1 = y2;
    y2 = temp;
  }

  if ( x1 < 0 ) x1 = 0;
  if ( y1 < 0 ) y1 = 0;
  if ( x1 > viewport->getActualWidth()-1 ) x1 = viewport->getActualWidth()-1;
  if ( y1 > viewport->getActualHeight()-1 ) y1 = viewport->getActualHeight()-1;
  if ( x2 < 0 ) x2 = 0;
  if ( y2 < 0 ) y2 = 0;
  if ( x2 > viewport->getActualWidth()-1 ) x2 = viewport->getActualWidth()-1;
  if ( y2 > viewport->getActualHeight()-1 ) y2 = viewport->getActualHeight()-1;

  // note: there is a bug somewhere which makes the lowest line have random values
  // the workaround is to make the image one pixel higher and ignore the last line
  y2++;

  unsigned w = x2-x1;
  unsigned h = y2-y1;

  if ( w==0 ) w=1;
  if ( h==0 ) h=1;

  // check if we need to change the texture size
  if ( render_textures_[pass]->getWidth() != w || render_textures_[pass]->getHeight() != h )
  {
    ROS_DEBUG_STREAM( "Texture for pass " << pass << " must be resized to " << w << " x " << h );
    std::string tex_name = render_textures_[pass]->getName();

    // destroy old
    Ogre::TextureManager::getSingleton().remove( tex_name );

    // create new texture
    render_textures_[pass] = Ogre::TextureManager::getSingleton().createManual( tex_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, w, h, 0,
        Ogre::PF_R8G8B8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);

    render_textures_[pass]->getBuffer()->getRenderTarget()->setAutoUpdated(false);

#if defined(PICKING_DEBUG)
    debug_material_[pass]->removeAllTechniques();
    Ogre::Technique *technique = debug_material_[pass]->createTechnique();
    technique->createPass();
    debug_material_[pass]->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    debug_material_[pass]->getTechnique(0)->getPass(0)->createTextureUnitState(render_textures_[pass]->getName());
    debug_material_[pass]->getTechnique(0)->getPass(0)->setTextureFiltering( Ogre::TFO_NONE );
#endif
  }

  Ogre::TexturePtr tex = render_textures_[pass];
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
  Ogre::RenderTexture* render_texture = pixel_buffer->getRenderTarget();

  // create a viewport if there is none
  if (render_texture->getNumViewports() == 0)
  {
    render_texture->removeAllViewports();
    render_texture->addViewport( viewport->getCamera() );
    Ogre::Viewport* render_viewport = render_texture->getViewport(0);
    render_viewport->setClearEveryFrame(true);
    render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
    render_viewport->setOverlaysEnabled(false);
    render_viewport->setDimensions(0,0,1,1);

    std::stringstream scheme;
    scheme << "Pick";
    if (pass > 0)
    {
      scheme << pass;
    }
    render_viewport->setMaterialScheme(scheme.str());
  }

  // adjust camera parameters so only the selection box gets rendered
  // this is much faster
  Ogre::Viewport* render_viewport = render_texture->getViewport(0);
  Ogre::Camera* camera = render_viewport->getCamera();

  float left,right,top,bottom;
  camera->getFrustumExtents( left,right,top,bottom );

  float x1_rel = (float)x1 / (float)(viewport->getActualWidth()-1);
  float y1_rel = (float)y1 / (float)(viewport->getActualHeight()-1);
  float x2_rel = (float)x2 / (float)(viewport->getActualWidth()-1);
  float y2_rel = (float)y2 / (float)(viewport->getActualHeight()-1);

  float left_new,right_new,top_new,bottom_new;

  left_new = left+x1_rel*(right-left);
  right_new = left+x2_rel*(right-left);
  top_new = top+y1_rel*(bottom-top);
  bottom_new = top+y2_rel*(bottom-top);

  camera->setFrustumExtents( left_new, right_new, top_new, bottom_new );

  ros::Time time1 = ros::Time::now();
  Ogre::MaterialManager::getSingleton().addListener(this);
  render_texture->update();
  Ogre::MaterialManager::getSingleton().removeListener(this);

  // restore old camera state
  camera->setFrustumExtents( left,right,top,bottom );

  int render_width = render_viewport->getActualWidth();
  int render_height = render_viewport->getActualHeight();

  Ogre::PixelFormat format = pixel_buffer->getFormat();

  int size = Ogre::PixelUtil::getMemorySize(render_width, render_height, 1, format);
  uint8_t* data = new uint8_t[size];

  Ogre::PixelBox& box = pixel_boxes_[pass];
  delete [] (uint8_t*)box.data;
  box = Ogre::PixelBox(render_width, render_height, 1, format, data);

  pixel_buffer->blitToMemory(box);

  unpackColors(box, pixels);
  //ROS_INFO_STREAM( ((ros::Time::now() - time1).toSec())*1.000 << "ms" );
}

void SelectionManager::pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, M_Picked& results)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

#if defined(PICKING_DEBUG)
  for (unsigned i = 0; i < s_num_render_textures_; ++i)
  {
    debug_nodes_[i]->setVisible(false);
  }
#endif

  bool need_additional_render = false;

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
          if (handler->needsAdditionalRenderPass(1))
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

  // Reset the "last viewport" of the camera, since picking changes it.
  viewport->getCamera()->_notifyViewport(viewport);

#if defined(PICKING_DEBUG)
  for (unsigned i = 0; i < s_num_render_textures_; ++i)
  {
    debug_nodes_[i]->setVisible(true);
  }
#endif
}

Ogre::Technique *SelectionManager::handleSchemeNotFound(unsigned short schemeIndex, const Ogre::String &schemeName,
                                                  Ogre::Material *originalMaterial, unsigned short lodIndex,
                                                  const Ogre::Renderable *rend)
{
  if(rend)
  {
    Ogre::MaterialPtr material = rend->getMaterial();

    if ( material.get() )
    {
      addPickTechnique(0,material);
    }
  }
  //else
  //  OGRE_LOG("MaterialSwitcher encountered a rendering scheme without a Renderable: " + schemeName + ", " + originalMaterial->getName());
  return 0;
}

void SelectionManager::addPickTechnique(CollObjectHandle handle, const Ogre::MaterialPtr& material)
{
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( &handle, 3 ));

  Ogre::Technique* technique = 0;

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
    Ogre::TextureUnitState* tex_unit = technique->getPass(0)->getTextureUnitState(0);
    std::string tex_name = tex_unit->getTextureName();

    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().getByName(tex_name);
    tex->unload();
    tex->loadRawData(pixel_stream, 1, 1, Ogre::PF_R8G8B8);
  }
}

CollObjectHandle SelectionManager::createCollisionForObject(ogre_tools::Object* obj, const SelectionHandlerPtr& handler, CollObjectHandle coll)
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

  if (ogre_tools::Shape* shape = dynamic_cast<ogre_tools::Shape*>(obj))
  {
    createCollisionForEntity(shape->getEntity(), handler, coll);
    if (!use_original)
    {
      handler->addTrackedObject(shape->getEntity());
    }
  }
  else if (ogre_tools::Axes* axes = dynamic_cast<ogre_tools::Axes*>(obj))
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
  else if (ogre_tools::Arrow* arrow = dynamic_cast<ogre_tools::Arrow*>(obj))
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

  selection_removed_(SelectionRemovedArgs(objs));
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

  selection_added_(SelectionAddedArgs(added));
}

void SelectionManager::setSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  selection_setting_(SelectionSettingArgs());

  M_Picked original(selection_.begin(), selection_.end());

  removeSelection(original);
  addSelection(objs);

  selection_set_(SelectionSetArgs(original, selection_));
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
