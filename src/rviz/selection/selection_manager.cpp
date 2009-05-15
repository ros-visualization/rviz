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
#include "common.h"

#include <ogre_tools/shape.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/arrow.h>
#include <ogre_tools/wx_ogre_render_window.h>
#include <ogre_tools/camera_base.h>

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

SelectionHandler::SelectionHandler()
: manager_(0)
, listener_(new Listener(this))
{

}

SelectionHandler::~SelectionHandler()
{
  S_Movable::iterator it = tracked_objects_.begin();
  S_Movable::iterator end = tracked_objects_.end();
  for (; it != end; ++it)
  {
    Ogre::MovableObject* m = *it;
    m->setListener(0);
  }

  while (!boxes_.empty())
  {
    destroyBox(boxes_.begin()->first);
  }
}

void SelectionHandler::initialize(VisualizationManager* manager)
{
  manager_ = manager;
}

void SelectionHandler::preRenderPass(uint32_t pass)
{
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it)
  {
    Ogre::WireBoundingBox* box = it->second.second;
    box->setVisible(false);
  }
}

void SelectionHandler::postRenderPass(uint32_t pass)
{
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it)
  {
    Ogre::WireBoundingBox* box = it->second.second;
    box->setVisible(true);
  }
}

void SelectionHandler::addTrackedObject(Ogre::MovableObject* object)
{
  tracked_objects_.insert(object);
  object->setListener(listener_.get());
}

void SelectionHandler::removeTrackedObject(Ogre::MovableObject* object)
{
  tracked_objects_.erase(object);
  object->setListener(0);

  updateTrackedBoxes();
}

void SelectionHandler::updateTrackedBoxes()
{
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it)
  {
    V_AABB aabbs;
    Picked p(it->first.first);
    p.extra_handles.insert(it->first.second);
    getAABBs(Picked(it->first.first), aabbs);

    if (!aabbs.empty())
    {
      Ogre::AxisAlignedBox combined;
      V_AABB::iterator aabb_it = aabbs.begin();
      V_AABB::iterator aabb_end = aabbs.end();
      for (; aabb_it != aabb_end; ++aabb_it)
      {
        combined.merge(*aabb_it);
      }

      createBox(std::make_pair(p.handle, it->first.second), combined, "RVIZ/Cyan");
    }
  }
}

void SelectionHandler::getAABBs(const Picked& obj, V_AABB& aabbs)
{
  S_Movable::iterator it = tracked_objects_.begin();
  S_Movable::iterator end = tracked_objects_.end();
  for (; it != end; ++it)
  {
    aabbs.push_back((*it)->getWorldBoundingBox());
  }
}

void SelectionHandler::updateProperties()
{
  V_Property::iterator it = properties_.begin();
  V_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    propertyChanged(*it);
  }
}

void SelectionHandler::destroyProperties(const Picked& obj, PropertyManager* property_manager)
{
  V_Property::iterator it = properties_.begin();
  V_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    property_manager->deleteProperty((*it).lock());
  }
}

void SelectionHandler::createBox(const std::pair<CollObjectHandle, uint64_t>& handles, const Ogre::AxisAlignedBox& aabb, const std::string& material_name)
{
  Ogre::WireBoundingBox* box = 0;
  Ogre::SceneNode* node = 0;

  M_HandleToBox::iterator it = boxes_.find(handles);
  if (it == boxes_.end())
  {
    Ogre::SceneManager* scene_manager = manager_->getSceneManager();
    node = scene_manager->getRootSceneNode()->createChildSceneNode();
    box = new Ogre::WireBoundingBox;

    bool inserted = boxes_.insert(std::make_pair(handles, std::make_pair(node, box))).second;
    ROS_ASSERT(inserted);
  }
  else
  {
    node = it->second.first;
    box = it->second.second;
  }

  box->setMaterial(material_name);

  box->setupBoundingBox(aabb);
  node->detachAllObjects();
  node->attachObject(box);
}

void SelectionHandler::destroyBox(const std::pair<CollObjectHandle, uint64_t>& handles)
{
  M_HandleToBox::iterator it = boxes_.find(handles);
  if (it != boxes_.end())
  {
    Ogre::SceneNode* node = it->second.first;
    Ogre::WireBoundingBox* box = it->second.second;

    node->detachAllObjects();
    node->getParentSceneNode()->removeAndDestroyChild(node->getName());

    delete box;

    boxes_.erase(it);
  }
}

void SelectionHandler::onSelect(const Picked& obj)
{
  ROS_DEBUG("Selected 0x%08x", obj.handle);

  V_AABB aabbs;
  getAABBs(obj, aabbs);

  if (!aabbs.empty())
  {
    Ogre::AxisAlignedBox combined;
    V_AABB::iterator it = aabbs.begin();
    V_AABB::iterator end = aabbs.end();
    for (; it != end; ++it)
    {
      combined.merge(*it);
    }

    createBox(std::make_pair(obj.handle, 0ULL), combined, "RVIZ/Cyan");
  }
}

void SelectionHandler::onDeselect(const Picked& obj)
{
  ROS_DEBUG("Deselected 0x%08x", obj.handle);

  destroyBox(std::make_pair(obj.handle, 0ULL));
}

SelectionManager::SelectionManager(VisualizationManager* manager)
: vis_manager_(manager)
, highlight_enabled_(false)
, uid_counter_(0)
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
  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    std::stringstream ss;
    static int count = 0;
    ss << "SelectionTexture" << count++;

    render_textures_[i] = Ogre::TextureManager::getSingleton().createManual(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, s_render_texture_size_, s_render_texture_size_, 0, Ogre::PF_R8G8B8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* render_texture = render_textures_[i]->getBuffer()->getRenderTarget();
    render_texture->setAutoUpdated(false);

#if defined(PICKING_DEBUG)
    Ogre::Rectangle2D* mini_screen = new Ogre::Rectangle2D(true);
    mini_screen->setCorners(0.0, i, 1.0, -1.0 + (i));
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    mini_screen->setBoundingBox(aabInf);
    Ogre::SceneNode* mini_screen_node = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode(ss.str() + "MiniScreenNode");
    mini_screen_node->attachObject(mini_screen);
    debug_nodes_[i] = mini_screen_node;

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(ss.str() + "RttMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Technique *technique = material->createTechnique();
    technique->createPass();
    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material->getTechnique(0)->getPass(0)->createTextureUnitState(render_textures_[i]->getName());

    mini_screen->setMaterial(material->getName());
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

void SelectionManager::addObject(CollObjectHandle obj, const SelectionHandlerPtr& handler)
{
  if (!obj)
  {
//    ROS_BREAK();
    return;
  }

  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  handler->initialize(vis_manager_);

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

void SelectionManager::unpackColors(Ogre::Viewport* pick_viewport, Ogre::Viewport* render_viewport, const Ogre::PixelBox& box, int x1, int y1, int x2, int y2, V_Pixel& pixels)
{
  float width = pick_viewport->getActualWidth();
  float height = pick_viewport->getActualHeight();
  int render_width = render_viewport->getActualWidth();
  int render_height = render_viewport->getActualHeight();



  float fracw = render_width / width;
  float frach = render_height / height;
  x1 = x1 * fracw;
  x2 = x2 * fracw;
  y1 = y1 * frach;
  y2 = y2 * frach;

  x1 = x1 < 0 ? 0 : (x1 > render_width ? render_width : x1);
  y1 = y1 < 0 ? 0 : (y1 > render_height ? render_height : y1);
  x2 = x2 < 0 ? 0 : (x2 > render_width ? render_width : x2);
  y2 = y2 < 0 ? 0 : (y2 > render_height ? render_height : y2);

  int step_x = (x2 - x1) >= 0 ? 1 : -1;
  int step_y = (y2 - y1) >= 0 ? 1 : -1;
  pixels.resize((abs(x2 - x1) + 1) * (abs(y2 - y1) + 1));
  int i = 0;
  for (int y = y1; y != (y2 + step_y); y += step_y)
  {
    for (int x = x1; x != (x2 + step_x); x += step_x)
    {
      uint32_t pos = (x + y*render_width) * 4;

      uint32_t pix_val = *(uint32_t*)((uint8_t*)box.data + pos);
      uint32_t handle = colorToHandle(box.format, pix_val);

      Pixel& p = pixels[i];
      p.x = x;
      p.y = y;
      p.handle = handle;

      ++i;
    }
  }
}

void SelectionManager::renderAndUnpack(Ogre::Viewport* viewport, uint32_t pass, int x1, int y1, int x2, int y2, V_Pixel& pixels)
{
  ROS_ASSERT(pass < s_num_render_textures_);

  Ogre::TexturePtr tex = render_textures_[pass];
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
  Ogre::RenderTexture* render_texture = pixel_buffer->getRenderTarget();

  if (render_texture->getNumViewports() == 0 || render_texture->getViewport(0)->getCamera() != viewport->getCamera())
  {
    render_texture->removeAllViewports();
    render_texture->addViewport(viewport->getCamera());
    Ogre::Viewport* render_viewport = render_texture->getViewport(0);
    render_viewport->setClearEveryFrame(true);
    render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
    render_viewport->setOverlaysEnabled(false);

    std::stringstream scheme;
    scheme << "Pick";
    if (pass > 0)
    {
      scheme << pass;
    }

    render_viewport->setMaterialScheme(scheme.str());
  }

  render_texture->update();

  Ogre::Viewport* render_viewport = pixel_buffer->getRenderTarget()->getViewport(0);
  int render_width = render_viewport->getActualWidth();
  int render_height = render_viewport->getActualHeight();

  Ogre::PixelFormat format = pixel_buffer->getFormat();

  int size = Ogre::PixelUtil::getMemorySize(render_width, render_height, 1, format);
  uint8_t* data = new uint8_t[size];

  Ogre::PixelBox& box = pixel_boxes_[pass];
  delete [] (uint8_t*)box.data;
  box = Ogre::PixelBox(render_width, render_height, 1, format, data);

  pixel_buffer->blitToMemory(box);

  unpackColors(viewport, render_viewport, box, x1, y1, x2, y2, pixels);
}

void SelectionManager::pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, M_Picked& results)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

#if defined(PICKING_DEBUG)
  for (int i = 0; i < s_num_render_textures_; ++i)
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
        if (results.insert(std::make_pair(handle, Picked(handle))).second)
        {
          if (handler->needsAdditionalRenderPass(1))
          {
            need_additional.insert(handle);
            need_additional_render = true;
          }
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

#if defined(PICKING_DEBUG)
  for (int i = 0; i < s_num_render_textures_; ++i)
  {
    debug_nodes_[i]->setVisible(true);
  }
#endif
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
    technique = material->createTechnique();
    technique->setSchemeName("Pick");
    Ogre::Pass* pass = technique->createPass();
    pass->setLightingEnabled(false);
    pass->setSceneBlending(Ogre::SBT_REPLACE);

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
    ogre_tools::CameraBase* camera = vis_manager_->getCurrentCamera();
    camera->lookAt(center);
  }
}

} // namespace rviz
