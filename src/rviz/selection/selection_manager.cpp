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

#include <algorithm>

#include <QTimer>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderSystem.h>
#include <OgreRenderTexture.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreWireBoundingBox.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreRectangle2D.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <ros/assert.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/custom_parameter_indices.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/ogre_helpers/qt_ogre_render_window.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

#include <rviz/selection/selection_manager.h>
#include <vector>


namespace rviz
{
SelectionManager::SelectionManager(VisualizationManager* manager)
  : vis_manager_(manager)
  , highlight_enabled_(false)
  , uid_counter_(0)
  , interaction_enabled_(false)
  , debug_mode_(false)
  , property_model_(new PropertyTreeModel(new Property("root")))
{
  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    pixel_boxes_[i].data = nullptr;
  }
  depth_pixel_box_.data = nullptr;

  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateProperties()));
  timer->start(200);
}

SelectionManager::~SelectionManager()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  setSelection(M_Picked());

  removeAndDestroyChildNode(highlight_node_->getParentSceneNode(), highlight_node_);
  delete highlight_rectangle_;

  for (uint32_t i = 0; i < s_num_render_textures_; ++i)
  {
    delete[](uint8_t*) pixel_boxes_[i].data;
  }
  delete[](uint8_t*) depth_pixel_box_.data;

  vis_manager_->getSceneManager()->destroyCamera(camera_);

  delete property_model_;
}

void SelectionManager::setDebugMode(bool debug)
{
  debug_mode_ = debug;
}

void SelectionManager::initialize()
{
  // Create our render textures
  setTextureSize(1);

  // Create our highlight rectangle
  Ogre::SceneManager* scene_manager = vis_manager_->getSceneManager();
  highlight_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();

  std::stringstream ss;
  static int count = 0;
  ss << "SelectionRect" << count++;
  highlight_rectangle_ = new Ogre::Rectangle2D(true);

  const static uint32_t texture_data[1] = {0xffff0080};
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream((void*)&texture_data[0], 4));

  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(
      ss.str() + "Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1,
      Ogre::PF_R8G8B8A8, Ogre::TEX_TYPE_2D, 0);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  // material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
  setMaterial(*highlight_rectangle_, material);
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  highlight_rectangle_->setBoundingBox(aabInf);
  highlight_rectangle_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setCullingMode(Ogre::CULL_NONE);

  Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tex_unit->setTextureName(tex->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  highlight_node_->attachObject(highlight_rectangle_);

  // create picking camera
  camera_ = scene_manager->createCamera(ss.str() + "_camera");

  // create fallback picking material
  fallback_pick_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/DefaultPickAndDepth");
  fallback_pick_material_->load();

  fallback_pick_cull_technique_ = fallback_pick_material_->getTechnique("PickCull");
  fallback_black_cull_technique_ = fallback_pick_material_->getTechnique("BlackCull");
  fallback_depth_cull_technique_ = fallback_pick_material_->getTechnique("DepthCull");

  fallback_pick_technique_ = fallback_pick_material_->getTechnique("Pick");
  fallback_black_technique_ = fallback_pick_material_->getTechnique("Black");
  fallback_depth_technique_ = fallback_pick_material_->getTechnique("Depth");
}


bool SelectionManager::get3DPoint(Ogre::Viewport* viewport, int x, int y, Ogre::Vector3& result_point)
{
  ROS_DEBUG("SelectionManager.get3DPoint()");

  std::vector<Ogre::Vector3> result_points_temp;
  bool success = get3DPatch(viewport, x, y, 1, 1, true, result_points_temp);
  if (result_points_temp.empty())
  {
    // return result_point unmodified if get point fails.
    return false;
  }
  result_point = result_points_temp[0];

  return success;
}


bool SelectionManager::getPatchDepthImage(Ogre::Viewport* viewport,
                                          int x,
                                          int y,
                                          unsigned width,
                                          unsigned height,
                                          std::vector<float>& depth_vector)
{
  unsigned int num_pixels = width * height;
  depth_vector.reserve(num_pixels);

  setDepthTextureSize(width, height);


  M_CollisionObjectToSelectionHandler::iterator handler_it = objects_.begin();
  M_CollisionObjectToSelectionHandler::iterator handler_end = objects_.end();

  for (; handler_it != handler_end; ++handler_it)
  {
    handler_it->second->preRenderPass(0);
  }

  if (render(viewport, depth_render_texture_, x, y, x + width, y + height, depth_pixel_box_, "Depth",
             depth_texture_width_, depth_texture_height_))
  {
    uint8_t* data_ptr = (uint8_t*)depth_pixel_box_.data;

    for (uint32_t pixel = 0; pixel < num_pixels; ++pixel)
    {
      uint8_t a = data_ptr[4 * pixel];
      uint8_t b = data_ptr[4 * pixel + 1];
      uint8_t c = data_ptr[4 * pixel + 2];

      int int_depth = (c << 16) | (b << 8) | a;
      float normalized_depth = ((float)int_depth) / (float)0xffffff;
      depth_vector.push_back(normalized_depth * camera_->getFarClipDistance());
    }
  }
  else
  {
    ROS_WARN("Failed to render depth patch\n");
    return false;
  }

  handler_it = objects_.begin();
  handler_end = objects_.end();
  for (; handler_it != handler_end; ++handler_it)
  {
    handler_it->second->postRenderPass(0);
  }

  return true;
}


bool SelectionManager::get3DPatch(Ogre::Viewport* viewport,
                                  int x,
                                  int y,
                                  unsigned width,
                                  unsigned height,
                                  bool skip_missing,
                                  std::vector<Ogre::Vector3>& result_points)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);
  ROS_DEBUG("SelectionManager.get3DPatch()");

  std::vector<float> depth_vector;


  if (!getPatchDepthImage(viewport, x, y, width, height, depth_vector))
    return false;


  unsigned int pixel_counter = 0;
  Ogre::Matrix4 projection = camera_->getProjectionMatrix();
  float depth;

  for (unsigned y_iter = 0; y_iter < height; ++y_iter)
    for (unsigned x_iter = 0; x_iter < width; ++x_iter)
    {
      depth = depth_vector[pixel_counter];

      // Deal with missing or invalid points
      if ((depth > camera_->getFarClipDistance()) || (depth == 0))
      {
        ++pixel_counter;
        if (!skip_missing)
        {
          result_points.push_back(Ogre::Vector3(NAN, NAN, NAN));
        }
        continue;
      }


      Ogre::Vector3 result_point;
      // We want to shoot rays through the center of pixels, not the corners,
      // so add .5 pixels to the x and y coordinate to get to the center
      // instead of the top left of the pixel.
      Ogre::Real screenx = float(x_iter + .5) / float(width);
      Ogre::Real screeny = float(y_iter + .5) / float(height);
      if (projection[3][3] == 0.0) // If this is a perspective projection
      {
        // get world-space ray from camera & mouse coord
        Ogre::Ray vp_ray = camera_->getCameraToViewportRay(screenx, screeny);

        // transform ray direction back into camera coords
        Ogre::Vector3 dir_cam = camera_->getDerivedOrientation().Inverse() * vp_ray.getDirection();

        // normalize, so dir_cam.z == -depth
        dir_cam = dir_cam / dir_cam.z * depth * -1;

        // compute 3d point from camera origin and direction*/
        result_point = camera_->getDerivedPosition() + camera_->getDerivedOrientation() * dir_cam;
      }
      else // else this must be an orthographic projection.
      {
        // For orthographic projection, getCameraToViewportRay() does
        // the right thing for us, and the above math does not work.
        Ogre::Ray ray;
        camera_->getCameraToViewportRay(screenx, screeny, &ray);

        result_point = ray.getPoint(depth);
      }

      result_points.push_back(result_point);
      ++pixel_counter;
    }

  return !result_points.empty();
}


void SelectionManager::setDepthTextureSize(unsigned width, unsigned height)
{
  // Cap and store requested texture size
  // It's probably an error if an invalid size is requested.
  if (width > 1024)
  {
    width = 1024;
    ROS_ERROR_STREAM("SelectionManager::setDepthTextureSize invalid width requested. Max Width: 1024 -- "
                     "Width requested: "
                     << width << ".  Capping Width at 1024.");
  }

  if (depth_texture_width_ != width)
    depth_texture_width_ = width;

  if (height > 1024)
  {
    height = 1024;
    ROS_ERROR_STREAM("SelectionManager::setDepthTextureSize invalid height requested. Max Height: 1024 "
                     "-- Height requested: "
                     << width << ".  Capping Height at 1024.");
  }

  if (depth_texture_height_ != height)
    depth_texture_height_ = height;

  if (!depth_render_texture_.get() || depth_render_texture_->getWidth() != width ||
      depth_render_texture_->getHeight() != height)
  {
    std::string tex_name = "DepthTexture";
    if (depth_render_texture_.get())
    {
      tex_name = depth_render_texture_->getName();

      // destroy old
      Ogre::TextureManager::getSingleton().remove(tex_name);
    }

    depth_render_texture_ = Ogre::TextureManager::getSingleton().createManual(
        tex_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        depth_texture_width_, depth_texture_height_, 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

    Ogre::RenderTexture* render_texture = depth_render_texture_->getBuffer()->getRenderTarget();
    render_texture->setAutoUpdated(false);
  }
}


void SelectionManager::setTextureSize(unsigned size)
{
  if (size > 1024)
  {
    size = 1024;
  }

  texture_size_ = size;

  for (uint32_t pass = 0; pass < s_num_render_textures_; ++pass)
  {
    // check if we need to change the texture size
    if (!render_textures_[pass].get() || render_textures_[pass]->getWidth() != size)
    {
      std::string tex_name;
      if (render_textures_[pass].get())
      {
        tex_name = render_textures_[pass]->getName();

        // destroy old
        Ogre::TextureManager::getSingleton().remove(tex_name);
      }
      else
      {
        std::stringstream ss;
        static int count = 0;
        ss << "SelectionTexture" << count++;
        tex_name = ss.str();
      }

      // create new texture
      render_textures_[pass] = Ogre::TextureManager::getSingleton().createManual(
          tex_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, size,
          size, 0, Ogre::PF_R8G8B8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);

      Ogre::RenderTexture* render_texture = render_textures_[pass]->getBuffer()->getRenderTarget();
      render_texture->setAutoUpdated(false);
    }
  }
}

void SelectionManager::clearHandlers()
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  objects_.clear();
}

void SelectionManager::enableInteraction(bool enable)
{
  interaction_enabled_ = enable;
  M_CollisionObjectToSelectionHandler::iterator handler_it = objects_.begin();
  M_CollisionObjectToSelectionHandler::iterator handler_end = objects_.end();
  for (; handler_it != handler_end; ++handler_it)
  {
    if (InteractiveObjectPtr object = handler_it->second->getInteractiveObject().lock())
    {
      object->enableInteraction(enable);
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
  for (unsigned int i = 0; i < 24; i++)
  {
    uint32_t shift = (((23 - i) % 3) * 8) + (23 - i) / 3;
    uint32_t bit = ((uint32_t)(uid_counter_ >> i) & (uint32_t)1) << shift;
    handle |= bit;
  }

  return handle;
}

void SelectionManager::addObject(CollObjectHandle obj, SelectionHandler* handler)
{
  if (!obj)
  {
    //    ROS_BREAK();
    return;
  }

  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  InteractiveObjectPtr object = handler->getInteractiveObject().lock();
  if (object)
  {
    object->enableInteraction(interaction_enabled_);
  }

  bool inserted = objects_.insert(std::make_pair(obj, handler)).second;
  ROS_ASSERT(inserted);
  Q_UNUSED(inserted);
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

void SelectionManager::unpackColors(Ogre::PixelBox& box, V_CollObject& pixels)
{
  int w = box.getWidth();
  int h = box.getHeight();

  pixels.clear();
  pixels.reserve(w * h);
  size_t size = Ogre::PixelUtil::getMemorySize(1, 1, 1, box.format);

  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      uint32_t pos = (x + y * w) * size;
      uint32_t pix_val = 0;
      memcpy((uint8_t*)&pix_val, (uint8_t*)box.data + pos, size);
      pixels.push_back(colorToHandle(box.format, pix_val));
    }
  }
}

void SelectionManager::renderAndUnpack(Ogre::Viewport* viewport,
                                       uint32_t pass,
                                       int x1,
                                       int y1,
                                       int x2,
                                       int y2,
                                       V_CollObject& pixels)
{
  ROS_ASSERT(pass < s_num_render_textures_);

  std::stringstream scheme;
  scheme << "Pick";
  if (pass > 0)
  {
    scheme << pass;
  }

  if (render(viewport, render_textures_[pass], x1, y1, x2, y2, pixel_boxes_[pass], scheme.str(),
             texture_size_, texture_size_))
  {
    unpackColors(pixel_boxes_[pass], pixels);
  }
}


bool SelectionManager::render(Ogre::Viewport* viewport,
                              const Ogre::TexturePtr& tex,
                              int x1,
                              int y1,
                              int x2,
                              int y2,
                              Ogre::PixelBox& dst_box,
                              const std::string& material_scheme,
                              unsigned texture_width,
                              unsigned texture_height)
{
  vis_manager_->lockRender();

  if (x1 > x2)
    std::swap(x1, x2);
  if (y1 > y2)
    std::swap(y1, y2);

  if (x1 < 0)
    x1 = 0;
  if (y1 < 0)
    y1 = 0;
  if (x1 > viewport->getActualWidth() - 2)
    x1 = viewport->getActualWidth() - 2;
  if (y1 > viewport->getActualHeight() - 2)
    y1 = viewport->getActualHeight() - 2;
  if (x2 < 0)
    x2 = 0;
  if (y2 < 0)
    y2 = 0;
  if (x2 > viewport->getActualWidth() - 2)
    x2 = viewport->getActualWidth() - 2;
  if (y2 > viewport->getActualHeight() - 2)
    y2 = viewport->getActualHeight() - 2;

  if (x2 == x1)
    x2++;
  if (y2 == y1)
    y2++;

  if (x2 == x1 || y2 == y1)
  {
    ROS_WARN("SelectionManager::render(): not rendering 0 size area.");
    vis_manager_->unlockRender();
    return false;
  }

  unsigned w = x2 - x1;
  unsigned h = y2 - y1;

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
  Ogre::RenderTexture* render_texture = pixel_buffer->getRenderTarget();

  Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  Ogre::Matrix4 scale_matrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 trans_matrix = Ogre::Matrix4::IDENTITY;

  float x1_rel = static_cast<float>(x1) / static_cast<float>(viewport->getActualWidth() - 1) - 0.5f;
  float y1_rel = static_cast<float>(y1) / static_cast<float>(viewport->getActualHeight() - 1) - 0.5f;
  float x2_rel = static_cast<float>(x2) / static_cast<float>(viewport->getActualWidth() - 1) - 0.5f;
  float y2_rel = static_cast<float>(y2) / static_cast<float>(viewport->getActualHeight() - 1) - 0.5f;

  scale_matrix[0][0] = 1.0 / (x2_rel - x1_rel);
  scale_matrix[1][1] = 1.0 / (y2_rel - y1_rel);

  trans_matrix[0][3] -= x1_rel + x2_rel;
  trans_matrix[1][3] += y1_rel + y2_rel;

  camera_->setCustomProjectionMatrix(true, scale_matrix * trans_matrix * proj_matrix);

  camera_->setPosition(viewport->getCamera()->getDerivedPosition());
  camera_->setOrientation(viewport->getCamera()->getDerivedOrientation());

  // create a viewport if there is none
  if (render_texture->getNumViewports() == 0)
  {
    render_texture->removeAllViewports();
    render_texture->addViewport(camera_);
    Ogre::Viewport* render_viewport = render_texture->getViewport(0);
    render_viewport->setClearEveryFrame(true);
    render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
    render_viewport->setOverlaysEnabled(false);
    render_viewport->setMaterialScheme(material_scheme);
  }

  unsigned render_w = w;
  unsigned render_h = h;

  if (w > h)
  {
    if (render_w > texture_width)
    {
      render_w = texture_width;
      render_h = round(float(h) * (float)texture_width / (float)w);
    }
  }
  else
  {
    if (render_h > texture_height)
    {
      render_h = texture_height;
      render_w = round(float(w) * (float)texture_height / (float)h);
    }
  }

  // safety clamping in case of rounding errors
  if (render_w > texture_width)
    render_w = texture_width;
  if (render_h > texture_height)
    render_h = texture_height;

  // set viewport to render to a subwindow of the texture
  Ogre::Viewport* render_viewport = render_texture->getViewport(0);
  render_viewport->setDimensions(0, 0, (float)render_w / (float)texture_width,
                                 (float)render_h / (float)texture_height);

  // make sure the same objects are visible as in the original viewport
  render_viewport->setVisibilityMask(viewport->getVisibilityMask());

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
  Q_UNUSED(d);

  Ogre::MaterialManager::getSingleton().removeListener(this);

  render_w = render_viewport->getActualWidth();
  render_h = render_viewport->getActualHeight();

  Ogre::PixelFormat format = pixel_buffer->getFormat();

  int size = Ogre::PixelUtil::getMemorySize(render_w, render_h, 1, format);
  uint8_t* data = new uint8_t[size];

  delete[](uint8_t*) dst_box.data;
  dst_box = Ogre::PixelBox(render_w, render_h, 1, format, data);

  pixel_buffer->blitToMemory(dst_box, dst_box);

  vis_manager_->unlockRender();

  if (debug_mode_)
  {
    publishDebugImage(dst_box, material_scheme);
  }

  return true;
}

void SelectionManager::publishDebugImage(const Ogre::PixelBox& pixel_box, const std::string& label)
{
  ros::Publisher pub;
  ros::NodeHandle nh;
  PublisherMap::const_iterator iter = debug_publishers_.find(label);
  if (iter == debug_publishers_.end())
  {
    pub = nh.advertise<sensor_msgs::Image>("/rviz_debug/" + label, 2);
    debug_publishers_[label] = pub;
  }
  else
  {
    pub = iter->second;
  }

  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();
  msg.width = pixel_box.getWidth();
  msg.height = pixel_box.getHeight();
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  msg.is_bigendian = false;
  msg.step = msg.width * 3;
  int dest_byte_count = msg.width * msg.height * 3;
  msg.data.resize(dest_byte_count);
  int dest_index = 0;
  uint8_t* source_ptr = (uint8_t*)pixel_box.data;
  int pre_pixel_padding = 0;
  int post_pixel_padding = 0;
  switch (pixel_box.format)
  {
  case Ogre::PF_R8G8B8:
    break;
  case Ogre::PF_A8R8G8B8:
  case Ogre::PF_X8R8G8B8:
    post_pixel_padding = 1;
    break;
  case Ogre::PF_R8G8B8A8:
    pre_pixel_padding = 1;
    break;
  default:
    ROS_ERROR("SelectionManager::publishDebugImage(): Incompatible pixel format [%d]", pixel_box.format);
    return;
  }
  uint8_t r, g, b;
  while (dest_index < dest_byte_count)
  {
    source_ptr += pre_pixel_padding;
    b = *source_ptr++;
    g = *source_ptr++;
    r = *source_ptr++;
    source_ptr += post_pixel_padding;
    msg.data[dest_index++] = r;
    msg.data[dest_index++] = g;
    msg.data[dest_index++] = b;
  }

  pub.publish(msg);
}

void SelectionManager::renderQueueStarted(uint8_t /*queueGroupId*/,
                                          const std::string& /*invocation*/,
                                          bool& skipThisInvocation)
{
  // This render queue listener function tells the scene manager to
  // skip every render step, so nothing actually gets drawn.

  //  ROS_DEBUG("SelectionManager renderQueueStarted(%d, '%s') returning skip = true.",
  //  (int)queueGroupId, invocation.c_str());
  skipThisInvocation = true;
}

void SelectionManager::pick(Ogre::Viewport* viewport,
                            int x1,
                            int y1,
                            int x2,
                            int y2,
                            M_Picked& results,
                            bool single_render_pass)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  bool need_additional_render = false;

  V_CollObject handles_by_pixel;
  S_CollObject need_additional;

  V_CollObject& pixels = pixel_buffer_;

  // First render is special... does the initial object picking, determines which objects have been
  // selected
  // After that, individual handlers can specify that they need additional renders (max # defined in
  // s_num_render_textures_)
  {
    M_CollisionObjectToSelectionHandler::iterator handler_it = objects_.begin();
    M_CollisionObjectToSelectionHandler::iterator handler_end = objects_.end();
    for (; handler_it != handler_end; ++handler_it)
    {
      handler_it->second->preRenderPass(0);
    }

    renderAndUnpack(viewport, 0, x1, y1, x2, y2, pixels);

    handler_it = objects_.begin();
    handler_end = objects_.end();
    for (; handler_it != handler_end; ++handler_it)
    {
      handler_it->second->postRenderPass(0);
    }

    handles_by_pixel.reserve(pixels.size());
    V_CollObject::iterator it = pixels.begin();
    V_CollObject::iterator end = pixels.end();
    for (; it != end; ++it)
    {
      const CollObjectHandle& p = *it;

      CollObjectHandle handle = p;

      handles_by_pixel.push_back(handle);

      if (handle == 0)
      {
        continue;
      }

      SelectionHandler* handler = getHandler(handle);

      if (handler)
      {
        std::pair<M_Picked::iterator, bool> insert_result =
            results.insert(std::make_pair(handle, Picked(handle)));
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
        SelectionHandler* handler = getHandler(*need_it);
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
        SelectionHandler* handler = getHandler(*need_it);
        ROS_ASSERT(handler);

        handler->postRenderPass(pass);
      }
    }

    int i = 0;
    V_CollObject::iterator pix_it = pixels.begin();
    V_CollObject::iterator pix_end = pixels.end();
    for (; pix_it != pix_end; ++pix_it, ++i)
    {
      const CollObjectHandle& p = *pix_it;

      CollObjectHandle handle = handles_by_pixel[i];

      if (pass == 1)
      {
        extra_by_pixel[i] = 0;
      }

      if (need_additional.find(handle) != need_additional.end())
      {
        CollObjectHandle extra_handle = p;
        extra_by_pixel[i] |= extra_handle << (32 * (pass - 1));
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
}

Ogre::Technique* SelectionManager::handleSchemeNotFound(unsigned short /*scheme_index*/,
                                                        const Ogre::String& scheme_name,
                                                        Ogre::Material* original_material,
                                                        unsigned short /*lod_index*/,
                                                        const Ogre::Renderable* rend)
{
  // Find the original culling mode
  Ogre::CullingMode culling_mode = Ogre::CULL_CLOCKWISE;
  Ogre::Technique* orig_tech = original_material->getTechnique(0);
  if (orig_tech && orig_tech->getNumPasses() > 0)
  {
    culling_mode = orig_tech->getPass(0)->getCullingMode();
  }

  // find out if the renderable has the picking param set
  bool has_pick_param = !rend->getUserObjectBindings().getUserAny("pick_handle").isEmpty();

  // NOTE: it is important to avoid changing the culling mode of the
  // fallback techniques here, because that change then propagates to
  // other uses of these fallback techniques in unpredictable ways.
  // If you want to change the technique programmatically (like with
  // Pass::setCullingMode()), make sure you do it on a cloned material
  // which doesn't get shared with other objects.

  // Use the technique with the right name and culling mode.
  if (culling_mode == Ogre::CULL_CLOCKWISE)
  {
    if (scheme_name == "Pick")
    {
      return has_pick_param ? fallback_pick_cull_technique_ : fallback_black_cull_technique_;
    }
    else if (scheme_name == "Depth")
    {
      return fallback_depth_cull_technique_;
    }
    if (scheme_name == "Pick1")
    {
      return fallback_black_cull_technique_;
    }
    else
    {
      return nullptr;
    }
  }
  else // Must be CULL_NONE because we never use CULL_ANTICLOCKWISE
  {
    if (scheme_name == "Pick")
    {
      return has_pick_param ? fallback_pick_technique_ : fallback_black_technique_;
    }
    else if (scheme_name == "Depth")
    {
      return fallback_depth_technique_;
    }
    if (scheme_name == "Pick1")
    {
      return fallback_black_technique_;
    }
    else
    {
      return nullptr;
    }
  }
}

Ogre::ColourValue SelectionManager::handleToColor(CollObjectHandle handle)
{
  float r = ((handle >> 16) & 0xff) / 255.0f;
  float g = ((handle >> 8) & 0xff) / 255.0f;
  float b = (handle & 0xff) / 255.0f;
  return Ogre::ColourValue(r, g, b, 1.0f);
}

void SelectionManager::setPickData(CollObjectHandle handle,
                                   const Ogre::ColourValue& color,
                                   Ogre::SceneNode* node)
{
  if (!node)
  {
    return;
  }
  // Loop over all objects attached to this node.
  Ogre::SceneNode::ObjectIterator obj_it = node->getAttachedObjectIterator();
  while (obj_it.hasMoreElements())
  {
    Ogre::MovableObject* obj = obj_it.getNext();
    setPickData(handle, color, obj);
  }
  // Loop over and recurse into all child nodes.
  Ogre::SceneNode::ChildNodeIterator child_it = node->getChildIterator();
  while (child_it.hasMoreElements())
  {
    Ogre::SceneNode* child = dynamic_cast<Ogre::SceneNode*>(child_it.getNext());
    setPickData(handle, color, child);
  }
}

class PickColorSetter : public Ogre::Renderable::Visitor
{
public:
  PickColorSetter(CollObjectHandle handle, const Ogre::ColourValue& color)
    : color_vector_(color.r, color.g, color.b, 1.0), handle_(handle)
  {
  }

  void visit(Ogre::Renderable* rend,
             ushort /*lodIndex*/,
             bool /*isDebug*/,
             Ogre::Any* /*pAny*/ = nullptr) override
  {
    rend->setCustomParameter(PICK_COLOR_PARAMETER, color_vector_);
    rend->getUserObjectBindings().setUserAny("pick_handle", Ogre::Any(handle_));
  }

  Ogre::Vector4 color_vector_;
  CollObjectHandle handle_;
};

void SelectionManager::setPickData(CollObjectHandle handle,
                                   const Ogre::ColourValue& color,
                                   Ogre::MovableObject* object)
{
  PickColorSetter visitor(handle, color);
  object->visitRenderables(&visitor);
  object->getUserObjectBindings().setUserAny("pick_handle", Ogre::Any(handle));
}

SelectionHandler* SelectionManager::getHandler(CollObjectHandle obj)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_CollisionObjectToSelectionHandler::iterator it = objects_.find(obj);
  if (it != objects_.end())
  {
    return it->second;
  }

  return nullptr;
}

void SelectionManager::removeSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked::const_iterator it = objs.begin();
  M_Picked::const_iterator end = objs.end();
  for (; it != end; ++it)
  {
    removeSelectedObject(it->second);
  }

  selectionRemoved(objs);
}

void SelectionManager::addSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked added;
  M_Picked::const_iterator it = objs.begin();
  M_Picked::const_iterator end = objs.end();
  for (; it != end; ++it)
  {
    std::pair<Picked, bool> ppb = addSelectedObject(it->second);
    if (ppb.second)
    {
      added.insert(std::make_pair(it->first, ppb.first));
    }
  }

  selectionAdded(added);
}

void SelectionManager::setSelection(const M_Picked& objs)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  M_Picked original(selection_.begin(), selection_.end());

  removeSelection(original);
  addSelection(objs);
}

std::pair<Picked, bool> SelectionManager::addSelectedObject(const Picked& obj)
{
  boost::recursive_mutex::scoped_lock lock(global_mutex_);

  std::pair<M_Picked::iterator, bool> pib = selection_.insert(std::make_pair(obj.handle, obj));

  SelectionHandler* handler = getHandler(obj.handle);

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

void SelectionManager::removeSelectedObject(const Picked& obj)
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

  SelectionHandler* handler = getHandler(obj.handle);
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

    SelectionHandler* handler = getHandler(p.handle);

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
    ViewController* controller = vis_manager_->getViewManager()->getCurrent();
    if (controller)
    {
      controller->lookAt(center);
    }
  }
}

void SelectionManager::selectionRemoved(const M_Picked& removed)
{
  M_Picked::const_iterator it = removed.begin();
  M_Picked::const_iterator end = removed.end();
  for (; it != end; ++it)
  {
    const Picked& picked = it->second;
    SelectionHandler* handler = getHandler(picked.handle);
    ROS_ASSERT(handler);

    handler->destroyProperties(picked, property_model_->getRoot());
  }
}

void SelectionManager::selectionAdded(const M_Picked& added)
{
  M_Picked::const_iterator it = added.begin();
  M_Picked::const_iterator end = added.end();
  for (; it != end; ++it)
  {
    const Picked& picked = it->second;
    SelectionHandler* handler = getHandler(picked.handle);
    ROS_ASSERT(handler);

    handler->createProperties(picked, property_model_->getRoot());
  }
  property_model_->sort(0, Qt::AscendingOrder);
}

void SelectionManager::updateProperties()
{
  M_Picked::const_iterator it = selection_.begin();
  M_Picked::const_iterator end = selection_.end();
  for (; it != end; ++it)
  {
    CollObjectHandle handle = it->first;
    SelectionHandler* handler = getHandler(handle);

    handler->updateProperties();
  }
}


} // namespace rviz
