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

#ifndef RVIZ_SELECTION_MANAGER_H
#define RVIZ_SELECTION_MANAGER_H

#include <QObject>

#include "forwards.h"
#include "selection_handler.h"
#include "rviz/properties/forwards.h"

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRenderQueueListener.h>

#include <vector>
#include <set>

namespace rviz
{
class Object;
}

namespace Ogre
{
class SceneManager;
class Viewport;
class WireBoundingBox;
class SceneNode;
class Material;
class PixelBox;
class Rectangle2D;
class MovableObject;
}

namespace rviz
{
class ViewportMouseEvent;
class VisualizationManager;
class PropertyManager;

class SelectionManager: public QObject, public Ogre::MaterialManager::Listener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  enum SelectType
  {
    Add,
    Remove,
    Replace
  };

  SelectionManager(VisualizationManager* manager);
  ~SelectionManager();

  void initialize( bool debug = false );

  void clearHandlers();
  void addObject(CollObjectHandle obj, const SelectionHandlerPtr& handler);
  void removeObject(CollObjectHandle obj);

  // control the highlight box being displayed while selecting
  void highlight(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);
  void removeHighlight();

  // select all objects in bounding box
  void select(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, SelectType type);

  // @return handles of all objects in the given bounding box
  // @param single_render_pass only perform one rendering pass (point cloud selecting won't work)
  void pick(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2, M_Picked& results, bool single_render_pass=false );

  // create handle, add or modify the picking scheme of the object's material accordingly
  CollObjectHandle createCollisionForObject(Object* obj, const SelectionHandlerPtr& handler, CollObjectHandle coll = 0);
  CollObjectHandle createCollisionForEntity(Ogre::Entity* entity, const SelectionHandlerPtr& handler, CollObjectHandle coll = 0);

  void update();

  // modify the list of currently selected objects
  void setSelection(const M_Picked& objs);
  void addSelection(const M_Picked& objs);
  void removeSelection(const M_Picked& objs);
  const M_Picked& getSelection() { return selection_; }

  SelectionHandlerPtr getHandler(CollObjectHandle obj);

  // modify the given material so it contains a technique for the picking scheme that uses the given handle
  Ogre::Technique *addPickTechnique(CollObjectHandle handle, const Ogre::MaterialPtr& material);

  // if a material does not support the picking scheme, paint it black
  virtual Ogre::Technique* handleSchemeNotFound(unsigned short scheme_index,
      const Ogre::String& scheme_name,
      Ogre::Material* original_material,
      unsigned short lod_index,
      const Ogre::Renderable* rend);

  // create a new unique handle
  CollObjectHandle createHandle();

  // tell all handlers that interactive mode is active/inactive
  void enableInteraction( bool enable );
  bool getInteractionEnabled() { return interaction_enabled_; }

  // tell the view controller to look at the selection
  void focusOnSelection();

  // change the size of the off-screen selection buffer texture
  void setTextureSize( unsigned size );

  /** Return true if the point at x, y in the viewport is showing an
   * object, false otherwise.  If it is showing an object, result will
   * be changed to contain the 3D point corresponding to it. */
  bool get3DPoint( Ogre::Viewport* viewport, int x, int y, Ogre::Vector3& result_point );

  // Implementation for Ogre::RenderQueueListener.
  void renderQueueStarted( uint8_t queueGroupId,
                           const std::string& invocation, 
                           bool& skipThisInvocation );

  /** @brief Get the viewport currently being rendered to. Used by
   * recipients of pre- and post-render callbacks to determine where
   * the rendering is happening; returns NULL when called not between
   * those callbacks. */
  Ogre::Viewport* getCurrentViewport() { return current_viewport_; }

Q_SIGNALS:
  void selectionSet( const M_Picked& old_selection, const M_Picked& new_selection );
  void selectionSetting();
  void selectionAdded( const M_Picked& added );
  void selectionRemoved( const M_Picked& removed );

protected:
  std::pair<Picked, bool> addSelection(const Picked& obj);
  void removeSelection(const Picked& obj);

  void setHighlightRect(Ogre::Viewport* viewport, int x1, int y1, int x2, int y2);

  /** Render to a texture for one of the picking passes and unpack the resulting pixels. */
  void renderAndUnpack(Ogre::Viewport* viewport, uint32_t pass, int x1, int y1, int x2, int y2, V_Pixel& pixels);

  /** Internal render function to render to a texture and read the pixels back out. */
  bool render( Ogre::Viewport* viewport, Ogre::TexturePtr tex,
               int x1, int y1, int x2, int y2,
               Ogre::PixelBox& dst_box, std::string material_scheme,
               unsigned texture_size );

  void unpackColors(const Ogre::PixelBox& box, V_Pixel& pixels);

  void initDepthFinder();

  // Set the visibility of the debug windows.  If debug_mode_ is false, this has no effect.
  void setDebugVisibility( bool visible );

  VisualizationManager* vis_manager_;

  boost::recursive_mutex global_mutex_;

  typedef boost::unordered_map<CollObjectHandle, SelectionHandlerPtr> M_CollisionObjectToSelectionHandler;
  M_CollisionObjectToSelectionHandler objects_;

  bool highlight_enabled_;

  struct Highlight
  {
    int x1;
    int y1;
    int x2;
    int y2;
    Ogre::Viewport* viewport;
  };
  Highlight highlight_;

  M_Picked selection_;

  const static uint32_t s_num_render_textures_ = 2; // If you want to change this number to something > 3 you must provide more width for extra handles in the Picked structure (currently a u64)
  Ogre::TexturePtr render_textures_[s_num_render_textures_];
  Ogre::PixelBox pixel_boxes_[s_num_render_textures_];

  // Graphics card -based depth finding of clicked points.
  Ogre::TexturePtr depth_render_texture_;
  uint32_t depth_texture_size_;
  Ogre::SceneNode* debug_depth_node_;
  Ogre::MaterialPtr debug_depth_material_;
  Ogre::PixelBox depth_pixel_box_;

  uint32_t uid_counter_;

  Ogre::Rectangle2D* highlight_rectangle_;
  Ogre::SceneNode* highlight_node_;
  Ogre::Camera *camera_;

  V_Pixel pixel_buffer_;

  bool interaction_enabled_;

  Ogre::SceneNode* debug_nodes_[s_num_render_textures_];
  Ogre::MaterialPtr debug_material_[s_num_render_textures_];
  bool debug_mode_;

  Ogre::MaterialPtr fallback_pick_material_;
  Ogre::Technique *fallback_pick_technique_;

  uint32_t texture_size_;

  // The viewport currently being rendered to. Used by recepients of pre- and post-render callbacks
  // to determine where the rendering is happening; set to NULL outside of those callbacks.
  Ogre::Viewport *current_viewport_;
};

} // namespace rviz

#endif // RVIZ_SELECTION_MANAGER_H
