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

#ifndef OGRE_TOOLS_OGRE_POINT_CLOUD_H
#define OGRE_TOOLS_OGRE_POINT_CLOUD_H

#include <OgreSimpleRenderable.h>
#include <OgreMovableObject.h>
#include <OgreString.h>
#include <OgreAxisAlignedBox.h>
#include <OgreVector3.h>
#include <OgreMaterial.h>
#include <OgreColourValue.h>
#include <OgreRoot.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSharedPtr.h>

#include <stdint.h>

#include <vector>

#include <boost/shared_ptr.hpp>

namespace Ogre
{
class SceneManager;
class ManualObject;
class SceneNode;
class RenderQueue;
class Camera;
class RenderSystem;
class Matrix4;
} // namespace Ogre

namespace rviz
{
class PointCloud;
class PointCloudRenderable : public Ogre::SimpleRenderable
{
public:
  PointCloudRenderable(PointCloud* parent, int num_points, bool use_tex_coords);
  ~PointCloudRenderable() override;

  Ogre::RenderOperation* getRenderOperation()
  {
    return &mRenderOp;
  }

  Ogre::HardwareVertexBufferSharedPtr getBuffer();

  Ogre::Real getBoundingRadius() const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const override;
  void _notifyCurrentCamera(Ogre::Camera* camera) override;
  unsigned short getNumWorldTransforms() const override
  {
    return 1;
  }
  void getWorldTransforms(Ogre::Matrix4* xform) const override;
  const Ogre::LightList& getLights() const override;

private:
  Ogre::MaterialPtr material_;
  PointCloud* parent_;
};
typedef boost::shared_ptr<PointCloudRenderable> PointCloudRenderablePtr;
typedef std::vector<PointCloudRenderablePtr> V_PointCloudRenderable;

/**
 * \class PointCloud
 * \brief A visual representation of a set of points.
 *
 * Displays a set of points using any number of Ogre BillboardSets.  PointCloud is optimized for sets of
 * points that change
 * rapidly, rather than for large clouds that never change.
 *
 * Most of the functions in PointCloud are not safe to call from any thread but the render thread.
 * Exceptions are clear() and addPoints(), which
 * are safe as long as we are not in the middle of a render (ie. Ogre::Root::renderOneFrame, or
 * Ogre::RenderWindow::update)
 */
class PointCloud : public Ogre::MovableObject
{
public:
  enum RenderMode
  {
    RM_POINTS,
    RM_SQUARES,
    RM_FLAT_SQUARES,
    RM_SPHERES,
    RM_TILES,
    RM_BOXES,
  };

  PointCloud();
  ~PointCloud() override;

  /**
   * \brief Clear all the points
   */
  void clear();

  /**
   * \struct Point
   * \brief Representation of a point, with x/y/z position and r/g/b color
   */
  struct Point
  {
    inline void setColor(float r, float g, float b, float a = 1.0)
    {
      color = Ogre::ColourValue(r, g, b, a);
    }

    Ogre::Vector3 position;
    Ogre::ColourValue color;
  };

  /**
   * \brief Add points to this point cloud
   *
   * @param points An array of Point structures
   * @param num_points The number of points in the array
   */
  void addPoints(Point* points, uint32_t num_points);

  /**
   * \brief Remove a number of points from this point cloud
   * \param num_points The number of points to pop
   */
  void popPoints(uint32_t num_points);

  /**
   * \brief Set what type of rendering primitives should be used, currently points, billboards and boxes
   * are supported
   */
  void setRenderMode(RenderMode mode);
  /**
   * \brief Set the dimensions of the billboards used to render each point
   * @param width Width
   * @param height Height
   * @note width/height are only applicable to billboards and boxes, depth is only applicable to boxes
   */
  void setDimensions(float width, float height, float depth);

  /*
   * If set to true, the size of each point will be multiplied by it z component.
   * (Used for depth image based point clouds)
   */
  void setAutoSize(bool auto_size);

  /// See Ogre::BillboardSet::setCommonDirection
  void setCommonDirection(const Ogre::Vector3& vec);
  /// See Ogre::BillboardSet::setCommonUpVector
  void setCommonUpVector(const Ogre::Vector3& vec);

  /// set alpha blending
  /// @param alpha global alpha value
  /// @param per_point_alpha indicates that each point will have an individual alpha value.
  ///                        if true, enables alpha blending regardless of the global alpha.
  void setAlpha(float alpha, bool per_point_alpha = false);

  void setPickColor(const Ogre::ColourValue& color);
  void setColorByIndex(bool set);

  void setHighlightColor(float r, float g, float b);

  const Ogre::String& getMovableType() const override
  {
    return sm_Type;
  }
  const Ogre::AxisAlignedBox& getBoundingBox() const override;
  float getBoundingRadius() const override;
  virtual void getWorldTransforms(Ogre::Matrix4* xform) const;
  virtual unsigned short getNumWorldTransforms() const
  {
    return 1;
  }
  void _updateRenderQueue(Ogre::RenderQueue* queue) override;
  void _notifyCurrentCamera(Ogre::Camera* camera) override;
  void _notifyAttached(Ogre::Node* parent, bool isTagPoint = false) override;
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
  void visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables) override;
#endif

  virtual void setName(const std::string& name)
  {
    mName = name;
  }

private:
  uint32_t getVerticesPerPoint();
  PointCloudRenderablePtr createRenderable(int num_points);
  void regenerateAll();
  void shrinkRenderables();

  Ogre::AxisAlignedBox bounding_box_; ///< The bounding box of this point cloud
  float bounding_radius_;             ///< The bounding radius of this point cloud

  typedef std::vector<Point> V_Point;
  V_Point points_;       ///< The list of points we're displaying.  Allocates to a high-water-mark.
  uint32_t point_count_; ///< The number of points currently in #points_

  RenderMode render_mode_;
  float width_;                    ///< width
  float height_;                   ///< height
  float depth_;                    ///< depth
  Ogre::Vector3 common_direction_; ///< See Ogre::BillboardSet::setCommonDirection
  Ogre::Vector3 common_up_vector_; ///< See Ogre::BillboardSet::setCommonUpVector

  Ogre::MaterialPtr point_material_;
  Ogre::MaterialPtr square_material_;
  Ogre::MaterialPtr flat_square_material_;
  Ogre::MaterialPtr sphere_material_;
  Ogre::MaterialPtr tile_material_;
  Ogre::MaterialPtr box_material_;
  Ogre::MaterialPtr current_material_;
  float alpha_;

  bool color_by_index_;

  V_PointCloudRenderable renderables_;

  bool current_mode_supports_geometry_shader_;
  Ogre::ColourValue pick_color_;

  static Ogre::String sm_Type; ///< The "renderable type" used by Ogre
};

} // namespace rviz

#endif
