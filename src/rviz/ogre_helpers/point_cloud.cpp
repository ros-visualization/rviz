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

#include "point_cloud.h"
#include <ros/assert.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreBillboardSet.h>
#include <OGRE/OgreBillboard.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>

#include <sstream>

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)
#define SIZE_PARAMETER 0
#define ALPHA_PARAMETER 1
#define PICK_COLOR_PARAMETER 2
#define NORMAL_PARAMETER 3
#define UP_PARAMETER 4
#define HIGHLIGHT_PARAMETER 5

namespace rviz
{

static float g_point_vertices[3] =
{
  0.0f, 0.0f, 0.0f
};

static float g_billboard_vertices[6*3] =
{
  -0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, -0.5f, 0.0f,
};

static float g_billboard_sphere_vertices[3*3] =
{
  0.0f, 1.5f, 0.0f,
  -1.5f, -1.5f, 0.0f,
  1.5f, -1.5f, 0.0f,
};

static float g_box_vertices[6*6*3] =
{
  // front
  -0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,

  // back
  -0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  0.5f, -0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,

  // right
  0.5, 0.5, 0.5,
  0.5, 0.5, -0.5,
  0.5, -0.5, 0.5,
  0.5, 0.5, -0.5,
  0.5, -0.5, -0.5,
  0.5, -0.5, 0.5,

  // left
  -0.5, 0.5, 0.5,
  -0.5, -0.5, 0.5,
  -0.5, 0.5, -0.5,
  -0.5, 0.5, -0.5,
  -0.5, -0.5, 0.5,
  -0.5, -0.5, -0.5,

  // top
  -0.5, 0.5, -0.5,
  0.5, 0.5, -0.5,
  -0.5, 0.5, 0.5,
  0.5, 0.5, -0.5,
  0.5, 0.5, 0.5,
  -0.5, 0.5, 0.5,

  // bottom
  -0.5, -0.5, -0.5,
  -0.5, -0.5, 0.5,
  0.5, -0.5, -0.5,
  0.5, -0.5, -0.5,
  -0.5, -0.5, 0.5,
  0.5, -0.5, 0.5,
};

Ogre::String PointCloud::sm_Type = "PointCloud";

PointCloud::PointCloud()
: bounding_radius_( 0.0f )
, point_count_( 0 )
, common_direction_( Ogre::Vector3::NEGATIVE_UNIT_Z )
, common_up_vector_( Ogre::Vector3::UNIT_Y )
, color_by_index_(false)
, current_mode_supports_geometry_shader_(false)
{
  std::stringstream ss;
  static int count = 0;
  ss << "PointCloudMaterial" << count++;
  point_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudPoint");
  point_material_ = point_material_->clone(ss.str() + "Point");
  billboard_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBillboard");
  billboard_material_ = billboard_material_->clone(ss.str() + "Billboard");
  billboard_sphere_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBillboardSphere");
  billboard_sphere_material_ = billboard_sphere_material_->clone(ss.str() + "BillboardSphere");
  billboard_common_facing_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBillboardCommonFacing");
  billboard_common_facing_material_ = billboard_common_facing_material_->clone(ss.str() + "BillboardCommonFacing");
  box_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");
  box_material_ = box_material_->clone(ss.str() + "Box");

  point_material_->load();
  billboard_material_->load();
  billboard_sphere_material_->load();
  billboard_common_facing_material_->load();
  box_material_->load();

  setAlpha( 1.0f );
  setRenderMode(RM_BILLBOARD_SPHERES);
  setDimensions(0.01f, 0.01f, 0.01f);

  clear();
}

PointCloud::~PointCloud()
{
  point_material_->unload();
  billboard_material_->unload();
  billboard_sphere_material_->unload();
  billboard_common_facing_material_->unload();
  box_material_->unload();
}

const Ogre::AxisAlignedBox& PointCloud::getBoundingBox() const
{
  return bounding_box_;
}

float PointCloud::getBoundingRadius() const
{
  return bounding_radius_;
}

void PointCloud::getWorldTransforms(Ogre::Matrix4* xform) const
{
  *xform = _getParentNodeFullTransform();
}

void PointCloud::clear()
{
  point_count_ = 0;
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->getRenderOperation()->vertexData->vertexStart = 0;
    (*it)->getRenderOperation()->vertexData->vertexCount = 0;
  }

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

void PointCloud::regenerateAll()
{
  if (point_count_ == 0)
  {
    return;
  }

  V_Point points;
  points.swap(points_);
  uint32_t count = point_count_;

  clear();

  addPoints(&points.front(), count);
}

void PointCloud::setColorByIndex(bool set)
{
  color_by_index_ = set;

  regenerateAll();
}

void PointCloud::setHighlightColor( float r, float g, float b )
{
  Ogre::Vector4 highlight( r, g, b, 0.0f );

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(HIGHLIGHT_PARAMETER, highlight);
  }
}

void PointCloud::setRenderMode(RenderMode mode)
{
  render_mode_ = mode;

  if (mode == RM_POINTS)
  {
    current_material_ = point_material_;
  }
  else if (mode == RM_BILLBOARDS)
  {
    current_material_ = billboard_material_;
  }
  else if (mode == RM_BILLBOARD_SPHERES)
  {
    current_material_ = billboard_sphere_material_;
  }
  else if (mode == RM_BILLBOARDS_COMMON_FACING)
  {
    current_material_ = billboard_common_facing_material_;
  }
  else if (mode == RM_BOXES)
  {
    current_material_ = box_material_;
  }

  current_material_->load();

  //ROS_INFO("Best technique [%s] [gp=%s]", current_material_->getBestTechnique()->getName().c_str(), current_material_->getBestTechnique()->getPass(0)->getGeometryProgramName().c_str());

  bool geom_support_changed = false;
  Ogre::Technique* best = current_material_->getBestTechnique();
  if (best)
  {
    if (current_material_->getBestTechnique()->getName() == "gp")
    {
      if (!current_mode_supports_geometry_shader_)
      {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = true;

      //ROS_INFO("Using geometry shader");
    }
    else
    {
      if (current_mode_supports_geometry_shader_)
      {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = false;
    }
  }
  else
  {
    geom_support_changed = true;
    current_mode_supports_geometry_shader_ = false;

    ROS_ERROR("No techniques available for material [%s]", current_material_->getName().c_str());
  }

  if (geom_support_changed)
  {
    renderables_.clear();
  }

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setMaterial(current_material_->getName());
  }

  regenerateAll();
}

void PointCloud::setDimensions(float width, float height, float depth)
{
  width_ = width;
  height_ = height;
  depth_ = depth;

  Ogre::Vector4 size(width_, height_, depth_, 0.0f);

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(SIZE_PARAMETER, size);
  }
}

void PointCloud::setCommonDirection(const Ogre::Vector3& vec)
{
  common_direction_ = vec;

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(NORMAL_PARAMETER, Ogre::Vector4(vec));
  }
}

void PointCloud::setCommonUpVector(const Ogre::Vector3& vec)
{
  common_up_vector_ = vec;

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(UP_PARAMETER, Ogre::Vector4(vec));
  }
}

void setAlphaBlending(const Ogre::MaterialPtr& mat)
{
  if (mat->getBestTechnique())
  {
    mat->getBestTechnique()->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    mat->getBestTechnique()->setDepthWriteEnabled( false );
  }
}

void setReplace(const Ogre::MaterialPtr& mat)
{
  if (mat->getBestTechnique())
  {
    mat->getBestTechnique()->setSceneBlending( Ogre::SBT_REPLACE );
    mat->getBestTechnique()->setDepthWriteEnabled( true );
  }
}

void PointCloud::setAlpha(float alpha)
{
  alpha_ = alpha;

  if ( alpha < 0.9998 )
  {
    setAlphaBlending(point_material_);
    setAlphaBlending(billboard_material_);
    setAlphaBlending(billboard_sphere_material_);
    setAlphaBlending(billboard_common_facing_material_);
    setAlphaBlending(box_material_);
  }
  else
  {
    setReplace(point_material_);
    setReplace(billboard_material_);
    setReplace(billboard_sphere_material_);
    setReplace(billboard_common_facing_material_);
    setReplace(box_material_);
  }

  Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(ALPHA_PARAMETER, alpha4);
  }
}

void PointCloud::addPoints(Point* points, uint32_t num_points)
{
  if (num_points == 0)
  {
    return;
  }

  Ogre::Root* root = Ogre::Root::getSingletonPtr();

  if ( points_.size() < point_count_ + num_points )
  {
    points_.resize( point_count_ + num_points );
  }

  Point* begin = &points_.front() + point_count_;
  memcpy( begin, points, sizeof( Point ) * num_points );

  uint32_t vpp = getVerticesPerPoint();
  Ogre::RenderOperation::OperationType op_type;
  if (current_mode_supports_geometry_shader_)
  {
    op_type = Ogre::RenderOperation::OT_POINT_LIST;
  }
  else
  {
    if (render_mode_ == RM_POINTS)
    {
      op_type = Ogre::RenderOperation::OT_POINT_LIST;
    }
    else
    {
      op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
    }
  }

  float* vertices = 0;
  if (current_mode_supports_geometry_shader_)
  {
    vertices = g_point_vertices;
  }
  else
  {
    if (render_mode_ == RM_POINTS)
    {
      vertices = g_point_vertices;
    }
    else if (render_mode_ == RM_BILLBOARDS)
    {
      vertices = g_billboard_vertices;
    }
    else if (render_mode_ == RM_BILLBOARD_SPHERES)
    {
      vertices = g_billboard_sphere_vertices;
    }
    else if (render_mode_ == RM_BILLBOARDS_COMMON_FACING)
    {
      vertices = g_billboard_vertices;
    }
    else if (render_mode_ == RM_BOXES)
    {
      vertices = g_box_vertices;
    }
  }

  PointCloudRenderablePtr rend;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  void* vdata = 0;
  Ogre::RenderOperation* op = 0;
  float* fptr = 0;

  Ogre::AxisAlignedBox aabb;
  aabb.setNull();
  uint32_t current_vertex_count = 0;
  bounding_radius_ = 0.0f;
  uint32_t vertex_size = 0;
  for (uint32_t current_point = 0; current_point < num_points; ++current_point)
  {
    while (current_vertex_count >= VERTEX_BUFFER_CAPACITY || !rend)
    {
      if (rend)
      {
        ROS_ASSERT(current_vertex_count == VERTEX_BUFFER_CAPACITY);

        op->vertexData->vertexCount = VERTEX_BUFFER_CAPACITY - op->vertexData->vertexStart;
        ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <= VERTEX_BUFFER_CAPACITY);
        vbuf->unlock();
        rend->setBoundingBox(aabb);
      }

      rend = getOrCreateRenderable();
      vbuf = rend->getBuffer();
      vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);

      op = rend->getRenderOperation();
      op->operationType = op_type;
      current_vertex_count = op->vertexData->vertexStart + op->vertexData->vertexCount;

      vertex_size = op->vertexData->vertexDeclaration->getVertexSize(0);
      uint32_t data_pos = current_vertex_count * vertex_size;
      fptr = (float*)((uint8_t*)vdata + data_pos);

      aabb.setNull();
    }

    const Point& p = points[current_point];
    float x = p.x;
    float y = p.y;
    float z = p.z;
    uint32_t color = p.color;

    if (color_by_index_)
    {
      // convert to ColourValue, so we can then convert to the rendersystem-specific color type
      color = (current_point + point_count_ + 1);
      Ogre::ColourValue c;
      c.a = 1.0f;
      c.r = ((color >> 16) & 0xff) / 255.0f;
      c.g = ((color >> 8) & 0xff) / 255.0f;
      c.b = (color & 0xff) / 255.0f;
      root->convertColourValue(c, &color);
    }

    Ogre::Vector3 pos(x, y, z);
    aabb.merge(pos);
    bounding_box_.merge( pos );
    bounding_radius_ = std::max( bounding_radius_, pos.squaredLength() );

    for (uint32_t j = 0; j < vpp; ++j, ++current_vertex_count)
    {
      *fptr++ = x;
      *fptr++ = y;
      *fptr++ = z;

      if (!current_mode_supports_geometry_shader_)
      {
        *fptr++ = vertices[(j*3)];
        *fptr++ = vertices[(j*3) + 1];
        *fptr++ = vertices[(j*3) + 2];
      }

      uint32_t* iptr = (uint32_t*)fptr;
      *iptr = color;
      ++fptr;

      ROS_ASSERT((uint8_t*)fptr <= (uint8_t*)vdata + VERTEX_BUFFER_CAPACITY * vertex_size);
    }
  }

  op->vertexData->vertexCount = current_vertex_count - op->vertexData->vertexStart;
  rend->setBoundingBox(aabb);
  ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <= VERTEX_BUFFER_CAPACITY);

  vbuf->unlock();

  point_count_ += num_points;

  shrinkRenderables();

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

void PointCloud::popPoints(uint32_t num_points)
{
  uint32_t vpp = getVerticesPerPoint();

  ROS_ASSERT(num_points <= point_count_);
  points_.erase(points_.begin(), points_.begin() + num_points);

  point_count_ -= num_points;

  // Now clear out popped points
  uint32_t popped_count = 0;
  while (popped_count < num_points * vpp)
  {
    PointCloudRenderablePtr rend = renderables_.front();
    Ogre::RenderOperation* op = rend->getRenderOperation();

    uint32_t popped = std::min((size_t)(num_points * vpp - popped_count), op->vertexData->vertexCount);
    op->vertexData->vertexStart += popped;
    op->vertexData->vertexCount -= popped;

    popped_count += popped;

    if (op->vertexData->vertexCount == 0)
    {
      renderables_.erase(renderables_.begin(), renderables_.begin() + 1);

      op->vertexData->vertexStart = 0;
      renderables_.push_back(rend);
    }
  }
  ROS_ASSERT(popped_count == num_points * vpp);

  // reset bounds
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;
  for (uint32_t i = 0; i < point_count_; ++i)
  {
    Point& p = points_[i];
    Ogre::Vector3 pos(p.x, p.y, p.z);
    bounding_box_.merge(pos);
    bounding_radius_ = std::max(bounding_radius_, pos.squaredLength());
  }

  shrinkRenderables();

  if (getParentSceneNode())
  {
    getParentSceneNode()->needUpdate();
  }
}

void PointCloud::shrinkRenderables()
{
  while (!renderables_.empty())
  {
    PointCloudRenderablePtr rend = renderables_.back();
    Ogre::RenderOperation* op = rend->getRenderOperation();
    if (op->vertexData->vertexCount == 0)
    {
      renderables_.pop_back();
    }
    else
    {
      break;
    }
  }
}

void PointCloud::_notifyCurrentCamera(Ogre::Camera* camera)
{
  MovableObject::_notifyCurrentCamera( camera );
}

void PointCloud::_updateRenderQueue(Ogre::RenderQueue* queue)
{
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    queue->addRenderable((*it).get());
  }
}

void PointCloud::_notifyAttached(Ogre::Node *parent, bool isTagPoint)
{
  MovableObject::_notifyAttached(parent, isTagPoint);
}

uint32_t PointCloud::getVerticesPerPoint()
{
  if (current_mode_supports_geometry_shader_)
  {
    return 1;
  }

  if (render_mode_ == RM_POINTS)
  {
    return 1;
  }

  if (render_mode_ == RM_BILLBOARDS)
  {
    return 6;
  }

  if (render_mode_ == RM_BILLBOARDS_COMMON_FACING)
    {
      return 6;
    }

  if (render_mode_ == RM_BILLBOARD_SPHERES)
  {
    return 3;
  }

  if (render_mode_ == RM_BOXES)
  {
    return 36;
  }

  return 1;
}

void PointCloud::setPickColor(const Ogre::ColourValue& color)
{
  pick_color_ = color;
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    (*it)->setCustomParameter(PICK_COLOR_PARAMETER, pick_col);
  }
}

PointCloudRenderablePtr PointCloud::getOrCreateRenderable()
{
  V_PointCloudRenderable::iterator it = renderables_.begin();
  V_PointCloudRenderable::iterator end = renderables_.end();
  for (; it != end; ++it)
  {
    const PointCloudRenderablePtr& rend = *it;
    Ogre::RenderOperation* op = rend->getRenderOperation();
    if (op->vertexData->vertexCount + op->vertexData->vertexStart < VERTEX_BUFFER_CAPACITY)
    {
      return rend;
    }
  }

  PointCloudRenderablePtr rend(new PointCloudRenderable(this, !current_mode_supports_geometry_shader_));
  rend->setMaterial(current_material_->getName());
  Ogre::Vector4 size(width_, height_, depth_, 0.0f);
  Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 highlight(0.0f, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);
  rend->setCustomParameter(SIZE_PARAMETER, size);
  rend->setCustomParameter(ALPHA_PARAMETER, alpha);
  rend->setCustomParameter(HIGHLIGHT_PARAMETER, highlight);
  rend->setCustomParameter(PICK_COLOR_PARAMETER, pick_col);
  rend->setCustomParameter(NORMAL_PARAMETER, Ogre::Vector4(common_direction_));
  rend->setCustomParameter(UP_PARAMETER, Ogre::Vector4(common_up_vector_));
  if (getParentSceneNode())
  {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);

  return rend;
}

#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
void PointCloud::visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables)
{

}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRenderable::PointCloudRenderable(PointCloud* parent, bool use_tex_coords)
: parent_(parent)
{
  // Initialize render operation
  mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;

  Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;

  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (use_tex_coords)
  {
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
      VERTEX_BUFFER_CAPACITY,
      Ogre::HardwareBuffer::HBU_DYNAMIC);

  // Bind buffer
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
}

PointCloudRenderable::~PointCloudRenderable()
{
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr PointCloudRenderable::getBuffer()
{
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

void PointCloudRenderable::_notifyCurrentCamera(Ogre::Camera* camera)
{
  SimpleRenderable::_notifyCurrentCamera( camera );
}

Ogre::Real PointCloudRenderable::getBoundingRadius(void) const
{
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(), mBox.getMinimum().squaredLength()));
}

Ogre::Real PointCloudRenderable::getSquaredViewDepth(const Ogre::Camera* cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
   vMin = mBox.getMinimum();
   vMax = mBox.getMaximum();
   vMid = ((vMax - vMin) * 0.5) + vMin;
   vDist = cam->getDerivedPosition() - vMid;

   return vDist.squaredLength();
}

void PointCloudRenderable::getWorldTransforms(Ogre::Matrix4* xform) const
{
   *xform = m_matWorldTransform * parent_->getParentNode()->_getFullTransform();
}

const Ogre::LightList& PointCloudRenderable::getLights() const
{
  return parent_->queryLights();
}

} // namespace rviz
