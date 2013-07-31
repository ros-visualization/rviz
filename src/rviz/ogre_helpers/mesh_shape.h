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

#ifndef OGRE_TOOLS_MESH_SHAPE_H
#define OGRE_TOOLS_MESH_SHAPE_H

#include "shape.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

/**
 */
class MeshShape : public Shape
{
public:

  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   */
  MeshShape(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = NULL);
  virtual ~MeshShape();

  /* \brief Estimate the number of vertices ahead of time. */
  void estimateVertexCount(size_t vcount);
  
  /** \brief Start adding triangles to the mesh */
  void beginTriangles();

  /** \brief Add a vertex to the mesh (no normal defined). If using
      this function it is assumed that triangles are added by
      specifying the 3 vertices in order (3 consecutive calls to this
      function). This means there must be 3*n calls to this function
      to add n triangles. */
  void addVertex(const Ogre::Vector3& position);

  /** \brief Add a vertex to the mesh with a normal defined. If using
      this function it is assumed that triangles are added by
      specifying the 3 vertices in order (3 consecutive calls to this
      function). This means there must be 3*n calls to this function
      to add n triangles. */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal);
  
  /** \brief Add the vertices for a triangle. Normal for triangle is computed using cross product and the next version of addTriangle() is called. */
  void addTriangle(const Ogre::Vector3 &p1, const Ogre::Vector3 &p2, const Ogre::Vector3 &p3);

  /** \brief Add the vertices for a triangle. Normal for triangle is specified (same at each vertex). This makes 3 calls to addVertex() */
  void addTriangle(const Ogre::Vector3 &p1, const Ogre::Vector3 &p2, const Ogre::Vector3 &p3, const Ogre::Vector3 &normal);

  /** \brief Add the vertices for a triangle. Normal for triangle is specified (same at each vertex). This makes 3 calls to addVertex() */
  void addTriangle(const Ogre::Vector3 &p1, const Ogre::Vector3 &p2, const Ogre::Vector3 &p3,
                   const Ogre::Vector3 &n1, const Ogre::Vector3 &n2, const Ogre::Vector3 &n3);
  
  /** \brief Notify that the set of triangles to add is complete. No more triangles can be added, beginTriangles() can no longer be called. */
  void endTriangles();

private:

  bool started_;
  Ogre::ManualObject *manual_object_;
  
};

} // namespace rviz

#endif

