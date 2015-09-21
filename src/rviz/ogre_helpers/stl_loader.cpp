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

#include "stl_loader.h"
#include <ros/console.h>

#include <OgreManualObject.h>

namespace ogre_tools
{

STLLoader::STLLoader()
{

}

STLLoader::~STLLoader()
{

}

bool STLLoader::load(const std::string& path)
{
  FILE* input = fopen( path.c_str(), "r" );
  if ( !input )
  {
    ROS_ERROR( "Could not open '%s' for read", path.c_str() );
    return false;
  }

  /* from wikipedia:
   * Because ASCII STL files can become very large, a binary version of STL exists. A binary STL file has an 80 character header
   * (which is generally ignored - but which should never begin with 'solid' because that will lead most software to assume that
   * this is an ASCII STL file). Following the header is a 4 byte unsigned integer indicating the number of triangular facets in
   * the file. Following that is data describing each triangle in turn. The file simply ends after the last triangle.
   *
   * Each triangle is described by twelve 32-bit-floating point numbers: three for the normal and then three for the X/Y/Z coordinate
   * of each vertex - just as with the ASCII version of STL. After the twelve floats there is a two byte unsigned 'short' integer that
   * is the 'attribute byte count' - in the standard format, this should be zero because most software does not understand anything else.
   *
   * Floating point numbers are represented as IEEE floating point numbers and the endianness is assumed to be little endian although this
   * is not stated in documentation.
   */

  // find the file size
  fseek( input, 0, SEEK_END );
  long fileSize = ftell( input );
  rewind( input );

  std::vector<uint8_t> buffer_vec(fileSize);
  uint8_t* buffer = &buffer_vec[0];

  long num_bytes_read = fread( buffer, 1, fileSize, input );
  if ( num_bytes_read != fileSize )
  {
    ROS_ERROR("STLLoader::load( \"%s\" ) only read %ld bytes out of total %ld.",
              path.c_str(), num_bytes_read, fileSize);
    fclose( input );
    return false;
  }
  fclose( input );

  return this->load(buffer, num_bytes_read, path);
}

bool STLLoader::load(uint8_t* buffer, const size_t num_bytes, const std::string& origin)
{
  // check for ascii since we can only load binary types with this class
  std::string buffer_str = std::string(reinterpret_cast<char *>(buffer), num_bytes);

  if (buffer_str.substr(0, 5) == std::string("solid"))
  {
    // file says that it is ascii, but why should we trust it?

    // check for "endsolid" as well
    if (buffer_str.find("endsolid", 5) != std::string::npos)
    {
      ROS_ERROR_STREAM("The STL file '" << origin << "' is malformed. It "
                       "starts with the word 'solid' and also contains the "
                       "word 'endsolid', indicating that it's an ASCII STL "
                       "file, but rviz can only load binary STL files so it "
                       "will not be loaded. Please convert it to a "
                       "binary STL file.");
      return false;
    }

    // chastise the user for malformed files
    ROS_WARN_STREAM("The STL file '" << origin << "' is malformed. It starts"
                    " with the word 'solid', indicating that it's an ASCII "
                    "STL file, but it does not contain the word 'endsolid' so"
                    "it is either a malformed ASCII STL file or it is actually "
                    "a binary STL file. Trying to interpret it as a binary "
                    "STL file instead.");
  }

  // make sure there's enough data for a binary STL header and triangle count
  static const size_t binary_stl_header_len = 84;
  if (num_bytes <= binary_stl_header_len)
  {
    ROS_ERROR_STREAM("The STL file '" << origin <<"' is malformed. It "
                     "appears to be a binary STL file but does not contain "
                     "enough data for the 80 byte header and 32-bit integer "
                     "triangle count.");
    return false;
  }

  // one last check to make sure that the size matches the number of triangles
  unsigned int num_triangles = *(reinterpret_cast<uint32_t *>(buffer + 80));
  static const size_t number_of_bytes_per_triangle = 50;
  size_t expected_size = binary_stl_header_len + num_triangles * number_of_bytes_per_triangle;
  if (num_bytes != expected_size)
  {
    ROS_ERROR_STREAM("The STL file '" << origin << "' is malformed. According "
                     "to the binary STL header it should have '" <<
                     num_triangles << "' triangles, but it has too " <<
                     (num_bytes > expected_size ? "much" : "little") <<
                     " data for that to be the case.");
    return false;
  }

  // load the binary STL data
  return this->load_binary(buffer);
}

bool STLLoader::load_binary(uint8_t* buffer)
{
  uint8_t* pos = buffer;

  pos += 80; // skip the 80 byte header

  unsigned int numTriangles = *(unsigned int*)pos;
  pos += 4;

  for ( unsigned int currentTriangle = 0; currentTriangle < numTriangles; ++currentTriangle )
  {
    Triangle tri;

    tri.normal_.x = *(float*)pos;
    pos += 4;
    tri.normal_.y = *(float*)pos;
    pos += 4;
    tri.normal_.z = *(float*)pos;
    pos += 4;

    tri.vertices_[0].x = *(float*)pos;
    pos += 4;
    tri.vertices_[0].y = *(float*)pos;
    pos += 4;
    tri.vertices_[0].z = *(float*)pos;
    pos += 4;

    tri.vertices_[1].x = *(float*)pos;
    pos += 4;
    tri.vertices_[1].y = *(float*)pos;
    pos += 4;
    tri.vertices_[1].z = *(float*)pos;
    pos += 4;

    tri.vertices_[2].x = *(float*)pos;
    pos += 4;
    tri.vertices_[2].y = *(float*)pos;
    pos += 4;
    tri.vertices_[2].z = *(float*)pos;
    pos += 4;

    // Blender was writing a large number into this short... am I misinterpreting what the attribute byte count is supposed to do?
    //unsigned short attributeByteCount = *(unsigned short*)pos;
    pos += 2;

    //pos += attributeByteCount;

    if (tri.normal_.squaredLength() < 0.001)
    {
      Ogre::Vector3 side1 = tri.vertices_[0] - tri.vertices_[1];
      Ogre::Vector3 side2 = tri.vertices_[1] - tri.vertices_[2];
      tri.normal_ = side1.crossProduct(side2);
    }
    tri.normal_.normalise();

    triangles_.push_back(tri);
  }

  return true;
}

void calculateUV(const Ogre::Vector3& vec, float& u, float& v)
{
  Ogre::Vector3 pos(vec);
  pos.normalise();
  u = acos( pos.y / pos.length() );

  float val = pos.x / ( sin( u ) );
  v = acos( val );

  u /= Ogre::Math::PI;
  v /= Ogre::Math::PI;
}


Ogre::MeshPtr STLLoader::toMesh(const std::string& name)
{
  Ogre::ManualObject* object = new Ogre::ManualObject( "the one and only" );
  object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST );

  unsigned int vertexCount = 0;
  V_Triangle::const_iterator it = triangles_.begin();
  V_Triangle::const_iterator end = triangles_.end();
  for (; it != end; ++it )
  {
    if( vertexCount >= 2004 )
    {
      // Subdivide large meshes into submeshes with at most 2004
      // vertices to prevent problems on some graphics cards.
      object->end();
      object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST );
      vertexCount = 0;
    }

    const STLLoader::Triangle& tri = *it;

    float u, v;
    u = v = 0.0f;
    object->position( tri.vertices_[0] );
    object->normal( tri.normal_);
    calculateUV( tri.vertices_[0], u, v );
    object->textureCoord( u, v );

    object->position( tri.vertices_[1] );
    object->normal( tri.normal_);
    calculateUV( tri.vertices_[1], u, v );
    object->textureCoord( u, v );

    object->position( tri.vertices_[2] );
    object->normal( tri.normal_);
    calculateUV( tri.vertices_[2], u, v );
    object->textureCoord( u, v );

    object->triangle( vertexCount + 0, vertexCount + 1, vertexCount + 2 );

    vertexCount += 3;
  }

  object->end();

  Ogre::MeshPtr mesh = object->convertToMesh( name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  mesh->buildEdgeList();

  delete object;

  return mesh;
}

}
