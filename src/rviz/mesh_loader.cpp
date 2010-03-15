/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "mesh_loader.h"
#include <resource_retriever/retriever.h>

#include <boost/filesystem.hpp>

#include <ogre_tools/stl_loader.h>

#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreMeshSerializer.h>

#include <ros/console.h>

namespace fs = boost::filesystem;

namespace rviz
{

Ogre::MeshPtr loadMeshFromResource(const std::string& resource_path)
{
  if (Ogre::MeshManager::getSingleton().resourceExists(resource_path))
  {
    return Ogre::MeshManager::getSingleton().getByName(resource_path);
  }
  else
  {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever.get(resource_path);
    }
    catch (resource_retriever::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      return Ogre::MeshPtr();
    }

    if (res.size == 0)
    {
      return Ogre::MeshPtr();
    }

    fs::path model_path(resource_path);
    std::string ext = model_path.extension();
    if (ext == ".stl" || ext == ".STL" || ext == ".stlb" || ext == ".STLB")
    {
      ogre_tools::STLLoader loader;
      if (!loader.load(res.data.get()))
      {
        ROS_ERROR("Failed to load file [%s]", resource_path.c_str());
        return Ogre::MeshPtr();
      }

      return loader.toMesh(resource_path);
    }
    else if (ext == ".mesh" || ext == ".MESH")
    {
      Ogre::MeshSerializer ser;
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(resource_path, "rviz");
      ser.importMesh(stream, mesh.get());

      return mesh;
    }
    else
    {
      ROS_ERROR("Unsupported mesh type [%s]", ext.c_str());
    }
  }

  return Ogre::MeshPtr();
}

}
