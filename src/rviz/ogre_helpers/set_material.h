/*
 * Copyright (c) 2018, Fizyr BV
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

#ifndef OGRE_TOOLS_SET_MATERIAL_H
#define OGRE_TOOLS_SET_MATERIAL_H

#include <OgreSimpleRenderable.h>
#include <OgreMaterialManager.h>

#include <string>

namespace rviz {

/*
 * This header allows setting the material of a renderable by either name or MaterialPtr
 * across different versions of OGRE.
 *
 * OGRE 1.10 added:   renderable.setMaterial(const Ogre::MaterialPtr &)
 * OGRE 1.11 removed: renderable.setMaterial(const std::string       &)
 */

#if OGRE_VERSION_MAJOR < 1 || (OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR < 10)
inline void setMaterial(Ogre::SimpleRenderable & renderable, const std::string & material_name)
{
  renderable.setMaterial(material_name);
}

inline void setMaterial(Ogre::SimpleRenderable & renderable, const Ogre::MaterialPtr & material)
{
  renderable.setMaterial(material->getName());
}
#else
inline void setMaterial(Ogre::SimpleRenderable & renderable, const std::string & material_name)
{
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(material_name);
  // OGRE 1.11 also deprecated their own SharedPtr class and switched to std::shared_ptr.
  // Checking for nullptr with .get() works in both versions.
  if (!material.get())
  {
    OGRE_EXCEPT(Ogre::Exception::ERR_ITEM_NOT_FOUND, "Could not find material " + material_name, "SimpleRenderable::setMaterial");
  }
  renderable.setMaterial(material);
}

inline void setMaterial(Ogre::SimpleRenderable & renderable, const Ogre::MaterialPtr & material)
{
  renderable.setMaterial(material);
}
#endif

}

#endif
