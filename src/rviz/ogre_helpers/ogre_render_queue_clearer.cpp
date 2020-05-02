/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "ogre_render_queue_clearer.h"

#include <OgreRoot.h>
#include <OgrePass.h>

namespace rviz
{
OgreRenderQueueClearer::OgreRenderQueueClearer()
{
  // TODO Auto-generated constructor stub
}

OgreRenderQueueClearer::~OgreRenderQueueClearer()
{
  // TODO Auto-generated destructor stub
}


bool OgreRenderQueueClearer::frameStarted(const Ogre::FrameEvent& /*evt*/)
{
  // Workaround taken from http://www.ogre3d.org/mantis/view.php?id=130
  // in case a plugin creates its own scene manager.
  //
  // Has the following modifications:
  // 1. Need to pass 'true' to RenderQueue::clear( bool destroyPassMaps ).
  // 2. Don't need to call Ogre::Pass::processPendingPassUpdates(),
  //    since this is done within RenderQueue::clear().

  // This workaround is only necessary if there is more than one
  // scene manager present, so check for that
  Ogre::SceneManagerEnumerator::SceneManagerIterator it =
      Ogre::Root::getSingletonPtr()->getSceneManagerIterator();
  it.getNext();
  if (!it.hasMoreElements())
  {
    return true;
  }

  it = Ogre::Root::getSingletonPtr()->getSceneManagerIterator();
  while (it.hasMoreElements())
  {
    it.getNext()->getRenderQueue()->clear(true);
  }
  return true;
}

} // namespace rviz
