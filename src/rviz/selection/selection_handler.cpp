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

#include "selection_handler.h"

#include <rviz/properties/property.h>
#include <rviz/visualization_manager.h>

#include <ros/assert.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreWireBoundingBox.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>

#include <rviz/selection/selection_manager.h>
#include <rviz/ogre_helpers/compatibility.h>

#include <utility>

namespace rviz
{
SelectionHandler::SelectionHandler(DisplayContext* context)
  : context_(context), listener_(new Listener(this))
{
  pick_handle_ = context_->getSelectionManager()->createHandle();
  context_->getSelectionManager()->addObject(pick_handle_, this);
}

SelectionHandler::~SelectionHandler()
{
  S_Movable::iterator it = tracked_objects_.begin();
  S_Movable::iterator end = tracked_objects_.end();
  for (; it != end; ++it)
  {
    Ogre::MovableObject* m = *it;
    m->setListener(nullptr);
  }

  while (!boxes_.empty())
  {
    destroyBox(boxes_.begin()->first);
  }
  context_->getSelectionManager()->removeObject(pick_handle_);
}

void SelectionHandler::preRenderPass(uint32_t /*pass*/)
{
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it)
  {
    Ogre::WireBoundingBox* box = it->second.second;
    box->setVisible(false);
  }
}

void SelectionHandler::postRenderPass(uint32_t /*pass*/)
{
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it)
  {
    Ogre::WireBoundingBox* box = it->second.second;
    box->setVisible(true);
  }
}


void SelectionHandler::addTrackedObjects(Ogre::SceneNode* node)
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
    addTrackedObject(obj);
  }
  // Loop over and recurse into all child nodes.
  Ogre::SceneNode::ChildNodeIterator child_it = node->getChildIterator();
  while (child_it.hasMoreElements())
  {
    Ogre::SceneNode* child = dynamic_cast<Ogre::SceneNode*>(child_it.getNext());
    addTrackedObjects(child);
  }
}

void SelectionHandler::addTrackedObject(Ogre::MovableObject* object)
{
  tracked_objects_.insert(object);
  object->setListener(listener_.get());

  SelectionManager::setPickHandle(pick_handle_, object);
}

void SelectionHandler::removeTrackedObject(Ogre::MovableObject* object)
{
  tracked_objects_.erase(object);
  object->setListener(nullptr);

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

void SelectionHandler::getAABBs(const Picked& /*obj*/, V_AABB& aabbs)
{
  S_Movable::iterator it = tracked_objects_.begin();
  S_Movable::iterator end = tracked_objects_.end();
  for (; it != end; ++it)
  {
    aabbs.push_back((*it)->getWorldBoundingBox());
  }
}

void SelectionHandler::destroyProperties(const Picked& /*obj*/, Property* /*parent_property*/)
{
  for (int i = 0; i < properties_.size(); i++)
  {
    delete properties_.at(i);
  }
  properties_.clear();
}

void SelectionHandler::createBox(const std::pair<CollObjectHandle, uint64_t>& handles,
                                 const Ogre::AxisAlignedBox& aabb,
                                 const std::string& material_name)
{
  Ogre::WireBoundingBox* box = nullptr;
  Ogre::SceneNode* node = nullptr;

  M_HandleToBox::iterator it = boxes_.find(handles);
  if (it == boxes_.end())
  {
    Ogre::SceneManager* scene_manager = context_->getSceneManager();
    node = scene_manager->getRootSceneNode()->createChildSceneNode();
    box = new Ogre::WireBoundingBox;

    bool inserted = boxes_.insert(std::make_pair(handles, std::make_pair(node, box))).second;
    ROS_ASSERT(inserted);
    Q_UNUSED(inserted);
  }
  else
  {
    node = it->second.first;
    box = it->second.second;
  }

  setMaterial(*box, material_name);

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
    removeAndDestroyChildNode(node->getParentSceneNode(), node);

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

void SelectionHandler::setInteractiveObject(InteractiveObjectWPtr object)
{
  interactive_object_ = std::move(object);
}

InteractiveObjectWPtr SelectionHandler::getInteractiveObject()
{
  return interactive_object_;
}

} // namespace rviz
