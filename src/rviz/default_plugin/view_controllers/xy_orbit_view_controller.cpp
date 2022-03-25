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

#include <stdint.h>

#include <ros/ros.h>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreQuaternion.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/viewport_mouse_event.h>

#include <rviz/default_plugin/view_controllers/xy_orbit_view_controller.h>

namespace rviz
{
// move camera up so the focal point appears in the lower image half
static const float CAMERA_OFFSET = 0.2;

void XYOrbitViewController::onInitialize()
{
  OrbitViewController::onInitialize();
  focal_shape_->setColor(0.0f, 1.0f, 1.0f, 0.5f);
}

bool XYOrbitViewController::intersectGroundPlane(Ogre::Ray mouse_ray, Ogre::Vector3& intersection_3d)
{
  // convert rays into reference frame
  mouse_ray.setOrigin(target_scene_node_->convertWorldToLocalPosition(mouse_ray.getOrigin()));
  mouse_ray.setDirection(target_scene_node_->convertWorldToLocalOrientation(Ogre::Quaternion::IDENTITY) *
                         mouse_ray.getDirection());

  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0);

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
  if (!intersection.first)
  {
    return false;
  }

  intersection_3d = mouse_ray.getPoint(intersection.second);
  return true;
}

void XYOrbitViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if (event.shift())
  {
    setStatus("<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.");
  }
  else
  {
    setStatus("<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z. "
              " <b>Shift</b>: More options.");
  }

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  bool moved = false;
  if (event.type == QEvent::MouseButtonPress)
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
  }
  else if (event.type == QEvent::MouseButtonRelease)
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
  }
  else if (event.type == QEvent::MouseMove)
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  if (event.left() && !event.shift())
  {
    setCursor(Rotate3D);
    yaw(diff_x * 0.005);
    pitch(-diff_y * 0.005);
  }
  else if (event.middle() || (event.left() && event.shift()))
  {
    setCursor(MoveXY);
    // handle mouse movement
    int width = event.viewport->getActualWidth();
    int height = event.viewport->getActualHeight();

    Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(event.x / (float)width,
                                                                              event.y / (float)height);

    Ogre::Ray last_mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
        event.last_x / (float)width, event.last_y / (float)height);

    Ogre::Vector3 last_intersect, intersect;

    if (intersectGroundPlane(last_mouse_ray, last_intersect) &&
        intersectGroundPlane(mouse_ray, intersect))
    {
      Ogre::Vector3 motion = last_intersect - intersect;

      // When dragging near the horizon, the motion can get out of
      // control.  This throttles it to an arbitrary limit per mouse
      // event.
      float motion_distance_limit = 1; /*meter*/
      if (motion.length() > motion_distance_limit)
      {
        motion.normalise();
        motion *= motion_distance_limit;
      }

      focal_point_property_->add(motion);
      emitConfigChanged();
    }
  }
  else if (event.right())
  {
    setCursor(Zoom);
    zoom(-diff_y * 0.1 * (distance_property_->getFloat() / 10.0f));
  }
  else
  {
    setCursor(event.shift() ? MoveXY : Rotate3D);
  }

  if (event.wheel_delta != 0)
  {
    int diff = event.wheel_delta;
    zoom(diff * 0.001 * distance_property_->getFloat());
    moved = true;
  }

  if (moved)
  {
    context_->queueRender();
  }
}

void XYOrbitViewController::mimic(ViewController* source_view)
{
  FramePositionTrackingViewController::mimic(source_view);

  Ogre::Camera* source_camera = source_view->getCamera();
  // do some trigonometry
  Ogre::Ray camera_dir_ray(source_camera->getRealPosition(), source_camera->getRealDirection());
  Ogre::Ray camera_down_ray(source_camera->getRealPosition(), -1.0 * source_camera->getRealUp());

  Ogre::Vector3 a, b;

  if (intersectGroundPlane(camera_dir_ray, b) && intersectGroundPlane(camera_down_ray, a))
  {
    float l_a = source_camera->getPosition().distance(b);
    float l_b = source_camera->getPosition().distance(a);

    distance_property_->setFloat((l_a * l_b) / (CAMERA_OFFSET * l_a + l_b));
    float distance = distance_property_->getFloat();

    camera_dir_ray.setOrigin(source_camera->getRealPosition() -
                             source_camera->getRealUp() * distance * CAMERA_OFFSET);
    Ogre::Vector3 new_focal_point;
    intersectGroundPlane(camera_dir_ray, new_focal_point);
    focal_point_property_->setVector(new_focal_point);

    calculatePitchYawFromPosition(source_camera->getPosition() -
                                  source_camera->getUp() * distance * CAMERA_OFFSET);
  }
}

void XYOrbitViewController::updateCamera()
{
  OrbitViewController::updateCamera();
  camera_->setPosition(camera_->getPosition() +
                       camera_->getUp() * distance_property_->getFloat() * CAMERA_OFFSET);
}

void XYOrbitViewController::lookAt(const Ogre::Vector3& point)
{
  Ogre::Vector3 camera_position = camera_->getPosition();
  Ogre::Vector3 new_focal_point =
      target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition());
  new_focal_point.z = 0;
  distance_property_->setFloat(new_focal_point.distance(camera_position));
  focal_point_property_->setVector(new_focal_point);

  calculatePitchYawFromPosition(camera_position);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::XYOrbitViewController, rviz::ViewController)
