/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
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

#include "covariance_visual.h"

#include "rviz/ogre_helpers/shape.h"
#include "rviz/validate_quaternions.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreQuaternion.h>

#include <ros/console.h>

#include <sstream>

namespace rviz
{
namespace
{
double deg2rad(double degrees)
{
  return degrees * 4.0 * atan(1.0) / 180.0;
}

// Local function to force the axis to be right handed for 3D. Taken from ecl_statistics
void makeRightHanded(Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being right-handed and normalised.
  Eigen::Vector3d c0 = eigenvectors.col(0);
  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.col(1);
  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.col(2);
  c2.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc.dot(c2) < 0)
  {
    eigenvectors << c1, c0, c2;
    std::swap(eigenvalues[0], eigenvalues[1]);
  }
  else
  {
    eigenvectors << c0, c1, c2;
  }
}

// Local function to force the axis to be right handed for 2D. Based on the one from ecl_statistics
void makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being righ-handed and normalised.
  Eigen::Vector3d c0;
  c0.setZero();
  c0.head<2>() = eigenvectors.col(0);
  c0.normalize();
  Eigen::Vector3d c1;
  c1.setZero();
  c1.head<2>() = eigenvectors.col(1);
  c1.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc[2] < 0)
  {
    eigenvectors << c1.head<2>(), c0.head<2>();
    std::swap(eigenvalues[0], eigenvalues[1]);
  }
  else
  {
    eigenvectors << c0.head<2>(), c1.head<2>();
  }
}

void computeShapeScaleAndOrientation3D(const Eigen::Matrix3d& covariance,
                                       Ogre::Vector3& scale,
                                       Ogre::Quaternion& orientation)
{
  Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
  Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  // FIXME: Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info() == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(
        1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector3d::Zero(); // Setting the scale to zero will hide it on the screen
    eigenvectors = Eigen::Matrix3d::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  // Define the rotation
  orientation.FromRotationMatrix(Ogre::Matrix3( // clang-format off
      eigenvectors(0, 0), eigenvectors(0, 1), eigenvectors(0, 2),
      eigenvectors(1, 0), eigenvectors(1, 1), eigenvectors(1, 2),
      eigenvectors(2, 0), eigenvectors(2, 1), eigenvectors(2, 2))); // clang-format on

  // Define the scale. eigenvalues are the variances, so we take the sqrt to draw the standard deviation
  scale.x = 2 * std::sqrt(eigenvalues[0]);
  scale.y = 2 * std::sqrt(eigenvalues[1]);
  scale.z = 2 * std::sqrt(eigenvalues[2]);
}

enum Plane
{
  YZ_PLANE, // normal is x-axis
  XZ_PLANE, // normal is y-axis
  XY_PLANE  // normal is z-axis
};

void computeShapeScaleAndOrientation2D(const Eigen::Matrix2d& covariance,
                                       Ogre::Vector3& scale,
                                       Ogre::Quaternion& orientation,
                                       Plane plane = XY_PLANE)
{
  Eigen::Vector2d eigenvalues(Eigen::Vector2d::Identity());
  Eigen::Matrix2d eigenvectors(Eigen::Matrix2d::Zero());

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  // FIXME: Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info() == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(
        1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector2d::Zero(); // Setting the scale to zero will hide it on the screen
    eigenvectors = Eigen::Matrix2d::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  // Define the rotation and scale of the plane
  // The Eigenvalues are the variances. The scales are two times the standard
  // deviation. The scale of the missing dimension is set to zero.
  if (plane == YZ_PLANE)
  {
    orientation.FromRotationMatrix(Ogre::Matrix3(1, 0, 0, // clang-format off
                                                 0, eigenvectors(0, 0), eigenvectors(0, 1),
                                                 0, eigenvectors(1, 0), eigenvectors(1, 1))); // clang-format on

    scale.x = 0;
    scale.y = 2 * std::sqrt(eigenvalues[0]);
    scale.z = 2 * std::sqrt(eigenvalues[1]);
  }
  else if (plane == XZ_PLANE)
  {
    orientation.FromRotationMatrix(
        Ogre::Matrix3(eigenvectors(0, 0), 0, eigenvectors(0, 1), // clang-format off
                      0, 1, 0,
                      eigenvectors(1, 0), 0, eigenvectors(1, 1))); // clang-format on

    scale.x = 2 * std::sqrt(eigenvalues[0]);
    scale.y = 0;
    scale.z = 2 * std::sqrt(eigenvalues[1]);
  }
  else // plane == XY_PLANE
  {
    orientation.FromRotationMatrix(
        Ogre::Matrix3(eigenvectors(0, 0), eigenvectors(0, 1), 0, // clang-format off
                      eigenvectors(1, 0), eigenvectors(1, 1), 0,
                      0, 0, 1)); // clang-format on

    scale.x = 2 * std::sqrt(eigenvalues[0]);
    scale.y = 2 * std::sqrt(eigenvalues[1]);
    scale.z = 0;
  }
}

void radianScaleToMetricScaleBounded(Ogre::Real& radian_scale, float max_degrees)
{
  radian_scale /= 2.0;
  if (radian_scale > deg2rad(max_degrees))
    radian_scale = deg2rad(max_degrees);
  radian_scale = 2.0 * tan(radian_scale);
}


} // namespace

const float CovarianceVisual::max_degrees = 89.0;

CovarianceVisual::CovarianceVisual(Ogre::SceneManager* scene_manager,
                                   Ogre::SceneNode* parent_node,
                                   bool is_local_rotation,
                                   bool is_visible,
                                   float pos_scale,
                                   float ori_scale,
                                   float ori_offset)
  : Object(scene_manager)
  , local_rotation_(is_local_rotation)
  , pose_2d_(false)
  , orientation_visible_(is_visible)
{
  // Main node of the visual
  root_node_ = parent_node->createChildSceneNode();
  // Node that will have the same orientation as the fixed frame. Updated from the message on
  // setCovariance()
  fixed_orientation_node_ = root_node_->createChildSceneNode();
  // Node to scale the position part of the covariance from the property value
  position_scale_node_ = fixed_orientation_node_->createChildSceneNode();
  // Node to be oriented and scaled from the message's covariance
  position_node_ = position_scale_node_->createChildSceneNode();
  position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_);

  // Node to scale the orientation part of the covariance. May be attached to both the local (root) node
  // or the fixed frame node.
  // May be re-attached later by setRotatingFrame()
  if (local_rotation_)
    orientation_root_node_ = root_node_->createChildSceneNode();
  else
    orientation_root_node_ = fixed_orientation_node_->createChildSceneNode();

  for (int i = 0; i < kNumOriShapes; i++)
  {
    // Node to position and orient the shape along the axis. One for each axis.
    orientation_offset_node_[i] = orientation_root_node_->createChildSceneNode();
    // Does not inherit scale from the parent. This is needed to keep the cylinders with the same height.
    // The scale is set by setOrientationScale()
    orientation_offset_node_[i]->setInheritScale(false);

    if (i != kYaw2D)
      orientation_shape_[i] =
          new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, orientation_offset_node_[i]);
    else
      orientation_shape_[i] =
          new rviz::Shape(rviz::Shape::Cone, scene_manager_, orientation_offset_node_[i]);

    // Initialize all current scales to 0
    current_ori_scale_[i] = Ogre::Vector3(0, 0, 0);
  }

  // Position the cylindes at position 1.0 in the respective axis, and perpendicular to the axis.
  // x-axis (roll)
  orientation_offset_node_[kRoll]->setPosition(Ogre::Vector3::UNIT_X);
  orientation_offset_node_[kRoll]->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X) *
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z));
  // y-axis (pitch)
  orientation_offset_node_[kPitch]->setPosition(Ogre::Vector3(Ogre::Vector3::UNIT_Y));
  orientation_offset_node_[kPitch]->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y));
  // z-axis (yaw)
  orientation_offset_node_[kYaw]->setPosition(Ogre::Vector3(Ogre::Vector3::UNIT_Z));
  orientation_offset_node_[kYaw]->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
  // z-axis (yaw 2D)
  // NOTE: rviz use a cone defined by the file rviz/ogre_media/models/rviz_cone.mesh, and it's
  //       origin is not at the top of the cone. Since we want the top to be at the origin of
  //       the pose we need to use an offset here.
  // WARNING: This number was found by trial-and-error on rviz and it's not the correct
  //          one, so changes on scale are expected to cause the top of the cone to move
  //          from the pose origin, although it's only noticeable with big scales.
  // FIXME: Find the right value from the cone.mesh file, or implement a class that draws
  //        something like a 2D "pie slice" and use it instead of the cone.
  static const double cone_origin_to_top = 0.49115;
  orientation_offset_node_[kYaw2D]->setPosition(cone_origin_to_top * Ogre::Vector3::UNIT_X);
  orientation_offset_node_[kYaw2D]->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z));

  // set initial visibility and scale
  // root node is always visible. The visibility will be updated on its childs.
  root_node_->setVisible(true);
  setVisible(is_visible);
  setScales(pos_scale, ori_scale);
  setOrientationOffset(ori_offset);
}

CovarianceVisual::~CovarianceVisual()
{
  delete position_shape_;
  scene_manager_->destroySceneNode(position_node_->getName());

  for (int i = 0; i < kNumOriShapes; i++)
  {
    delete orientation_shape_[i];
    scene_manager_->destroySceneNode(orientation_offset_node_[i]->getName());
  }

  scene_manager_->destroySceneNode(position_scale_node_->getName());
  scene_manager_->destroySceneNode(fixed_orientation_node_->getName());
  scene_manager_->destroySceneNode(root_node_->getName());
}

void CovarianceVisual::setCovariance(const geometry_msgs::PoseWithCovariance& pose)
{
  // check for NaN in covariance
  for (unsigned i = 0; i < 3; ++i)
  {
    if (std::isnan(pose.covariance[i]))
    {
      ROS_WARN_THROTTLE(1, "covariance contains NaN");
      return;
    }
  }

  pose_2d_ = pose.covariance[14] <= 0 && pose.covariance[21] <= 0 && pose.covariance[28] <= 0;

  updateOrientationVisibility();

  // store orientation in Ogre structure
  Ogre::Quaternion ori;
  normalizeQuaternion(pose.pose.orientation, ori);

  // Set the orientation of the fixed node. Since this node is attached to the root node, it's
  // orientation will be the
  // inverse of pose's orientation.
  fixed_orientation_node_->setOrientation(ori.Inverse());
  // Map covariance to a Eigen::Matrix
  Eigen::Map<const Eigen::Matrix<double, 6, 6> > covariance(pose.covariance.data());

  updatePosition(covariance);
  if (!pose_2d_)
  {
    updateOrientation(covariance, kRoll);
    updateOrientation(covariance, kPitch);
    updateOrientation(covariance, kYaw);
  }
  else
  {
    updateOrientation(covariance, kYaw2D);
  }
}

void CovarianceVisual::updatePosition(const Eigen::Matrix6d& covariance)
{
  // Compute shape and orientation for the position part of covariance
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  if (pose_2d_)
  {
    computeShapeScaleAndOrientation2D(covariance.topLeftCorner<2, 2>(), shape_scale, shape_orientation,
                                      XY_PLANE);
    // Make the scale in z minimal for better visualization
    shape_scale.z = 0.001;
  }
  else
  {
    computeShapeScaleAndOrientation3D(covariance.topLeftCorner<3, 3>(), shape_scale, shape_orientation);
  }
  // Rotate and scale the position scene node
  position_node_->setOrientation(shape_orientation);
  if (!shape_scale.isNaN())
    position_node_->setScale(shape_scale);
  else
    ROS_WARN_STREAM("position shape_scale contains NaN: " << shape_scale);
}

void CovarianceVisual::updateOrientation(const Eigen::Matrix6d& covariance, ShapeIndex index)
{
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  // Compute shape and orientation for the orientation shape
  if (pose_2d_)
  {
    // We should only enter on this scope if the index is kYaw2D
    assert(index == kYaw2D);
    // 2D poses only depend on yaw.
    shape_scale.x = 2.0 * sqrt(covariance(5, 5));
    // To display the cone shape properly the scale along y-axis has to be one.
    shape_scale.y = 1.0;
    // Give a minimal height for the cone for better visualization
    shape_scale.z = 0.001;
    // Store the computed scale to be used if the user change the scale
    current_ori_scale_[index] = shape_scale;
    // Apply the current scale factor
    shape_scale.x *= current_ori_scale_factor_;
    // The scale on x means twice the standard deviation, but _in radians_.
    // So we need to convert it to the linear scale of the shape using tan().
    // Also, we bound the maximum std
    radianScaleToMetricScaleBounded(shape_scale.x, max_degrees);
  }
  else
  {
    assert(index != kYaw2D);

    // Get the correct sub-matrix based on the index
    Eigen::Matrix2d covarianceAxis;
    if (index == kRoll)
    {
      covarianceAxis = covariance.bottomRightCorner<2, 2>();
    }
    else if (index == kPitch)
    {
      covarianceAxis << covariance(3, 3), covariance(3, 5), covariance(5, 3), covariance(5, 5);
    }
    else if (index == kYaw)
    {
      covarianceAxis = covariance.block<2, 2>(3, 3);
    }

    // NOTE: The cylinder mesh is oriented along its y axis, we want to flat it out into the XZ plane
    computeShapeScaleAndOrientation2D(covarianceAxis, shape_scale, shape_orientation, XZ_PLANE);
    // Give a minimal height for the cylinder for better visualization
    shape_scale.y = 0.001;
    // Store the computed scale to be used if the user change the scale
    current_ori_scale_[index] = shape_scale;
    // Apply the current scale factor
    shape_scale.x *= current_ori_scale_factor_;
    shape_scale.z *= current_ori_scale_factor_;
    // The computed scale is equivalent to twice the standard deviation _in radians_.
    // So we need to convert it to the linear scale of the shape using tan().
    // Also, we bound the maximum std.
    radianScaleToMetricScaleBounded(shape_scale.x, max_degrees);
    radianScaleToMetricScaleBounded(shape_scale.z, max_degrees);
  }

  // Rotate and scale the scene node of the orientation part
  orientation_shape_[index]->setOrientation(shape_orientation);
  if (!shape_scale.isNaN())
    orientation_shape_[index]->setScale(shape_scale);
  else
    ROS_WARN_STREAM("orientation shape_scale contains NaN: " << shape_scale);
}

void CovarianceVisual::setScales(float pos_scale, float ori_scale)
{
  setPositionScale(pos_scale);
  setOrientationScale(ori_scale);
}

void CovarianceVisual::setPositionScale(float pos_scale)
{
  if (pose_2d_)
    position_scale_node_->setScale(pos_scale, pos_scale, 1.0);
  else
    position_scale_node_->setScale(pos_scale, pos_scale, pos_scale);
}

void CovarianceVisual::setOrientationOffset(float ori_offset)
{
  // Scale the orientation root node to position the shapes along the axis
  orientation_root_node_->setScale(ori_offset, ori_offset, ori_offset);
  // The scale the offset_nodes as well so the displayed shape represents a 1-sigma
  // standard deviation when displayed with an scale of 1.0
  // NOTE: We only want to change the scales of the dimentions that represent the
  //       orientation covariance. The other dimensions are set to 1.0.
  for (int i = 0; i < kNumOriShapes; i++)
  {
    if (i == kYaw2D)
    {
      // For 2D, the angle is only encoded on x, but we also scale on y to put the top of the cone at the
      // pose origin
      orientation_offset_node_[i]->setScale(ori_offset, ori_offset, 1.0);
    }
    else
    {
      // For 3D, the angle covariance is encoded on x and z dimensions
      orientation_offset_node_[i]->setScale(ori_offset, 1.0, ori_offset);
    }
  }
}

void CovarianceVisual::setOrientationScale(float ori_scale)
{
  // Here we update the current scale factor, apply it to the current scale _in radians_,
  // convert it to meters and apply to the shape scale. Note we have different invariant
  // scales in the 3D and in 2D.
  current_ori_scale_factor_ = ori_scale;
  for (int i = 0; i < kNumOriShapes; i++)
  {
    // Recover the last computed scale
    Ogre::Vector3 shape_scale = current_ori_scale_[i];
    if (i == kYaw2D)
    {
      // Changes in scale in 2D only affects the x dimension
      // Apply the current scale factor
      shape_scale.x *= current_ori_scale_factor_;
      // Convert from radians to meters
      radianScaleToMetricScaleBounded(shape_scale.x, max_degrees);
    }
    else
    {
      // Changes in scale in 3D only affects the x and z dimensions
      // Apply the current scale factor
      shape_scale.x *= current_ori_scale_factor_;
      shape_scale.z *= current_ori_scale_factor_;
      // Convert from radians to meters
      radianScaleToMetricScaleBounded(shape_scale.x, max_degrees);
      radianScaleToMetricScaleBounded(shape_scale.z, max_degrees);
    }
    // Apply the new scale
    orientation_shape_[i]->setScale(shape_scale);
  }
}

void CovarianceVisual::setPositionColor(const Ogre::ColourValue& c)
{
  position_shape_->setColor(c);
}

void CovarianceVisual::setOrientationColor(const Ogre::ColourValue& c)
{
  for (int i = 0; i < kNumOriShapes; i++)
  {
    orientation_shape_[i]->setColor(c);
  }
}

void CovarianceVisual::setOrientationColorToRGB(float a)
{
  orientation_shape_[kRoll]->setColor(Ogre::ColourValue(1.0, 0.0, 0.0, a));
  orientation_shape_[kPitch]->setColor(Ogre::ColourValue(0.0, 1.0, 0.0, a));
  orientation_shape_[kYaw]->setColor(Ogre::ColourValue(0.0, 0.0, 1.0, a));
  orientation_shape_[kYaw2D]->setColor(Ogre::ColourValue(0.0, 0.0, 1.0, a));
}

void CovarianceVisual::setPositionColor(float r, float g, float b, float a)
{
  setPositionColor(Ogre::ColourValue(r, g, b, a));
}

void CovarianceVisual::setOrientationColor(float r, float g, float b, float a)
{
  setOrientationColor(Ogre::ColourValue(r, g, b, a));
}

const Ogre::Vector3& CovarianceVisual::getPositionCovarianceScale()
{
  return position_node_->getScale();
}

const Ogre::Quaternion& CovarianceVisual::getPositionCovarianceOrientation()
{
  return position_node_->getOrientation();
}

void CovarianceVisual::setUserData(const Ogre::Any& data)
{
  position_shape_->setUserData(data);
  for (int i = 0; i < kNumOriShapes; i++)
  {
    orientation_shape_[i]->setUserData(data);
  }
}

void CovarianceVisual::setVisible(bool visible)
{
  setPositionVisible(visible);
  setOrientationVisible(visible);
}

void CovarianceVisual::setPositionVisible(bool visible)
{
  position_node_->setVisible(visible);
}

void CovarianceVisual::setOrientationVisible(bool visible)
{
  orientation_visible_ = visible;
  updateOrientationVisibility();
}

void CovarianceVisual::updateOrientationVisibility()
{
  orientation_offset_node_[kRoll]->setVisible(orientation_visible_ && !pose_2d_);
  orientation_offset_node_[kPitch]->setVisible(orientation_visible_ && !pose_2d_);
  orientation_offset_node_[kYaw]->setVisible(orientation_visible_ && !pose_2d_);
  orientation_offset_node_[kYaw2D]->setVisible(orientation_visible_ && pose_2d_);
}


const Ogre::Vector3& CovarianceVisual::getPosition()
{
  return position_node_->getPosition();
}

const Ogre::Quaternion& CovarianceVisual::getOrientation()
{
  return position_node_->getOrientation();
}

void CovarianceVisual::setPosition(const Ogre::Vector3& position)
{
  root_node_->setPosition(position);
}

void CovarianceVisual::setOrientation(const Ogre::Quaternion& orientation)
{
  root_node_->setOrientation(orientation);
}

void CovarianceVisual::setRotatingFrame(bool is_local_rotation)
{
  if (local_rotation_ == is_local_rotation)
    return;

  local_rotation_ = is_local_rotation;

  if (local_rotation_)
    root_node_->addChild(fixed_orientation_node_->removeChild(orientation_root_node_->getName()));
  else
    fixed_orientation_node_->addChild(root_node_->removeChild(orientation_root_node_->getName()));
}

rviz::Shape* CovarianceVisual::getOrientationShape(ShapeIndex index)
{
  return orientation_shape_[index];
}

} // namespace rviz
