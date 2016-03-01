#include "covariance_visual.h"

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <ros/console.h>

#include <sstream>

#include <Eigen/Dense>

namespace rviz
{

CovarianceVisual::CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_visible, float pos_scale, float ori_scale)
: Object( scene_manager ),
  position_scale_factor_( 1.0f ), orientation_scale_factor_( 1.0f ),
  position_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f)),
  orientation_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f))
{
  frame_node_ = parent_node->createChildSceneNode();

  position_node_ = frame_node_->createChildSceneNode();
  position_shape_ = new Shape(Shape::Sphere, scene_manager_, position_node_);

  orientation_node_ = frame_node_->createChildSceneNode();
  orientation_shape_ = new Shape(Shape::Cone, scene_manager_, orientation_node_);

  setVisible( is_visible );

  setScales( pos_scale, ori_scale );
}

CovarianceVisual::~CovarianceVisual()
{
  delete position_shape_;
  delete orientation_shape_;

  scene_manager_->destroySceneNode( position_node_->getName() );
  scene_manager_->destroySceneNode( orientation_node_->getName() );
  scene_manager_->destroySceneNode( frame_node_->getName() );
}

// Local function to force the axis to be right handed. Taken from ecl_statistics
void makeRightHanded( Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being righ-handed and normalised.
  Eigen::Vector3d c0 = eigenvectors.block<3,1>(0,0);  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.block<3,1>(0,1);  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.block<3,1>(0,2);  c2.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc.dot(c2) < 0) {
    eigenvectors << c1, c0, c2;
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0, c1, c2;
  }
}

void computeShapeScaleAndOrientation(const Eigen::Matrix3d& covariance, Ogre::Vector3& scale, Ogre::Quaternion& orientation)
{
  Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
  Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  // FIXME: Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info () == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector3d::Zero();      // Setting the scale to zero will hide it on the screen
    eigenvectors = Eigen::Matrix3d::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  // Define the rotation
  orientation.FromRotationMatrix(Ogre::Matrix3(eigenvectors(0,0), eigenvectors(0,1), eigenvectors(0,2),
                                               eigenvectors(1,0), eigenvectors(1,1), eigenvectors(1,2),
                                               eigenvectors(2,0), eigenvectors(2,1), eigenvectors(2,2)));

  // Define the scale. eigenvalues are the variances, so we take the sqrt to draw the standard deviation
  scale.x = 2*std::sqrt (eigenvalues[0]);
  scale.y = 2*std::sqrt (eigenvalues[1]);
  scale.z = 2*std::sqrt (eigenvalues[2]);
}

// This method compute the eigenvalues and eigenvectors of the position and orientation part covariance matrix
// separatelly and use their values to rotate and scale the covarance shapes.
void CovarianceVisual::setCovariance( const geometry_msgs::PoseWithCovariance& message )
{
  // check for NaN in covariance
  for (unsigned i = 0; i < 3; ++i)
  {
      if(isnan(message.covariance[i]))
      {
          ROS_WARN_THROTTLE(1, "covariance contains NaN");
          return;
      }
  }

  // store pose in Ogre structures
  Ogre::Vector3 msg_position(message.pose.position.x,message.pose.position.y,message.pose.position.z);
  Ogre::Quaternion msg_orientation(message.pose.orientation.w, message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z);

  Eigen::Map<const Eigen::Matrix<double,6,6> > covariance(message.covariance.data());

  // Compute shape and orientation for the position part of covariance
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  computeShapeScaleAndOrientation(covariance.topLeftCorner<3,3>(), shape_scale, shape_orientation);
  // store the shape (needed if the scale factor changes later)
  (*position_msg_scale_) = shape_scale;
  // update the shape by the scale factor
  shape_scale *= position_scale_factor_;

  // Position, rotate and scale the scene node of the position part
  position_node_->setPosition(msg_position);
  position_node_->setOrientation(shape_orientation);
  if(!shape_scale.isNaN())
      position_node_->setScale(shape_scale);
  else
      ROS_WARN_STREAM("position shape_scale contains NaN: " << shape_scale);

  // Repeat the same for the orientation part of the covariance matrix
  computeShapeScaleAndOrientation(covariance.bottomRightCorner<3,3>(), shape_scale, shape_orientation);
  (*orientation_msg_scale_) = shape_scale;
  shape_scale *= orientation_scale_factor_;

  // Position, rotate and scale the scene node of the orientation part
  // Note the shape_orientation is composed with the msg_orientation
  orientation_node_->setPosition(msg_position);
  orientation_node_->setOrientation(msg_orientation * shape_orientation);
  if(!shape_scale.isNaN())
      orientation_node_->setScale(shape_scale);
  else
      ROS_WARN_STREAM("orientation shape_scale contains NaN: " << shape_scale);
}

void CovarianceVisual::setScales( float pos_scale, float ori_scale)
{
  setPositionScale(pos_scale);
  setOrientationScale(ori_scale);
}

void CovarianceVisual::setPositionScale( float pos_scale ) 
{
  position_scale_factor_ = pos_scale;
  position_node_->setScale((*position_msg_scale_) * position_scale_factor_);
}

void CovarianceVisual::setOrientationScale( float ori_scale )
{
  orientation_scale_factor_ = ori_scale;
  orientation_node_->setScale((*orientation_msg_scale_) * orientation_scale_factor_);
}

void CovarianceVisual::setPositionColor(const Ogre::ColourValue& c)
{
  position_shape_->setColor(c);
}

void CovarianceVisual::setOrientationColor(const Ogre::ColourValue& c)
{
  orientation_shape_->setColor(c);
}

void CovarianceVisual::setPositionColor( float r, float g, float b, float a )
{
  setPositionColor( Ogre::ColourValue(r, g, b, a ));
}

void CovarianceVisual::setOrientationColor( float r, float g, float b, float a )
{
  setOrientationColor( Ogre::ColourValue(r, g, b, a ));
}

const Ogre::Vector3& CovarianceVisual::getPositionCovarianceScale()
{
  return position_node_->getScale();
}

const Ogre::Quaternion& CovarianceVisual::getPositionCovarianceOrientation()
{
  return position_node_->getOrientation();
}

const Ogre::Vector3& CovarianceVisual::getOrientationCovarianceScale()
{
  return orientation_node_->getScale();
}

const Ogre::Quaternion& CovarianceVisual::getOrientationCovarianceOrientation()
{
  return orientation_node_->getOrientation();
}

void CovarianceVisual::setUserData( const Ogre::Any& data )
{
  position_shape_->setUserData( data );
  orientation_shape_->setUserData( data );
}

void CovarianceVisual::setVisible( bool visible )
{
  frame_node_->setVisible( visible );
}

const Ogre::Vector3& CovarianceVisual::getPosition() 
{
  return position_node_->getPosition();
}

const Ogre::Quaternion& CovarianceVisual::getOrientation()
{
  return position_node_->getOrientation();
}

void CovarianceVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void CovarianceVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

} // namespace rviz

