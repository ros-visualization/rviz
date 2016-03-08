#ifndef COVARIANCE_VISUAL_H
#define COVARIANCE_VISUAL_H

#include "rviz/ogre_helpers/object.h"

#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/PoseWithCovariance.h>

#include <Eigen/Dense>

#include <OgreColourValue.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
}

namespace Eigen
{
  typedef Matrix<double,6,6> Matrix6d;
}

namespace rviz
{
class Shape;
class CovarianceProperty;

/**
 * \class CovarianceVisual
 * \brief CovarianceVisual consisting in a ellipse for position and 2D ellipses along the axis for orientation.
 */
class CovarianceVisual : public Object
{
public:
  enum ShapeIndex
  {
    kRoll=0,
    kPitch=1,
    kYaw=2,
    kYaw2D=3,
    kNumOriShapes
  };

private:
  /**
   * \brief Private Constructor
   * 
   * CovarianceVisual can only be constructed by friend class CovarianceProperty.
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_object A rviz object that this covariance will be attached.
   * @param is_local_rotation Initial attachment of the rotation part
   * @param is_visible Initial visibility
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_local_rotation, bool is_visible = true, float pos_scale = 1.0f, float ori_scale = 0.1f);
public:
  virtual ~CovarianceVisual();

  /**
   * \brief Set the position and orientation scales for this covariance
   *
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  void setScales( float pos_scale, float ori_scale);
  void setPositionScale( float pos_scale );
  void setOrientationScale( float ori_scale );

  /**
   * \brief Set the color of the position covariance. Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  virtual void setPositionColor( float r, float g, float b, float a );
  void setPositionColor(const Ogre::ColourValue& color);

  /**
   * \brief Set the color of the orientation covariance. Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  virtual void setOrientationColor( float r, float g, float b, float a );
  void setOrientationColor(const Ogre::ColourValue& color);
  void setOrientationColorToRGB(float a);

  /** @brief Set the covariance.
   *
   * This effectively changes the orientation and scale of position and orientation 
   * covariance shapes
   */
  virtual void setCovariance( const geometry_msgs::PoseWithCovariance& pose );

  virtual const Ogre::Vector3& getPositionCovarianceScale();
  virtual const Ogre::Quaternion& getPositionCovarianceOrientation();

  /**
   * \brief Get the root scene node of the position part of this covariance
   * @return the root scene node of the position part of this covariance
   */
  Ogre::SceneNode* getPositionSceneNode() { return position_scale_node_; }

  /**
   * \brief Get the root scene node of the orientation part of this covariance
   * @return the root scene node of the orientation part of this covariance
   */
  Ogre::SceneNode* getOrientationSceneNode() { return orientation_scale_node_; }

  /**
   * \brief Get the shape used to display position covariance
   * @return the shape used to display position covariance
   */
  Shape* getPositionShape() { return position_shape_; }

  /**
   * \brief Get the shape used to display orientation covariance in an especific axis
   * @return the shape used to display orientation covariance in an especific axis
   */  
  Shape* getOrientationShape(ShapeIndex index);

  /**
   * \brief Sets user data on all ogre objects we own
   */
  virtual void setUserData( const Ogre::Any& data );

  /**
   * \brief Sets visibility of this covariance
   * 
   * Convenience method that sets visibility of both position and orientation parts.
   */
  virtual void setVisible( bool visible );

  /**
   * \brief Sets visibility of the position part of this covariance
   */
  virtual void setPositionVisible( bool visible );

  /**
   * \brief Sets visibility of the orientation part of this covariance
   */
  virtual void setOrientationVisible( bool visible );

  /**
   * \brief Sets position of the frame this covariance is attached
   */
  virtual void setPosition( const Ogre::Vector3& position );

  /**
   * \brief Sets orientation of the frame this covariance is attached
   */
  virtual void setOrientation( const Ogre::Quaternion& orientation );

  /**
   * \brief Sets which frame to attach the covariance of the orientation
   */
  virtual void setRotatingFrame( bool use_rotating_frame );

private:
  void updatePosition( const Eigen::Matrix6d& covariance );
  void updateOrientation( const Eigen::Matrix6d& covariance, ShapeIndex index );
  void updateOrientationVisibility();

  Ogre::SceneNode* root_node_;
  Ogre::SceneNode* fixed_orientation_node_;
  Ogre::SceneNode* position_scale_node_;
  Ogre::SceneNode* position_node_;

  Ogre::SceneNode* orientation_scale_node_;
  Ogre::SceneNode* orientation_offset_node_[kNumOriShapes];
  Ogre::SceneNode* orientation_node_[kNumOriShapes];

  Shape* position_shape_;   ///< Ellipse used for the position covariance
  Shape* orientation_shape_[kNumOriShapes];   ///< Cylinders used for the orientation covariance

  bool local_rotation_;

  bool pose_2d_;

  bool orientation_visible_; ///< If the orientation component is visible.

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  virtual void setScale( const Ogre::Vector3& scale ) {};
  virtual void setColor( float r, float g, float b, float a ) {};
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

  // Make CovarianceProperty friend class so it create CovarianceVisual objects
  friend class CovarianceProperty;
};

} // namespace rviz

#endif /* COVARIANCE_VISUAL_H */
