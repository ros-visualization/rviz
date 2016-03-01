#ifndef COVARIANCE_VISUAL_H
#define COVARIANCE_VISUAL_H

#include "rviz/ogre_helpers/object.h"

#include <boost/scoped_ptr.hpp>
#include <boost/array.hpp>

#include <geometry_msgs/PoseWithCovariance.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;
class ColourValue;
class Any;
}

namespace rviz
{
class Shape;

/**
 * \class CovarianceVisual
 * \brief CovarianceVisual consisting in a sphere for position and cone (?) for orientation.
 */
class CovarianceVisual : public Object
{
public:
  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_object A rviz object that this covariance will be attached.
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_visible = true, float pos_scale = 1.0f, float ori_scale = 0.1f);
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

  /** @brief Set the covariance.
   *
   * This effectively changes the orientation and scale of position and orientation 
   * covariance shapes
   */
  virtual void setCovariance( const geometry_msgs::PoseWithCovariance& message );

  virtual const Ogre::Vector3& getPositionCovarianceScale();
  virtual const Ogre::Quaternion& getPositionCovarianceOrientation();
  virtual const Ogre::Vector3& getOrientationCovarianceScale();
  virtual const Ogre::Quaternion& getOrientationCovarianceOrientation();

  /**
   * \brief Get the scene node the frame this covariance is defined
   * @return the scene node associated with frame this covariance is defined
   */
  Ogre::SceneNode* getFrameSceneNode() { return frame_node_; }

  /**
   * \brief Get the scene node associated with the position covariance
   * @return the scene node associated with the position covariance
   */
  Ogre::SceneNode* getPositionSceneNode() { return position_node_; }

  /**
   * \brief Get the scene node associated with the orientation covariance
   * @return the scene node associated with the orientation covariance
   */
  Ogre::SceneNode* getOrientationSceneNode() { return orientation_node_; }

  Shape* getPositionShape() { return position_shape_; }
  Shape* getOrientationShape() { return orientation_shape_; }

  /**
   * \brief Sets user data on all ogre objects we own
   */
  virtual void setUserData( const Ogre::Any& data );

  /**
   * \brief Sets visibility of this covariance
   *
   * This convenience function sets the visibility of the both position and orientation
   * scene nodes 
   */
  virtual void setVisible( bool visible );

  /**
   * \brief Sets position of the frame this covariance is attached
   */
  virtual void setFramePosition( const Ogre::Vector3& position );

  /**
   * \brief Sets orientation of the frame this covariance is attached
   */
  virtual void setFrameOrientation( const Ogre::Quaternion& orientation );


private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneNode* position_node_;
  Ogre::SceneNode* orientation_node_;

  Shape* position_shape_;   ///< Ellipse used for the position covariance
  Shape* orientation_shape_;   ///< Cone used for the orientation covariance

  float position_scale_factor_;
  float orientation_scale_factor_;

  boost::scoped_ptr<Ogre::Vector3> position_msg_scale_;
  boost::scoped_ptr<Ogre::Vector3> orientation_msg_scale_;

// Hide Object methods we don't want to expose
// NOTE: Apparently we still need to define them...
private:
  virtual void setPosition( const Ogre::Vector3& position ) {};
  virtual void setOrientation( const Ogre::Quaternion& orientation ) {};
  virtual void setScale( const Ogre::Vector3& scale ) {};
  virtual void setColor( float r, float g, float b, float a ) {};
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

};

} // namespace rviz_plugin_covariance

#endif /* COVARIANCE_VISUAL_H */
