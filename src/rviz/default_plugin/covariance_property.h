#ifndef COVARIANCE_PROPERTY_H
#define COVARIANCE_PROPERTY_H

#include <QColor>

#include <OgreColourValue.h>

#include "rviz/properties/bool_property.h"

namespace Ogre
{
  class SceneManager;
  class SceneNode;
}

namespace rviz
{

class Property;
class ColorProperty;
class FloatProperty;
class EnumProperty;
class CovarianceVisual;

/** @brief Property specialized to provide getter for booleans. */
class CovarianceProperty: public BoolProperty
{
Q_OBJECT
public:
  typedef boost::shared_ptr<CovarianceVisual> CovarianceVisualPtr;

  enum Frame
  {
    Local,
    Fixed,
  };

  enum ColorStyle
  {
    Unique,
    RGB,
  };

  CovarianceProperty( const QString& name = "Covariance",
                bool default_value = false,
                const QString& description = QString(),
                Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual ~CovarianceProperty();

  bool getPositionBool();
  bool getOrientationBool();

  // Methods to manage the deque of Covariance Visuals
  CovarianceVisualPtr createAndPushBackVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  void popFrontVisual();
  void clearVisual();
  size_t sizeVisual();

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateColorAndAlphaAndScale();
  void updateOrientationFrame();
  void updateColorStyleChoice();

private:
  void updateColorAndAlphaAndScale( const CovarianceVisualPtr& visual );
  void updateOrientationFrame( const CovarianceVisualPtr& visual );
  void updateVisibility( const CovarianceVisualPtr& visual );

  typedef std::deque<CovarianceVisualPtr> D_Covariance;
  D_Covariance covariances_;

  BoolProperty*  position_property_;
  ColorProperty* position_color_property_;
  FloatProperty* position_alpha_property_;
  FloatProperty* position_scale_property_;
  BoolProperty*  orientation_property_;
  EnumProperty*  orientation_frame_property_;
  EnumProperty*  orientation_colorstyle_property_;
  ColorProperty* orientation_color_property_;
  FloatProperty* orientation_alpha_property_;
  FloatProperty* orientation_scale_property_;
};

} // end namespace rviz

#endif // COVARIANCE_PROPERTY_H
